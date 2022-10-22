#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Rmagine deps
#include <rmagine/map/OptixMap.hpp>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/math.cuh>
#include <rmagine/util/prints.h>

// RCML msgs
#include <rmcl_msgs/ScanStamped.h>

// RMCL code
#include <rmcl/correction/SphereCorrectorOptixROS.hpp>
#include <rmcl/util/conversions.h>
#include <rmcl/util/scan_operations.h>

#include <chrono>
#include <memory>
#include <omp.h>
#include <thread>
#include <mutex>

using namespace rmcl;
using namespace rmcl_msgs;
using namespace rmagine;

SphereCorrectorOptixROSPtr  scan_correct;
CorrectionParams            corr_params;

float max_distance;

bool adaptive_max_dist = false;
float adaptive_max_dist_min = 0.15;

bool        pose_received = false;
ros::Time   last_pose;
bool        scan_received = false;
ros::Time   last_scan;
size_t      valid_scan_ranges;

std::string map_frame;
std::string odom_frame;
bool has_odom_frame = true;
std::string base_frame;
bool has_base_frame = true;
std::string sensor_frame;


// for testing
size_t Nposes = 100;

std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener; 

// Estimate this
geometry_msgs::TransformStamped T_odom_map;
Transform                       Tom;
std::mutex                      T_odom_map_mutex;
// dynamic: ekf
geometry_msgs::TransformStamped T_base_odom;
Transform                       Tbo;
// static: urdf
geometry_msgs::TransformStamped T_sensor_base;
Transform                       Tsb;


std::thread correction_thread;
bool stop_correction_thread = false;


/**
 * @brief Update T_sensor_base and T_base_odom globally
 */
bool fetchTF()
{
    bool ret = true;

    if(has_base_frame)
    {
        try{
            T_sensor_base = tfBuffer->lookupTransform(base_frame, sensor_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ROS_WARN_STREAM("Source (Sensor): " << sensor_frame << ", Target (Base): " << base_frame);

            ret = false;
        }
    } else {
        T_sensor_base.header.frame_id = base_frame;
        T_sensor_base.child_frame_id = sensor_frame;
        T_sensor_base.transform.translation.x = 0.0;
        T_sensor_base.transform.translation.y = 0.0;
        T_sensor_base.transform.translation.z = 0.0;
        T_sensor_base.transform.rotation.x = 0.0;
        T_sensor_base.transform.rotation.y = 0.0;
        T_sensor_base.transform.rotation.z = 0.0;
        T_sensor_base.transform.rotation.w = 1.0;
    }
    convert(T_sensor_base.transform, Tsb);
    scan_correct->setTsb(T_sensor_base.transform);
    
    if(has_odom_frame && has_base_frame)
    {
        try{
            T_base_odom = tfBuffer->lookupTransform(odom_frame, base_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ROS_WARN_STREAM("Source (Base): " << base_frame << ", Target (Odom): " << odom_frame);
            ret = false;
        }
    } else {
        T_base_odom.header.frame_id = odom_frame;
        T_base_odom.child_frame_id = base_frame;
        T_base_odom.transform.translation.x = 0.0;
        T_base_odom.transform.translation.y = 0.0;
        T_base_odom.transform.translation.z = 0.0;
        T_base_odom.transform.rotation.x = 0.0;
        T_base_odom.transform.rotation.y = 0.0;
        T_base_odom.transform.rotation.z = 0.0;
        T_base_odom.transform.rotation.w = 1.0;
    }
    convert(T_base_odom.transform, Tbo);

    return ret;
}

void correctOnce()
{
    std::lock_guard<std::mutex> guard(T_odom_map_mutex);
    // ROS_INFO("Correction started.");
    // StopWatch sw;
    // 1. Get Base in Map
    Transform Tbm = Tom * Tbo;
    
    
    Memory<Transform, RAM> poses(Nposes);
    for(size_t i=0; i<Nposes; i++)
    {
        poses[i] = Tbm;
    }
    // convert(T_base_map.transform, poses[0]);
    // upload to GPU
    Memory<Transform, VRAM_CUDA> poses_;
    poses_ = poses;
    // sw();
    auto corrRes = scan_correct->correct(poses_);

    
    
    // Tdelta -> T_base_new_base_old
    // T_base_new_to_map = T_base_old_map * T_base_new_base_old
    poses_ = multNxN(poses_, corrRes.Tdelta);
    // not: P = Tdelta * P 
    // download to CPU
    poses = poses_;



    Memory<Transform, RAM> Tdelta0 = corrRes.Tdelta(0,1);
    Memory<unsigned int, RAM> Ncorr0 = corrRes.Ncorr(0,1);


    Transform Td = Tdelta0[0];
    float trans_force = Td.t.l2norm();
    float trans_progress = 1.0 / exp(10.0 * trans_force);

    Quaternion qunit;
    qunit.setIdentity();
    float qscalar = Td.R.dot(qunit);
    float rot_progress = qscalar * qscalar;

    
    float match_ratio = static_cast<float>(Ncorr0[0]) / static_cast<float>(valid_scan_ranges);

    

    // std::cout << "Correction Stats:" << std::endl;
    // std::cout << "- match ratio: " << match_ratio << std::endl;
    // std::cout << "- trans progress: " << trans_progress << std::endl;
    // std::cout << "- rot progress:   " << rot_progress << std::endl;
    // std::cout << "- magic_number: " << magic_number << std::endl;

    if(adaptive_max_dist)
    {
        float adaption_rate = trans_progress * rot_progress * match_ratio;
        corr_params.max_distance = max_distance + (adaptive_max_dist_min - max_distance) * adaption_rate;
        // std::cout << "Adjusting max distance to: " << corr_params.max_distance << std::endl;
    }
    
    

    scan_correct->setParams(corr_params);

    // Update T_odom_map
    Tom = poses[0] * ~Tbo;
}

void correct()
{
    if(pose_received && scan_received)
    {
        fetchTF();
        correctOnce();
    }
}


// Storing Pose information globally
// Calculate transformation from map to odom from pose in map frame
void poseCB(geometry_msgs::PoseStamped msg)
{
    std::lock_guard<std::mutex> guard(T_odom_map_mutex);

    ROS_INFO_STREAM_NAMED(ros::this_node::getName(), ros::this_node::getName() << " Received new pose guess");

    // rest max distance
    corr_params.max_distance = max_distance;
    scan_correct->setParams(corr_params);


    map_frame = msg.header.frame_id;
    pose_received = true;

    // msg.pose.position.z -= 1.0;

    // set T_base_map
    Transform Tbm;
    convert(msg.pose, Tbm);

    fetchTF();
    
    Tom = Tbm * ~Tbo;
}

void poseWcCB(geometry_msgs::PoseWithCovarianceStamped msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    poseCB(pose);
}


// Storing scan information globally
// updating real data inside the global scan corrector
void scanCB(const ScanStamped::ConstPtr& msg)
{
    sensor_frame = msg->header.frame_id;
    scan_correct->setModelAndInputData(msg->scan);

    // count valid
    valid_scan_ranges = 0;
    for(size_t i=0; i<msg->scan.data.ranges.size(); i++)
    {
        if(msg->scan.data.ranges[i] >= msg->scan.info.range_min 
        && msg->scan.data.ranges[i] <= msg->scan.info.range_max)
        {
            valid_scan_ranges++;
        }
    }

    last_scan = msg->header.stamp;
    scan_received = true;
}


void updateTF()
{
    // std::cout << "updateTF" << std::endl;
    static tf2_ros::TransformBroadcaster br;
    
    geometry_msgs::TransformStamped T;

    // What is the source frame?
    if(has_odom_frame && has_base_frame)
    {
        // With EKF and base_frame: Send odom to map
        convert(Tom, T.transform);
        T.header.frame_id = map_frame;
        T.child_frame_id = odom_frame;
    } else if(has_base_frame) {
        // With base but no EKF: send base to map
        auto Tbm = Tom * Tbo;
        convert(Tbm, T.transform);
        T.header.frame_id = map_frame;
        T.child_frame_id = base_frame;
    } else {
        // Default:
        // Sensor to map
        auto Tbm = Tom * Tbo * Tsb;
        convert(Tbm, T.transform);
        T.header.frame_id = map_frame;
        T.child_frame_id = sensor_frame;
    }

    T.header.stamp = ros::Time::now();
    br.sendTransform(T);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_corrector_optix");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO_STREAM_NAMED(ros::this_node::getName(), ros::this_node::getName() << " started");

    std::string map_frame;
    std::string meshfile;

    nh_p.param<std::string>("map_file", meshfile, "/home/amock/ros_workspaces/amcl_flex/avz_floor.ply");

    // Default minimal setting: Map and Sensor
    nh_p.param<std::string>("odom_frame", odom_frame, "");
    nh_p.param<std::string>("base_frame", base_frame, "");

    if(base_frame == "")
    {
        has_base_frame = false;
    }

    if(odom_frame == "")
    {
        has_odom_frame = false;
    }

    double tf_rate;
    nh_p.param<double>("tf_rate", tf_rate, 30);

    double corr_rate_max;
    nh_p.param<double>("corr_rate_max", corr_rate_max, 30);

    int Nposes_tmp;
    nh_p.param<int>("poses", Nposes_tmp, 1);
    Nposes = Nposes_tmp;



    OptixMapPtr map = import_optix_map(meshfile);
    scan_correct = std::make_shared<SphereCorrectorOptixROS>(map);
    

    
    nh_p.param<float>("max_distance", max_distance, 0.8);
    nh_p.param<bool>("adaptive_max_dist", adaptive_max_dist, false);
    nh_p.param<float>("adaptive_max_dist_min", adaptive_max_dist_min, 0.15);



    corr_params.max_distance = max_distance;
    scan_correct->setParams(corr_params);

    std::cout << "Max Distance: " << corr_params.max_distance << std::endl;

    // get TF of scanner
    tfBuffer.reset(new tf2_ros::Buffer);
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

    ros::Subscriber sub = nh.subscribe<ScanStamped>("scan", 1, scanCB);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, poseCB);
    ros::Subscriber pose_wc_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_wc", 1, poseWcCB);


    ROS_INFO_STREAM_NAMED(ros::this_node::getName(), ros::this_node::getName() << ": Open RViz. Set fixed frame to map frame. Set goal. ICP to Mesh");


    // CORRECTION THREAD
    stop_correction_thread = false;
    correction_thread = std::thread([corr_rate_max](){
        StopWatch sw;
        double el;

        // minimum duration for one loop
        double el_min = 1.0 / corr_rate_max;

        while(!stop_correction_thread)
        {
            sw();
            correct();
            el = sw();
            double el_left = el_min - el;
            if(el_left > 0.0)
            {
                std::this_thread::sleep_for(std::chrono::duration<double>(el_left));
            }
            // std::cout << "Current Correction Rate: " << 1.0 / el << std::endl;
        }

        stop_correction_thread = false;
    });


    // MAIN LOOP (TF)
    ros::Rate r(tf_rate);
    ros::Time stamp = ros::Time::now();

    while(ros::ok())
    {
        if(pose_received && scan_received)
        {
            // updateTF();
            // weird bug. new_stamp sometimes is equal to stamp. results 
            
            ros::Time new_stamp = ros::Time::now();
            if(new_stamp > stamp)
            {
                updateTF();
                stamp = new_stamp;
            }
        }
        
        r.sleep();
        ros::spinOnce();
    }

    stop_correction_thread = true;
    correction_thread.join();
    
    return 0;
}