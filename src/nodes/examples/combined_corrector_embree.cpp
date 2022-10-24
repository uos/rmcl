#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Rmagine deps
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/util/prints.h>

// RCML msgs
#include <rmcl_msgs/ScanStamped.h>

// RMCL code
#include <rmcl/correction/OnDnCorrectorEmbreeROS.hpp>
#include <rmcl/correction/SphereCorrectorEmbreeROS.hpp>
#include <rmcl/util/conversions.h>
#include <rmcl/util/scan_operations.h>
#include <rmcl/math/math.h>

#include <chrono>
#include <memory>
#include <omp.h>
#include <thread>
#include <mutex>

using namespace rmcl;
using namespace rmcl_msgs;
using namespace rmagine;

SphereCorrectorEmbreeROSPtr scan_correct;
OnDnCorrectorEmbreeROSPtr ondn_correct;
CorrectionParams corr_params;

float max_distance;

bool adaptive_max_dist = false;
float adaptive_max_dist_min = 0.15;

ros::Publisher model_pub;

OnDnModel wheel_model;

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
size_t Nposes = 1;


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


void publish_model(const OnDnModel& model)
{
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = base_frame;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.ns = "";
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.01;

    float scale = 0.1;

    for(size_t i=0; i<model.getWidth(); i++)
    {
        Vector orig = model.getOrigin(0, i);
        Vector dir = model.getDirection(0, i);
        
        Vector end = orig + dir * scale;
        geometry_msgs::Point orig_ros, end_ros;
        orig_ros.x = orig.x;
        orig_ros.y = orig.y;
        orig_ros.z = orig.z;

        marker.points.push_back(orig_ros);

        end_ros.x = end.x;
        end_ros.y = end.y;
        end_ros.z = end.z;

        marker.points.push_back(end_ros);
    }

    model_pub.publish(marker);
}

/**
 * @brief Update T_sensor_base and T_base_odom globally
 */
bool fetchTF()
{
    bool ret = true;

    if(has_base_frame)
    {
        try {
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

    Transform identity = Transform::Identity();
    ondn_correct->setTsb(identity);
    scan_correct->setTsb(T_sensor_base.transform);
    
    if(has_odom_frame && has_base_frame)
    {
        try{
            T_base_odom = tfBuffer->lookupTransform(odom_frame, base_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ROS_WARN_STREAM("Source: " << odom_frame << ", Target: " << base_frame);
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

    // std::cout << "correctOnce" << std::endl;
    // 1. Get Base in Map
    Transform Tbm = Tom * Tbo;

    Memory<Transform, RAM> poses(Nposes);
    for(size_t i=0; i<Nposes; i++)
    {
        poses[i] = Tbm;
    }
    
    // Extra memory for laser (_l) and wheels (_w)

    auto laser_covs = scan_correct->computeCovs(poses);
    auto wheel_covs = ondn_correct->computeCovs(poses);
    auto merged_covs = weighted_average({laser_covs, wheel_covs}, {0.5, 0.5});
    auto Tdelta = Correction()(merged_covs);

    poses = multNxN(poses, Tdelta);


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

    map_frame = msg.header.frame_id;
    pose_received = true;

    // set T_base_map
    Transform Tbm;
    convert(msg.pose, Tbm);

    fetchTF();

    Tom = Tbm * ~Tbo;
}

// Storing scan information globally
// updating real data inside the global scan corrector
void scanCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_frame = msg->header.frame_id;

    fetchTF();

    SphericalModel laser_model;
    convert(*msg, laser_model);

    scan_correct->setModel(laser_model);
    scan_correct->setInputData(msg->ranges);
    
    publish_model(wheel_model);

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

void genWheelModel()
{
    float wheel_dist_front = 0.14;
    float wheel_dist_left = 0.188;
    float wheel_radius = 0.135;

    
    wheel_model.width = 4;
    wheel_model.height = 1;
    wheel_model.range.min = 0.0;
    wheel_model.range.max = wheel_radius * 2.0;

    // center of four wheels
    wheel_model.origs.resize(4);
    wheel_model.origs[0] = {wheel_dist_front, wheel_dist_left, wheel_radius}; // front left
    wheel_model.origs[1] = {wheel_dist_front, -wheel_dist_left, wheel_radius}; // front right
    wheel_model.origs[2] = {-wheel_dist_front, wheel_dist_left, wheel_radius}; // rear left
    wheel_model.origs[3] = {-wheel_dist_front, -wheel_dist_left, wheel_radius}; // rear right
    

    // direction pointing towards ground
    wheel_model.dirs.resize(4);
    wheel_model.dirs[0] = {0.0, 0.0, -1.0};
    wheel_model.dirs[1] = {0.0, 0.0, -1.0};
    wheel_model.dirs[2] = {0.0, 0.0, -1.0};
    wheel_model.dirs[3] = {0.0, 0.0, -1.0};

    ondn_correct->setModel(wheel_model);

    Memory<float> ranges_wheels(4);
    ranges_wheels[0] = wheel_radius;
    ranges_wheels[1] = wheel_radius;
    ranges_wheels[2] = wheel_radius;
    ranges_wheels[3] = wheel_radius;

    ondn_correct->setInputData(ranges_wheels);
    Transform Tsb;
    Tsb.setIdentity();
    ondn_correct->setTsb(Tsb);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "combined_corrector_embree");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("Embree Corrector started");

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
    nh_p.param<double>("tf_rate", tf_rate, 50);

    double corr_rate_max;
    nh_p.param<double>("corr_rate_max", corr_rate_max, 200);

    int Nposes_tmp;
    nh_p.param<int>("poses", Nposes_tmp, 1);
    Nposes = Nposes_tmp;


    EmbreeMapPtr map = import_embree_map(meshfile);
    
    scan_correct = std::make_shared<SphereCorrectorEmbreeROS>(map);
    ondn_correct = std::make_shared<OnDnCorrectorEmbreeROS>(map);

    nh_p.param<float>("max_distance", max_distance, 0.8);
    nh_p.param<bool>("adaptive_max_dist", adaptive_max_dist, false);
    nh_p.param<float>("adaptive_max_dist_min", adaptive_max_dist_min, 0.15);
    
    corr_params.max_distance = max_distance;
    scan_correct->setParams(corr_params);
    ondn_correct->setParams(corr_params);

    std::cout << "Max Distance: " << corr_params.max_distance << std::endl;

    // set OnDnModel for wheels
    genWheelModel();

    // get TF of scanner
    tfBuffer.reset(new tf2_ros::Buffer);
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

    model_pub = nh_p.advertise<visualization_msgs::Marker>("model", 1);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, scanCB);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, poseCB);


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