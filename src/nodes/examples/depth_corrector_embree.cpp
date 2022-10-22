#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Rmagine deps
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/util/StopWatch.hpp>

// RCML msgs
#include <rmcl_msgs/DepthStamped.h>

// RMCL code
#include <rmcl/correction/PinholeCorrectorEmbreeROS.hpp>
#include <rmcl/util/conversions.h>
#include <rmcl/util/scan_operations.h>

#include <chrono>
#include <memory>
#include <omp.h>


using namespace rmcl;
using namespace rmcl_msgs;
using namespace rmagine;

PinholeCorrectorEmbreeROSPtr depth_correct;
ros::Publisher cloud_pub;
ros::Publisher pose_pub;

bool        pose_received = false;
ros::Time   last_pose;
bool        scan_received = false;
ros::Time   last_scan;

std::string map_frame;
std::string odom_frame;
bool has_odom_frame = true;
std::string base_frame;
bool has_base_frame = true;
std::string sensor_frame;
bool sensor_frame_optical = false;

std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener; 

// Estimate this
geometry_msgs::TransformStamped T_odom_map;
Transform                       Tom;
// dynamic: ekf
geometry_msgs::TransformStamped T_base_odom;
Transform                       Tbo;
// static: urdf
geometry_msgs::TransformStamped T_sensor_base;
Transform                       Tsb;

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
    depth_correct->setTsb(T_sensor_base.transform);
    
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
    StopWatch sw;
    // std::cout << "correctOnce" << std::endl;
    // 1. Get Base in Map
    Transform Tbm = Tom * Tbo;
    
    size_t Nposes = 1;

    Memory<Transform, RAM> poses(Nposes);
    for(size_t i=0; i<Nposes; i++)
    {
        poses[i] = Tbm;
    }
    
    sw();
    auto corrRes = depth_correct->correct(poses);
    double el = sw();

    ROS_INFO_STREAM("correctOnce: poses " << Nposes << " in " << el << "s");

    poses = multNxN(poses, corrRes.Tdelta);

    // Update T_odom_map
    Tom = poses[0] * ~Tbo;
}

// Storing Pose information globally
// Calculate transformation from map to odom from pose in map frame
void poseCB(geometry_msgs::PoseStamped msg)
{
    // std::cout << "poseCB" << std::endl;
    map_frame = msg.header.frame_id;
    pose_received = true;

    Transform Tbm;
    convert(msg.pose, Tbm);

    fetchTF();

    // T_odom_map = T_base_map * ~T_base_odom;
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
void depthCB(const DepthStamped::ConstPtr& msg)
{
    sensor_frame = msg->header.frame_id;
    sensor_frame_optical = (sensor_frame.find("_optical") != std::string::npos);

    depth_correct->setOptical(sensor_frame_optical);
    depth_correct->setModelAndInputData(msg->depth);
    last_scan = msg->header.stamp;
    scan_received = true;

    if(pose_received)
    {
        fetchTF();
        correctOnce();
    }
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
    ros::init(argc, argv, "depth_corrector_embree");
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

    EmbreeMapPtr map = import_embree_map(meshfile);
    
    depth_correct.reset(new PinholeCorrectorEmbreeROS(map));

    CorrectionParams corr_params;
    nh_p.param<float>("max_distance", corr_params.max_distance, 0.5);
    depth_correct->setParams(corr_params);

    std::cout << "Max Distance: " << corr_params.max_distance << std::endl;

    // get TF of scanner
    tfBuffer.reset(new tf2_ros::Buffer);
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

    cloud_pub = nh_p.advertise<sensor_msgs::PointCloud>("sim_cloud", 1);
    pose_pub = nh_p.advertise<geometry_msgs::PoseStamped>("sim_pose", 1);
    ros::Subscriber sub = nh.subscribe<DepthStamped>("depth", 1, depthCB);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, poseCB);

    ROS_INFO_STREAM(ros::this_node::getName() << ": Open RViz. Set fixed frame to map frame. Set goal. ICP to Mesh");

    ros::Rate r(30);
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
    
    return 0;
}