#include <ros/ros.h>

#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>

#include <rmcl/correction/MICP.hpp>
#include <rmcl/util/conversions.h>


#include <thread>
#include <mutex>


#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>

// rosmath
#include <rosmath/sensor_msgs/conversions.h>
#include <rosmath/sensor_msgs/math.h>
#include <rosmath/eigen/conversions.h>


using namespace rmcl;
using namespace rmagine;
using namespace rosmath;

MICPPtr micp;

std::string map_frame;
std::string odom_frame;
bool has_odom_frame = true;
std::string base_frame;

// Estimate this
geometry_msgs::TransformStamped T_odom_map;
std::mutex                      T_odom_map_mutex;
// dynamic: ekf
geometry_msgs::TransformStamped T_base_odom;


// testing
size_t Nposes = 1;


std::thread correction_thread;
bool stop_correction_thread = false;

TFBufferPtr tf_buffer;
TFListenerPtr tf_listener;


bool pose_received = false;



void fetchTF()
{
    if(has_odom_frame)
    {
        try{
            T_base_odom = tf_buffer->lookupTransform(odom_frame, base_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ROS_WARN_STREAM("Source: " << odom_frame << ", Target: " << base_frame);
            return;
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
}


void updateTF()
{
    static tf2_ros::TransformBroadcaster br;
    
    geometry_msgs::TransformStamped T;

    // What is the source frame?
    if(has_odom_frame)
    {
        // With EKF and base_frame: Send odom to map
        T = T_odom_map;
    } else {
        // With base but no EKF: send base to map
        T = T_odom_map * T_base_odom;
    }

    T.header.stamp = ros::Time::now();

    br.sendTransform(T);
}


void correctOnce()
{
    std::lock_guard<std::mutex> guard(T_odom_map_mutex);

    // 1. Get Base in Map
    geometry_msgs::TransformStamped T_base_map = T_odom_map * T_base_odom;

    Memory<Transform, RAM> poses(Nposes);
    for(size_t i=0; i<Nposes; i++)
    {
        convert(T_base_map.transform, poses[i]);
    }

    // std::cout << "CORRECT!" << std::endl;

    {
        auto Tdelta = micp->correct(poses);
        poses = multNxN(poses, Tdelta);
    }

    // Update T_odom_map
    convert(poses[0], T_base_map.transform);
    T_odom_map = T_base_map * ~T_base_odom;
}



// Storing Pose information globally
// Calculate transformation from map to odom from pose in map frame
void poseCB(geometry_msgs::PoseStamped msg)
{
    std::lock_guard<std::mutex> guard(T_odom_map_mutex);

    ROS_INFO_STREAM_NAMED(ros::this_node::getName(), ros::this_node::getName() << " Received new pose guess");

    // rest max distance
    // corr_params.max_distance = max_distance;
    // scan_correct->setParams(corr_params);

    // std::cout << "poseCB" << std::endl;
    map_frame = msg.header.frame_id;
    pose_received = true;

    // set T_base_map
    geometry_msgs::TransformStamped T_base_map;
    T_base_map.header.frame_id = map_frame;
    T_base_map.child_frame_id = base_frame;
    T_base_map.transform <<= msg.pose;

    // fetchTF();
    T_odom_map = T_base_map * ~T_base_odom;
}

void poseWcCB(geometry_msgs::PoseWithCovarianceStamped msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    poseCB(pose);
}


void correct()
{
    if(pose_received)
    {
        fetchTF();
        correctOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "micp_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    double tf_rate = 50.0;
    double corr_rate_max = 200.0;
    base_frame = "base_footprint";
    odom_frame = "odom";
    map_frame = "map";

    tf_buffer.reset(new tf2_ros::Buffer);
    tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer));


    micp = std::make_shared<MICP>();
    micp->loadParams();

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, poseCB);
    ros::Subscriber pose_wc_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_wc", 1, poseWcCB);

    // CORRECTION THREAD
    stop_correction_thread = false;
    correction_thread = std::thread([corr_rate_max]()
    {
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
            // std::cout << "Current Correction Rate: " << el << " s" << ", " << 1.0/el << " hz" << std::endl;
        }

        stop_correction_thread = false;
    });

    // MAIN LOOP TF
    ros::Rate r(tf_rate);
    ros::Time stamp = ros::Time::now();

    std::cout << "Waiting for pose guess..." << std::endl;

    while(ros::ok())
    {
        fetchTF();

        if(pose_received)
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