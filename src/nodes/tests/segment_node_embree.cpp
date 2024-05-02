#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Rmagine deps
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/math.h>
#include <rmagine/util/prints.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/SimulationResults.hpp>

// RCML msgs
#include <rmcl_msgs/ScanStamped.h>

// RMCL code
#include <rmcl/util/conversions.h>
#include <rmcl/util/scan_operations.h>


// // rosmath
// #include <rosmath/sensor_msgs/conversions.h>
// #include <rosmath/sensor_msgs/math.h>
// #include <rosmath/eigen/conversions.h>

#include <chrono>
#include <memory>
#include <omp.h>
#include <thread>
#include <mutex>

// #include <Eigen/Dense>

// using namespace rosmath;
using namespace rmcl;
using namespace rmcl_msgs;
using namespace rmagine;


SphereSimulatorEmbreePtr scan_sim;


float min_dist_outlier_scan;
float min_dist_outlier_map;

std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener; 


std::string map_frame;

geometry_msgs::TransformStamped T_sensor_map;

ros::Publisher pub_outlier_scan;
ros::Publisher pub_outlier_map;


void scanCB(const ScanStamped::ConstPtr& msg)
{
    geometry_msgs::TransformStamped T_sensor_map;
    
    try{
        T_sensor_map = tfBuffer->lookupTransform(map_frame, msg->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        // ROS_WARN("%s", ex.what());
        // ROS_WARN_STREAM("Source: " << msg->header.frame_id << ", Target: " << map_frame);
        return;
    }

    Memory<Transform, RAM> T(1);
    convert(T_sensor_map.transform, T[0]);

    // Memory<Transform, VRAM_CUDA> T_ = T;

    SphericalModel model;
    convert(msg->scan.info, model);
    scan_sim->setModel(model);

    using ResultT = Bundle<
        Ranges<RAM>,
        Normals<RAM>
    >;

    ResultT res = scan_sim->simulate<ResultT>(T);


    Memory<float, RAM> ranges = res.ranges;
    Memory<Vector, RAM> normals = res.normals;

    // float total_error = 0.0;
    // float dev = 0.0;

    sensor_msgs::PointCloud cloud_outlier_scan;
    cloud_outlier_scan.header.stamp = msg->header.stamp;
    cloud_outlier_scan.header.frame_id = msg->header.frame_id;

    sensor_msgs::PointCloud cloud_outlier_map;
    cloud_outlier_map.header.stamp = msg->header.stamp;
    cloud_outlier_map.header.frame_id = msg->header.frame_id;


    for(size_t vid = 0; vid < model.getHeight(); vid++)
    {
        for(size_t hid = 0; hid < model.getWidth(); hid++)
        {
            const size_t bid = model.getBufferId(vid, hid);

            const float range_real = msg->scan.data.ranges[bid];
            const float range_sim = ranges[bid];
            

            const bool range_real_valid = model.range.inside(range_real);
            const bool range_sim_valid = model.range.inside(range_sim);

            if(range_real_valid)
            {
                Vector preal_s = model.getDirection(vid, hid) * range_real;

                if(range_sim_valid)
                {
                    
                    Vector pint_s = model.getDirection(vid, hid) * range_sim;
                    Vector nint_s = normals[bid];
                    nint_s.normalizeInplace();

                    float signed_plane_dist = (preal_s - pint_s).dot(nint_s);
                    const Vector pmesh_s = preal_s + nint_s * signed_plane_dist;  
                    const float plane_distance = (pmesh_s - preal_s).l2norm();

                    if(range_real < range_sim)
                    {
                        // something is in front of surface
                        if( plane_distance > min_dist_outlier_scan )
                        {
                            geometry_msgs::Point32 p_ros;
                            p_ros.x = preal_s.x;
                            p_ros.y = preal_s.y;
                            p_ros.z = preal_s.z;
                            cloud_outlier_scan.points.push_back(p_ros);
                        }
                    } else {
                        // ray cutted the surface
                        if( plane_distance > min_dist_outlier_map )
                        {
                            geometry_msgs::Point32 p_ros;
                            p_ros.x = pint_s.x;
                            p_ros.y = pint_s.y;
                            p_ros.z = pint_s.z;
                            cloud_outlier_map.points.push_back(p_ros);
                        }
                    }
                    
                } else {
                    // point in real scan but not in simulated
                    geometry_msgs::Point32 p_ros;
                    p_ros.x = preal_s.x;
                    p_ros.y = preal_s.y;
                    p_ros.z = preal_s.z;
                    cloud_outlier_scan.points.push_back(p_ros);
                }
            } else {
                if(range_sim_valid)
                {
                    // sim hits surface but real not: map could be wrong
                    Vector pint_s = model.getDirection(vid, hid) * range_sim;
                    geometry_msgs::Point32 p_ros;
                    p_ros.x = pint_s.x;
                    p_ros.y = pint_s.y;
                    p_ros.z = pint_s.z;
                    cloud_outlier_map.points.push_back(p_ros);
                } else {
                    // both sim and real does not hit the map
                }
            }
        }
    }

    pub_outlier_scan.publish(cloud_outlier_scan);
    pub_outlier_map.publish(cloud_outlier_map);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segment_node_embree");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO_STREAM_NAMED(ros::this_node::getName(), ros::this_node::getName() << " started");

    std::string meshfile;

    nh_p.param<std::string>("map_file", meshfile, "/home/amock/workspaces/ros/mamcl_ws/src/uos_tools/uos_gazebo_worlds/Media/models/avz_neu.dae");
    nh_p.param<std::string>("map_frame", map_frame, "map");

    nh_p.param<float>("min_dist_outlier_scan", min_dist_outlier_scan, 0.15);
    nh_p.param<float>("min_dist_outlier_map", min_dist_outlier_map, 0.15);

    EmbreeMapPtr map = import_embree_map(meshfile);
    scan_sim = std::make_shared<SphereSimulatorEmbree>(map);
    scan_sim->setTsb(Transform::Identity());

    // get TF of scanner
    tfBuffer.reset(new tf2_ros::Buffer);
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));


    ros::Subscriber sub = nh.subscribe<ScanStamped>("scan", 1, scanCB);

    pub_outlier_scan = nh_p.advertise<sensor_msgs::PointCloud>("outlier_scan", 1);
    pub_outlier_map = nh_p.advertise<sensor_msgs::PointCloud>("outlier_map", 1);

    ros::spin();

    return 0;
}
