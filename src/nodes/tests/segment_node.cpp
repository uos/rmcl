#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Rmagine deps
#include <rmagine/map/OptixMap.hpp>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/math.cuh>
#include <rmagine/util/prints.h>
#include <rmagine/simulation/SphereSimulatorOptix.hpp>
#include <rmagine/simulation/SimulationResults.hpp>

// RCML msgs
#include <rmcl_msgs/ScanStamped.h>

// RMCL code
#include <rmcl/util/conversions.h>
#include <rmcl/util/scan_operations.h>


// rosmath
#include <rosmath/sensor_msgs/conversions.h>
#include <rosmath/sensor_msgs/math.h>
#include <rosmath/eigen/conversions.h>

#include <chrono>
#include <memory>
#include <omp.h>
#include <thread>
#include <mutex>

#include <Eigen/Dense>

using namespace rosmath;
using namespace rmcl;
using namespace rmcl_msgs;
using namespace rmagine;


SphereSimulatorOptixPtr scan_sim;


std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener; 


std::string map_frame;

geometry_msgs::TransformStamped T_sensor_map;


ros::Publisher pub_cloud;

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

    Memory<Transform, VRAM_CUDA> T_ = T;

    SphericalModel model;
    convert(msg->scan.info, model);
    scan_sim->setModel(model);

    using ResultT = Bundle<
        Ranges<VRAM_CUDA>,
        Normals<VRAM_CUDA>
    >;

    ResultT res = scan_sim->simulate<ResultT>(T_);

    // std::cout << "Simulated " << res.ranges.size() << " ranges" << std::endl;



    Memory<float, RAM> ranges = res.ranges;
    Memory<Vector, RAM> normals = res.normals;

    // float total_error = 0.0;
    // float dev = 0.0;

    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = msg->header.stamp;
    cloud.header.frame_id = msg->header.frame_id;

    float max_error = 0.1;

    for(size_t vid = 0; vid < model.getHeight(); vid++)
    {
        for(size_t hid = 0; hid < model.getWidth(); hid++)
        {
            size_t bid = model.getBufferId(vid, hid);

            float range_sim = ranges[bid];
            float range_real = msg->scan.ranges[bid];

            bool add_point = false;

            if(model.range.inside(range_real))
            {
                if(model.range.inside(range_sim))
                {
                    Vector preal_s = model.getDirection(vid, hid) * range_real;
                    Vector pint_s = model.getDirection(vid, hid) * range_sim;
                    Vector nint_s = normals[bid];
                    nint_s.normalize();

                    float signed_plane_dist = (preal_s - pint_s).dot(nint_s);
                    const Vector pmesh_s = preal_s + nint_s * signed_plane_dist;  
                    const float plane_distance = (pmesh_s - preal_s).l2norm();

                    // sim and real point are valid: check if they are different enough
                    if( plane_distance > max_error )
                    {
                        add_point = true;
                    }
                } else {
                    // point in real scan but not in simulated
                    add_point = true;
                }
            }

            if(add_point)
            {
                Vector p = model.getDirection(vid, hid) * range_real;
                geometry_msgs::Point32 p_ros;
                p_ros.x = p.x;
                p_ros.y = p.y;
                p_ros.z = p.z;
                cloud.points.push_back(p_ros);
            }
        }
    }


    pub_cloud.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segment_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO_STREAM_NAMED(ros::this_node::getName(), ros::this_node::getName() << " started");

    std::string meshfile;

    nh_p.param<std::string>("map_file", meshfile, "/home/amock/workspaces/ros/mamcl_ws/src/uos_tools/uos_gazebo_worlds/Media/models/avz_neu.dae");
    nh_p.param<std::string>("map_frame", map_frame, "map");

    OptixMapPtr map = importOptixMap(meshfile);
    scan_sim = std::make_shared<SphereSimulatorOptix>(map);
    scan_sim->setTsb(Transform::Identity());

    // get TF of scanner
    tfBuffer.reset(new tf2_ros::Buffer);
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));


    ros::Subscriber sub = nh.subscribe<ScanStamped>("scan", 1, scanCB);

    pub_cloud = nh_p.advertise<sensor_msgs::PointCloud>("outlier", 1);

    ros::spin();

    return 0;
}