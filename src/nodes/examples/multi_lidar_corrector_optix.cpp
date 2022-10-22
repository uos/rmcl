#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseArray.h>

// Rmagine deps
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/math.cuh>

// RCML msgs
#include <rmcl_msgs/ScanStamped.h>

// RMCL code
#include <rmcl/correction/SphereCorrectorOptixROS.hpp>
#include <rmcl/util/conversions.h>
#include <rmcl/util/scan_operations.h>
#include <rmcl/clustering/clustering.h>

// rosmath
#include <rosmath/sensor_msgs/conversions.h>
#include <rosmath/sensor_msgs/math.h>
#include <rosmath/eigen/conversions.h>
#include <rosmath/eigen/stats.h>

#include <chrono>
#include <memory>
#include <omp.h>


using namespace rmcl;
using namespace rmcl_msgs;
using namespace rmagine;

SphereCorrectorOptixROSPtr scan_correct;
ros::Publisher cloud_pub;
ros::Publisher pose_pub;
ros::Publisher pub_poses;
ros::Publisher pub_best_poses;

unsigned int Nparticles = 2000;

Memory<Transform, VRAM_CUDA> poses;

std::vector<std::vector<size_t> > clusters;


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

std::shared_ptr<tf2_ros::Buffer> tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener; 

// Estimate this
geometry_msgs::TransformStamped T_odom_map;
// dynamic: ekf
geometry_msgs::TransformStamped T_base_odom;
// static: urdf
geometry_msgs::TransformStamped T_sensor_base;


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
            ROS_WARN_STREAM("Source: " << base_frame << ", Target: " << sensor_frame);

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

    return ret;
}

void vizOnce()
{
    Memory<Transform, RAM_CUDA> poses_ram;
    poses_ram = poses;

    geometry_msgs::PoseArray P;
    P.header.stamp = ros::Time::now();
    P.header.frame_id = map_frame;

    for(size_t i=0; i<poses_ram.size(); i++)
    {
        geometry_msgs::Pose pose_ros;
        convert(poses_ram[i], pose_ros);
        P.poses.push_back(pose_ros);
    }

    pub_poses.publish(P);

    if(clusters.size() > 0)
    {
        geometry_msgs::PoseArray P_best;
        P_best.header.stamp = ros::Time::now();
        P_best.header.frame_id = map_frame;

        auto best_cluster = clusters[0];
        for(size_t id : best_cluster)
        {
            geometry_msgs::Pose pose_ros;
            convert(poses_ram[id], pose_ros);
            P_best.poses.push_back(pose_ros);
        }

        pub_best_poses.publish(P_best);
    }
}

void computeClusters()
{
    std::cout << "Compute Clusters" << std::endl;
    StopWatch sw;
    double el;
    Memory<Transform, RAM> poses_ram = poses;
    Memory<Vector, RAM> points(poses_ram.size());

    for(size_t i=0; i<poses_ram.size(); i++)
    {
        points[i] = poses_ram[i].t;
    }

    sw();
    KdPointsPtr kd_points(new KdPoints(points));
    KdTreePtr tree = std::make_shared<KdTree>(kd_points);
    el = sw();

    std::cout << "- Built Tree in " << el << " s" << std::endl;
    sw();
    clusters = dbscan(tree, 
        0.01, 
        8,
        30);

    el = sw();

    std::cout << "- Extracted " << clusters.size() << " clusters from " << points.size() << " points in " << el << "s" << std::endl;

    sort_clusters(clusters);
    size_t cluster_points = 0;
    for(size_t i=0; i<clusters.size(); i++)
    {
        cluster_points += clusters[i].size();
        std::cout << "-- Cluster " << i+1 << ": " << clusters[i].size() << std::endl;
    }


    if(clusters.size() > 5 && clusters[0].size() > 100)
    {
        sw();
        Memory<Transform, RAM> poses_ram_new(cluster_points);
        size_t offset = 0;
        for(size_t cid=0; cid<clusters.size(); cid++)
        {
            const auto& cluster = clusters[cid];
            for(size_t i=0; i<clusters[cid].size(); i++)
            {
                poses_ram_new[i + offset] = poses_ram[cluster[i]];
            }
            offset += cluster.size();
        }
        // upload
        poses = poses_ram_new;
        el = sw();

        std::cout << "- eleminate not clustered in " << el << "s" << std::endl;
    }
}

void correctOnce()
{
    StopWatch sw;

    sw();
    auto corrRes = scan_correct->correct(poses);
    poses = multNxN(poses, corrRes.Tdelta);
    double el = sw();
    ROS_INFO_STREAM("correctOnce " << poses.size() << " poses in " << el << "s");

    // count valid
    Memory<unsigned int, RAM> Ncorr = corrRes.Ncorr;
    size_t num_valid = 0;
    for(size_t i=0; i<Ncorr.size(); i++)
    {
        if(Ncorr[i] > 50)
        {
            num_valid++;
        }
    }
    std::cout << "- " << num_valid << "/" << Ncorr.size() << " valid" << std::endl;


    // cluster
    computeClusters();

    // Update T_odom_map
    // convert(poses[poses.size() - 1], T_base_map.transform);
    // T_odom_map = T_base_map * ~T_base_odom;
}

void poseCBRandom(geometry_msgs::PoseStamped msg)
{
    map_frame = msg.header.frame_id;
    pose_received = true;

    Eigen::Matrix<double, 6, 6> cov;
    cov <<  10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, M_PI, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, M_PI;

    Eigen::Matrix<double, 6, 1> mean;

    mean(0) = msg.pose.position.x;
    mean(1) = msg.pose.position.y;
    mean(2) = msg.pose.position.z + 0.5;

    geometry_msgs::Vector3 rpy_ros = quat2rpy(msg.pose.orientation);

    mean(3) = rpy_ros.x;
    mean(4) = rpy_ros.y;
    mean(5) = rpy_ros.z;

    stats::Normal normal_dist(mean, cov);
    
    Memory<Transform, RAM_CUDA> poses_ram(Nparticles);
    for(size_t i=0; i<Nparticles; i++)
    {
        auto particle = normal_dist.sample();
        poses_ram[i].t.x = particle(0);
        poses_ram[i].t.y = particle(1);
        poses_ram[i].t.z = particle(2);
        EulerAngles rpy;
        rpy.roll = particle(3);
        rpy.pitch = particle(4);
        rpy.yaw = particle(5);
        poses_ram[i].R.set(rpy);
    }

    poses = poses_ram;
}

// Storing Pose information globally
// Calculate transformation from map to odom from pose in map frame
void poseCB(geometry_msgs::PoseStamped msg)
{
    map_frame = msg.header.frame_id;
    pose_received = true;

    // set T_base_map
    geometry_msgs::TransformStamped T_base_map;
    T_base_map.header.frame_id = map_frame;
    T_base_map.child_frame_id = base_frame;
    T_base_map.transform <<= msg.pose;

    fetchTF();

    T_odom_map = T_base_map * ~T_base_odom;

}

// Storing scan information globally
// updating real data inside the global scan corrector
void scanCB(const ScanStamped::ConstPtr& msg)
{
    sensor_frame = msg->header.frame_id;
    scan_correct->setModelAndInputData(msg->scan);
    last_scan = msg->header.stamp;
    scan_received = true;

    if(pose_received)
    {
        fetchTF();
        correctOnce();
        vizOnce();
    }
}

void updateTF()
{
    static tf2_ros::TransformBroadcaster br;
    
    geometry_msgs::TransformStamped T;

    // What is the source frame?
    if(has_odom_frame && has_base_frame)
    {
        // With EKF and base_frame: Send odom to map
        T = T_odom_map;
    } else if(has_base_frame) {
        // With base but no EKF: send base to map
        T = T_odom_map * T_base_odom;
    } else {
        // Default:
        // Sensor to map
        T = T_odom_map * T_base_odom * T_sensor_base;
    }

    T.header.stamp = ros::Time::now();
    T.header.frame_id = map_frame;

    br.sendTransform(T);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_lidar_corrector_optix");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("Optix Corrector started");

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

    OptixMapPtr map = import_optix_map(meshfile);
    
    scan_correct.reset(new SphereCorrectorOptixROS(map));

    CorrectionParams corr_params;
    nh_p.param<float>("max_distance", corr_params.max_distance, 0.5);
    int particles_tmp;
    nh_p.param<int>("particles", particles_tmp, 2000);
    if(particles_tmp > 0)
    {
        Nparticles = particles_tmp;
    }
    scan_correct->setParams(corr_params);

    std::cout << "- Max Distance: " << corr_params.max_distance << std::endl;
    std::cout << "- particles: " << Nparticles << std::endl;

    // get TF of scanner
    tfBuffer.reset(new tf2_ros::Buffer);
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

    // cloud_pub = nh_p.advertise<sensor_msgs::PointCloud>("sim_cloud", 1);
    // pose_pub = nh_p.advertise<geometry_msgs::PoseStamped>("sim_pose", 1);
    pub_poses = nh_p.advertise<geometry_msgs::PoseArray>("particles", 1);
    pub_best_poses = nh_p.advertise<geometry_msgs::PoseArray>("best_particles", 1);
    ros::Subscriber sub = nh.subscribe<ScanStamped>("scan", 1, scanCB);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, poseCBRandom);

    ROS_INFO_STREAM(ros::this_node::getName() << ": Open RViz. Set fixed frame to map frame. Set goal. ICP to Mesh");

    ros::Rate r(30);
    ros::Time stamp = ros::Time::now();

    while(ros::ok())
    {
        if(pose_received && scan_received)
        {
            // updateTF();
            // weird bug. new_stamp sometimes is equal to stamp. results 
            
            // ros::Time new_stamp = ros::Time::now();
            // if(new_stamp > stamp)
            // {
            //     updateTF();
            //     stamp = new_stamp;
            // }
        }
        
        r.sleep();
        ros::spinOnce();
    }
    
    return 0;
}