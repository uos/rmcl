#include <ros/ros.h>
#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmcl/clustering/clustering.h>
#include <Eigen/Dense>
#include <sstream>

#include <rosmath/eigen/stats.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>

#include <rmagine/util/StopWatch.hpp>

using namespace rmcl;
using namespace rmagine;
using namespace rosmath;


Memory<Vector, RAM> random_normal_sample(Vector mean, Matrix3x3 cov, size_t N)
{
    Memory<Vector, RAM> samples(N);


    Eigen::Vector3d mean_eig(mean.x, mean.y, mean.z);
    Eigen::Matrix3d cov_eig;
    for(size_t i=0; i<3; i++)
    {
        for(size_t j=0; j<3; j++)
        {
            cov_eig(i,j) = cov(i,j);
        }
    }

    stats::Normal norm_dist(mean_eig, cov_eig);

    for(size_t i=0; i<N; i++)
    {
        Eigen::Vector3d sample = norm_dist.sample();
        samples[i].x = sample(0);
        samples[i].y = sample(1);
        samples[i].z = sample(2);
    }

    return samples;
}

sensor_msgs::PointCloud make_cloud(const MemoryView<Vector, RAM>& points)
{
    sensor_msgs::PointCloud cloud;

    cloud.header.frame_id = "base_footprint";
    cloud.points.resize(points.size());

    for(size_t i=0; i<points.size(); i++)
    {
        cloud.points[i].x = points[i].x;
        cloud.points[i].y = points[i].y;
        cloud.points[i].z = points[i].z;
    }

    return cloud;
}

std_msgs::ColorRGBA color_from_id(size_t id)
{
    std_msgs::ColorRGBA color;

    color.r = 0.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0;

    if(id == 0) {
        color.r = 1.0;
    } else if(id == 1) {
        color.g = 1.0;
    } else if(id == 2) {
        color.b = 1.0;
    } else if(id == 3) {
        color.r = 1.0;
        color.g = 1.0;
    } else if(id == 4) {
        color.r = 1.0;
        color.b = 1.0;
    } else if(id == 5) {
        color.g = 1.0;
        color.b = 1.0;
    }

    return color;
}

visualization_msgs::Marker make_marker(
    const MemoryView<Vector, RAM>& points,
    const std::vector<size_t>& cluster,
    size_t id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    std::stringstream ss;
    ss << id;
    marker.ns = ss.str();
    marker.id = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;

    // Points are green
    marker.color = color_from_id(id);

    marker.points.resize(cluster.size());
    for(size_t i=0; i<cluster.size(); i++)
    {
        marker.points[i].x = points[cluster[i]].x;
        marker.points[i].y = points[cluster[i]].y;
        marker.points[i].z = points[cluster[i]].z;
    }

    return marker;
}

std::vector<visualization_msgs::Marker> make_markers(
    const MemoryView<Vector, RAM>& points,
    const std::vector<std::vector<size_t> >& clusters)
{
    std::vector<visualization_msgs::Marker> markers;

    for(size_t i=0; i<clusters.size(); i++)
    {
        markers.push_back(make_marker(points, clusters[i], i));
    }

    return markers;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "clustering_node");

    Memory<Vector, RAM> points(10000);

    std::cout << "1. cluster" << std::endl;
    Vector mean1 = {0.0, 0.0, 0.0};

    Matrix3x3 cov1;
    cov1.setIdentity();
    cov1 *= 0.1;
    cov1(2,2) = 0.01;

    points(0,2000) = random_normal_sample(mean1, cov1, 2000);


    std::cout << "2. cluster" << std::endl;
    Vector mean2 = {1.0, 2.0, 0.1};

    Matrix3x3 cov2;
    cov2.setIdentity();
    cov2 *= 0.3;

    points(2000, 8000) = random_normal_sample(mean2, cov2, 6000);

    std::cout << "3. cluster" << std::endl;

    Vector mean3 = {2.0, -3.0, -0.5};
    Matrix3x3 cov3;
    cov3.setIdentity();
    cov3 *= 0.005;

    points(8000,10000) = random_normal_sample(mean3, cov3, 2000);

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    ros::Publisher pub_cloud = nh_p.advertise<sensor_msgs::PointCloud>("cloud", 1);
    ros::Publisher pub_clusters = nh_p.advertise<visualization_msgs::Marker>("clusters", 1);


    double search_dist;
    nh_p.param<double>("search_dist", search_dist, 0.2);

    int min_points_in_radius;
    nh_p.param<int>("min_points_in_radius", min_points_in_radius, 5);

    int min_points_per_cluster;
    nh_p.param<int>("min_points_per_cluster", min_points_per_cluster, 20);

    StopWatch sw;
    double el;

    ros::Rate r(10);

    while(ros::ok())
    {
        auto cloud = make_cloud(points);
        cloud.header.stamp = ros::Time::now();
        pub_cloud.publish(cloud);

        sw();
        KdPointsPtr kd_points(new KdPoints(points));
        KdTreePtr tree = std::make_shared<KdTree>(kd_points);
        el = sw();

        std::cout << "- Built Tree in " << el << " s" << std::endl;

        sw();
        auto clusters = dbscan(tree, 
            search_dist, 
            min_points_in_radius,
            min_points_per_cluster);
        el = sw();

        std::cout << "- Extracted " << clusters.size() << " clusters from " << points.size() << " points in " << el << "s" << std::endl;

        sort_clusters(clusters);

        for(size_t i=0; i<clusters.size(); i++)
        {
            std::cout << "-- Cluster " << i+1 << ": " << clusters[i].size() << std::endl;
        }

        auto markers = make_markers(points, clusters);

        for(auto marker : markers)
        {
            marker.header.stamp = ros::Time::now();
            pub_clusters.publish(marker);
        }
        

        r.sleep();
        ros::spinOnce();
    }


    return 0;
}