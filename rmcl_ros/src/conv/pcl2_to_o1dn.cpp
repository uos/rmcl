#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <rmcl_msgs/O1DnStamped.h>

#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/scan_operations.h>

#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>

#include <Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>



using namespace rmcl;

namespace rm = rmagine;

std::string sensor_frame = "";
bool debug_cloud = false;

ros::Publisher scan_pub;
ros::Publisher back_conv_pub;

rmcl_msgs::O1DnStamped scan;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

void loadParameters(ros::NodeHandle &nh_p)
{
    rmcl_msgs::O1DnInfo &scanner_model = scan.o1dn.info;
    nh_p.param<bool>("debug_cloud", debug_cloud, false);
    if (!nh_p.getParam("model/range_min", scanner_model.range_min))
    {
        ROS_ERROR_STREAM("When specifying auto_detect_phi to false you have to provide model/phi_min");
        return;
    }
    if (!nh_p.getParam("model/range_max", scanner_model.range_max))
    {
        ROS_ERROR_STREAM("When specifying auto_detect_phi to false you have to provide model/phi_max");
        return;
    }

    nh_p.param<std::string>("sensor_frame", sensor_frame, "");

}

void convert(
        const sensor_msgs::PointCloud2::ConstPtr &pcl,
        rmcl_msgs::O1DnStamped &scan)
{
    rm::Transform T = rm::Transform::Identity();

    if (pcl->header.frame_id != sensor_frame)
    {
        geometry_msgs::TransformStamped Tros;

        try
        {
            Tros = tf_buffer->lookupTransform(sensor_frame, pcl->header.frame_id,
                    ros::Time(0));
            convert(Tros.transform, T);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    scan.o1dn.data.ranges.resize(pcl->width * pcl->height);

    scan.o1dn.info.width  = pcl->width;
    scan.o1dn.info.height = pcl->height;
    scan.o1dn.info.dirs.resize(scan.o1dn.info.width * scan.o1dn.info.height);
    
    scan.o1dn.info.orig.x = 0.0;
    scan.o1dn.info.orig.y = 0.0;
    scan.o1dn.info.orig.z = 0.0;

    sensor_msgs::PointField field_x;
    sensor_msgs::PointField field_y;
    sensor_msgs::PointField field_z;

    for (size_t i = 0; i < pcl->fields.size(); i++)
    {
        if (pcl->fields[i].name == "x")
        {
            field_x = pcl->fields[i];
        }
        if (pcl->fields[i].name == "y")
        {
            field_y = pcl->fields[i];
        }
        if (pcl->fields[i].name == "z")
        {
            field_z = pcl->fields[i];
        }

    }

    for (size_t i = 0; i < pcl->width * pcl->height; i++)
    {
        const uint8_t *data_ptr = &pcl->data[i * pcl->point_step];

        // rmagine::Vector point;
        float x, y, z;

        if (field_x.datatype == sensor_msgs::PointField::FLOAT32)
        {
            // Float
            x = *reinterpret_cast<const float *>(data_ptr + field_x.offset);
            y = *reinterpret_cast<const float *>(data_ptr + field_y.offset);
            z = *reinterpret_cast<const float *>(data_ptr + field_z.offset);
        }
        else if (field_x.datatype == sensor_msgs::PointField::FLOAT64)
        {
            // Double
            x = *reinterpret_cast<const double *>(data_ptr + field_x.offset);
            y = *reinterpret_cast<const double *>(data_ptr + field_y.offset);
            z = *reinterpret_cast<const double *>(data_ptr + field_z.offset);
        }
        else
        {
            throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
        }

        if(!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
        {
            rm::Vector ps_s = rm::Vector{x, y, z};
            rm::Vector ps = T * ps_s;

            float range_est = ps.l2norm();
            ps = ps / range_est;
            scan.o1dn.data.ranges[i] = range_est;
            scan.o1dn.info.dirs[i].x = ps.x;
            scan.o1dn.info.dirs[i].y = ps.y;
            scan.o1dn.info.dirs[i].z = ps.z;

        } else {
            scan.o1dn.data.ranges[i] = scan.o1dn.info.range_max + 1;
            scan.o1dn.info.dirs[i].x = 0;
            scan.o1dn.info.dirs[i].y = 0;
            scan.o1dn.info.dirs[i].z = 0;
        }
    }
}

void veloCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (sensor_frame == "")
    {
        sensor_frame = msg->header.frame_id;
    }

    scan.header.stamp = msg->header.stamp;
    scan.header.frame_id = sensor_frame;
    convert(msg, scan);

    scan_pub.publish(scan);

    if (debug_cloud)
    {
        sensor_msgs::PointCloud cloud;
        rmcl::convert(scan, cloud);
        cloud.header.stamp = msg->header.stamp;
        back_conv_pub.publish(cloud);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl2_to_scan");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    loadParameters(nh_p);

    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    ROS_INFO("sensor_msgs::PointCloud2 to rmcl_msgs::O1DnStamped Converter started");

    ros::Subscriber velo_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, veloCB);
    scan_pub = nh_p.advertise<rmcl_msgs::O1DnStamped>("scan", 1);
    back_conv_pub = nh_p.advertise<sensor_msgs::PointCloud>("cloud_back", 1);

    ros::spin();

    return 0;
}
