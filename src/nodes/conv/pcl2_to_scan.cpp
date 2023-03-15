#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <rmcl_msgs/ScanStamped.h>

#include <rmcl/util/conversions.h>
#include <rmcl/util/scan_operations.h>

#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>

#include <Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>



using namespace rmcl;

namespace rm = rmagine;

std::string focal_frame = "";
bool debug_cloud = false;

ros::Publisher scan_pub;
ros::Publisher back_conv_pub;

rmcl_msgs::ScanStamped scan;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

void initScanArray()
{
    fillEmpty(scan.scan);
}

void loadParameters(ros::NodeHandle &nh_p)
{
    rmcl_msgs::ScanInfo &scanner_model = scan.scan.info;
    if (!nh_p.getParam("model/phi_min", scanner_model.phi_min))
    {
        ROS_ERROR_STREAM("When specifying auto_detect_phi to false you have to provide model/phi_min");
        return;
    }
    if (!nh_p.getParam("model/phi_inc", scanner_model.phi_inc))
    {
        ROS_ERROR_STREAM("When specifying auto_detect_phi to false you have to provide model/phi_max");
        return;
    }

    if (!nh_p.getParam("model/theta_min", scanner_model.theta_min))
    {
        ROS_ERROR_STREAM("When specifying auto_detect_phi to false you have to provide model/phi_min");
        return;
    }
    if (!nh_p.getParam("model/theta_inc", scanner_model.theta_inc))
    {
        ROS_ERROR_STREAM("When specifying auto_detect_phi to false you have to provide model/phi_max");
        return;
    }

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

    int phi_N_tmp, theta_N_tmp;
    if (!nh_p.getParam("model/phi_N", phi_N_tmp))
    {
        ROS_ERROR_STREAM("When specifying auto_detect_phi to false you have to provide model/phi_min");
        return;
    }
    if (!nh_p.getParam("model/theta_N", theta_N_tmp))
    {
        ROS_ERROR_STREAM("When specifying auto_detect_phi to false you have to provide model/phi_max");
        return;
    }
    scanner_model.phi_N = phi_N_tmp;
    scanner_model.theta_N = theta_N_tmp;

    nh_p.param<bool>("debug_cloud", debug_cloud, false);
}

void convert(
    const sensor_msgs::PointCloud2::ConstPtr &pcl,
    rmcl_msgs::ScanStamped &scan)
{
    rm::Transform T = rm::Transform::Identity();

    if (pcl->header.frame_id != focal_frame)
    {
        // TODO: get transform

        geometry_msgs::TransformStamped Tros;

        try
        {
            Tros = tf_buffer->lookupTransform(focal_frame, pcl->header.frame_id,
                                              ros::Time(0));
            convert(Tros.transform, T);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    fillEmpty(scan.scan);

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

    rmagine::SphericalModel model;
    convert(scan.scan.info, model);

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
            float theta_est = atan2(ps.y, ps.x); // horizontal
            float phi_est = atan2(ps.z, range_est); // vertical
            
            int phi_id = ((phi_est - model.phi.min) / model.phi.inc) + 0.5;
            int theta_id = ((theta_est - model.theta.min) / model.theta.inc) + 0.5;

            // rm::Vector ps_ = model.getDirection(phi_id, theta_id) * range_est;

            // std::cout << "-------------------" << std::endl;

            // std::cout << "origorig: " << ps_s << std::endl;
            // std::cout << "orig: " << ps << std::endl;

            // std::cout << "polar (phi, theta, range): " << phi_est << ", " << theta_est << ", " << range_est << std::endl; 
            // std::cout << "scan id (phi_id, theta_id): " << phi_id << ", " << theta_id << std::endl; 

            // std::cout << "recon: " << ps_ << std::endl;

            if(phi_id >= 0 && phi_id < model.phi.size
                && theta_id >= 0 && theta_id < model.theta.size)
            {
                if(model.range.inside(range_est))
                {
                    // std::cout << "Polar (theta, phi, range): " << theta_est << ", " << phi_est << ", " << range_est << std::endl;
                    // std::cout << "- matrix id (theta, phi): " << theta_id << ", " << phi_id << std::endl;
                    // std::cout << "- valid: add" << std::endl;
                    unsigned int p_id = model.getBufferId(phi_id, theta_id);
                    scan.scan.data.ranges[p_id] = range_est;
                }

                // } else {
                //     // std::cout << "- out of range" << std::endl; 
                // }

                
            } else {
                // std::cout << "- out scanner matrix" << std::endl;
            }
        }
    }
}

void veloCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (focal_frame == "")
    {
        focal_frame = msg->header.frame_id;
    }

    scan.header.stamp = msg->header.stamp;
    scan.header.frame_id = focal_frame;
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

    nh_p.param<std::string>("focal_frame", focal_frame, "");

    loadParameters(nh_p);

    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    ROS_INFO("sensor_msgs::PointCloud2 to mamcl_msgs::ScanStamped Converter started");

    ros::Subscriber velo_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, veloCB);
    scan_pub = nh_p.advertise<rmcl_msgs::ScanStamped>("scan", 1);
    back_conv_pub = nh_p.advertise<sensor_msgs::PointCloud>("cloud_back", 1);

    ros::spin();

    return 0;
}