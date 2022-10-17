#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <rmcl/util/conversions.h>
#include <rmcl_msgs/DepthStamped.h>

using namespace rmagine;
using namespace rmcl;

sensor_msgs::CameraInfo info;
ros::Publisher pub_depth;
ros::Publisher pub_cloud;

void convert(
    const rmcl_msgs::DepthStamped& from,
    sensor_msgs::PointCloud& to,
    bool optical = true)
{
    PinholeModel model;
    convert(from.depth.info, model);

    for(unsigned int vid = 0; vid < model.getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < model.getWidth(); hid++)
        {
            const unsigned int loc_id = model.getBufferId(vid, hid);
            const float range = from.depth.data.ranges[loc_id];
            if(model.range.inside(range))
            {
                Vector p;
                if(optical)
                {
                    p = model.getDirectionOptical(vid, hid) * range;
                } else {
                    p = model.getDirection(vid, hid) * range;
                }
                geometry_msgs::Point32 p_ros;
                p_ros.x = p.x;
                p_ros.y = p.y;
                p_ros.z = p.z;
                to.points.push_back(p_ros);
            }
        }
    }

    to.header.frame_id = from.header.frame_id;
    to.header.stamp = from.header.stamp;
}

void convert(
    const sensor_msgs::PointCloud2& from,
    rmcl_msgs::Depth& to)
{
    to.data.ranges.resize(from.width * from.height);

    sensor_msgs::PointField field_x;
    sensor_msgs::PointField field_y;
    sensor_msgs::PointField field_z;

    for(size_t i=0; i<from.fields.size(); i++)
    {
        if(from.fields[i].name == "x")
        {
            field_x = from.fields[i];
        }
        if(from.fields[i].name == "y")
        {
            field_y = from.fields[i];
        }
        if(from.fields[i].name == "z")
        {
            field_z = from.fields[i];
        }
    }

    // what to do if order is different?
    for(size_t i=0; i<from.width * from.height; i++)
    {
        const uint8_t* data_ptr = &from.data[i * from.point_step];

        float x,y,z;
        if(field_x.datatype == sensor_msgs::PointField::FLOAT32)
        {
            // Float
            x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
            y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
            z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
        } else if(field_x.datatype == sensor_msgs::PointField::FLOAT64) {
            // Double
            x = *reinterpret_cast<const double*>(data_ptr + field_x.offset);
            y = *reinterpret_cast<const double*>(data_ptr + field_y.offset);
            z = *reinterpret_cast<const double*>(data_ptr + field_z.offset);
        } else {
            throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
        }

        if(!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
        {
            float range_est = sqrt(x*x + y*y + z*z);
            // if(i == 0)
            // {
            //     std::cout << "Correct Depth (0,0): " << range_est << std::endl;
            // }
            to.data.ranges[i] = range_est;
        }
    }
}

void convert(
    const sensor_msgs::CameraInfo& info,
    const sensor_msgs::PointCloud2& cloud,
    rmcl_msgs::DepthStamped& depth)
{
    if(cloud.header.frame_id != info.header.frame_id)
    {
        ROS_WARN("Cloud and Camera info are not in same frame");
    }

    depth.header.stamp = cloud.header.stamp;
    depth.header.frame_id = cloud.header.frame_id;
    convert(info, depth.depth.info);
    
    // manual setting range limits
    depth.depth.info.range_min = 0.3;
    depth.depth.info.range_max = 8.0;

    convert(cloud, depth.depth);
}

void infoCB(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    info = *msg;
}

void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    rmcl_msgs::DepthStamped out;
    convert(info, *msg, out);
    pub_depth.publish(out);

    sensor_msgs::PointCloud out_cloud;
    convert(out, out_cloud);
    pub_cloud.publish(out_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl2_to_depth");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    // two options: either use fixed parameters from yaml or use camera info topic
    ros::Subscriber sub_info = nh.subscribe<sensor_msgs::CameraInfo>("info", 1, infoCB);
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("cloud", 1, cloudCB);

    pub_depth = nh_p.advertise<rmcl_msgs::DepthStamped>("depth", 1);
    pub_cloud = nh_p.advertise<sensor_msgs::PointCloud>("depth_cloud", 1);

    ros::spin();

    return 0;
}