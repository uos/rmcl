#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <rmcl/util/conversions.h>
#include <rmcl_msgs/DepthStamped.h>
#include <image_transport/image_transport.h>

using namespace rmagine;
using namespace rmcl;

sensor_msgs::CameraInfo info;
ros::Publisher pub_depth;
ros::Publisher pub_cloud;
image_transport::Publisher pub_image;

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
                Vector dir;

                if(optical)
                {
                    dir = model.getDirectionOptical(vid, hid);
                } else {
                    dir = model.getDirection(vid, hid);
                }

                // dir /= dir.z;
                Vector p = dir * range;
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
    const sensor_msgs::Image::ConstPtr& from,
    rmcl_msgs::Depth& to)
{
    
    PinholeModel model;
    convert(info, model);

    unsigned int bytes = from->step / from->width;

    if(from->encoding == "32FC1")
    {
        if(bytes != 4)
        {
            std::cout << "32FC1 should have 4 bytes. Calculated " << bytes << std::endl;
        }

        // Problem: 
        // floating point value in pixel is not the range in meters!
        // it is the scale of the camera vector intersecting the pixel
        // with z=1
        // the solution to that is in the for loop
        // -- Tested on simulated depth images --
        to.data.ranges.resize(from->width * from->height);
        for(unsigned int vid = 0; vid < model.getHeight(); vid++)
        {
            for(unsigned int hid = 0; hid < model.getWidth(); hid++)
            {
                unsigned int loc_id = model.getBufferId(vid, hid);
                const float wrong_range = *reinterpret_cast<const float*>(&from->data[loc_id * sizeof(float)]);
                Vector dir = model.getDirectionOptical(vid, hid);
                const float real_range = wrong_range / dir.z;
                to.data.ranges[loc_id] = real_range;
            }
        }
        
    } else {
        ROS_WARN_STREAM("Could not convert image of encoding " << from->encoding);
        // pub_image.publish(from);
    }
}

void convert(
    const sensor_msgs::CameraInfo& info,
    const sensor_msgs::Image::ConstPtr& dimage,
    rmcl_msgs::DepthStamped& depth)
{
    if(dimage->header.frame_id != info.header.frame_id)
    {
        ROS_WARN("Cloud and Camera info are not in same frame");
    }

    depth.header.stamp = dimage->header.stamp;
    depth.header.frame_id = dimage->header.frame_id;
    convert(info, depth.depth.info);
    
    // manual setting range limits
    depth.depth.info.range_min = 0.3;
    depth.depth.info.range_max = 8.0;

    convert(dimage, depth.depth);
}

void infoCB(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    info = *msg;
}

void imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
    rmcl_msgs::DepthStamped out;
    
    convert(info, msg, out);

    if(out.depth.data.ranges.size() > 0)
    {
        // std::cout << "Publish " << out.depth.ranges.size() << " points." << std::endl;
        pub_depth.publish(out);

        sensor_msgs::PointCloud out_cloud;
        convert(out, out_cloud);
        
        pub_cloud.publish(out_cloud);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_to_depth");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_image = it.subscribe("image", 1, imageCB);

    // two options: either use fixed parameters from yaml or use camera info topic
    ros::Subscriber sub_info = nh.subscribe<sensor_msgs::CameraInfo>("info", 1, infoCB);

    pub_depth = nh_p.advertise<rmcl_msgs::DepthStamped>("depth", 1);
    pub_cloud = nh_p.advertise<sensor_msgs::PointCloud>("depth_cloud", 1);
    pub_image = it.advertise("image_filtered", 1);

    ros::spin();

    return 0;
}