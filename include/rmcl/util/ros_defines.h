#ifndef RMCL_UTIL_ROS_DEFINES_H
#define RMCL_UTIL_ROS_DEFINES_H

#include <ros/ros.h>
#include <memory>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>

namespace rmcl
{

using NodeHandlePtr = std::shared_ptr<ros::NodeHandle>;
using SubscriberPtr = std::shared_ptr<ros::Subscriber>;
using ImageTransportPtr = std::shared_ptr<image_transport::ImageTransport>;
using ITSubscriberPtr = std::shared_ptr<image_transport::Subscriber>;
using TFBufferPtr = std::shared_ptr<tf2_ros::Buffer>;
using TFListenerPtr = std::shared_ptr<tf2_ros::TransformListener>;

} // namespace rmcl

#endif // RMCL_UTIL_ROS_DEFINES_H