#ifndef RMCL_ROS_NODES_MICP_LOCALIZATION_HPP
#define RMCL_ROS_NODES_MICP_LOCALIZATION_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmcl_ros/util/ros_helper.h>

#include <rmcl_ros/correction/MICPRangeSensor.hpp>

#include <unordered_map>
#include <string>

namespace rmcl
{

class MICPLocalizationNode : public rclcpp::Node
{
public:
    explicit MICPLocalizationNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * Loads a single sensor config
     */
    MICPRangeSensorPtr loadSensor(
        ParamTree<rclcpp::Parameter>::SharedPtr sensor_params);

private:
    std::string map_frame_;
    std::string base_frame_;
    std::string odom_frame_;
    bool        use_odom_frame_;

    std::string map_filename_;
    std::unordered_map<std::string, MICPRangeSensorPtr> sensors_;
};


} // namespace rmcl

#endif // RMCL_ROS_NODES_MICP_LOCALIZATION_HPP