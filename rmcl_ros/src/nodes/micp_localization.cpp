#include <rclcpp/rclcpp.hpp>

#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/math.h>
#ifdef RMCL_CUDA
#include <rmagine/math/math.cuh>
#endif // RMCL_CUDA

// #include <rmcl_ros/correction/MICP.hpp>
#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/ros_helper.h>

#include <thread>
#include <mutex>


#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <rmcl_ros/nodes/micp_localization.hpp>
#include <rmcl_ros/correction/MICPSensorSphericalEmbree.hpp>

// #include <exception>

#include <rmcl_ros/correction/DataLoader.hpp>
#include <rmcl_ros/correction/data_loader/Topic.hpp>


// #include <rmcl_ros/correction/RCCEmbree.hpp>


namespace rm = rmagine;

namespace rmcl
{

MICPLocalizationNode::MICPLocalizationNode(const rclcpp::NodeOptions& options)
:rclcpp::Node("micp_localization_node", rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
  std::cout << "MICPLocalizationNode" << std::endl;

  base_frame_ = rmcl::get_parameter(this, "base_frame", "base_link");
  map_frame_ = rmcl::get_parameter(this, "map_frame", "map");

  odom_frame_ = rmcl::get_parameter(this, "odom_frame", "");
  use_odom_frame_ = (odom_frame_ != "");

  map_filename_ = rmcl::get_parameter(this, "map_file", "");
  if(map_filename_ == "")
  {
    RCLCPP_ERROR(get_logger(), "User must provide ~map_file");
    throw std::runtime_error("User must provide ~map_file");
  }

  std::cout << "MAP FILE: " << map_filename_ << std::endl;

  // loading general micp config

  // loading sensors from parameter tree
  std::map<std::string, rclcpp::Parameter> sensors_param;
  if(this->get_parameters("sensors", sensors_param))
  {
    std::cout << std::endl;
    std::cout << "-------------------------" << std::endl;
    std::cout << "     --- SENSORS ---     " << std::endl;
    std::cout << "-------------------------" << std::endl;

    ParamTree<rclcpp::Parameter>::SharedPtr sensors_param_tree 
      = get_parameter_tree(this, "sensors");
    for(auto elem : *sensors_param_tree)
    {
      auto sensor = loadSensor(elem.second);
      if(sensor)
      {
        std::cout << "Loaded:  " << sensor->name << std::endl;
        sensors_[sensor->name] = sensor;

      } else {
        std::string sensor_name = elem.second->name;
        std::cout << "Couldn't load sensor: '" << sensor_name << "'" << std::endl;
      }
    }
  } else {
    RCLCPP_ERROR(get_logger(), "ERROR: NO SENSORS SPECIFIED!");
    throw std::runtime_error("UERROR: NO SENSORS SPECIFIED!");
  }

  std::cout << "MICP load params - done. Valid Sensors: " << sensors_.size() << std::endl;
}

MICPSensorPtr make_sensor(rclcpp::Node::SharedPtr nh_sensor)
{
  MICPSensorPtr sensor;

  // std::string param_path = make_sub_parameter(nh_sensor, "~bla");
  // std::cout << param_path << std::endl;

  ParamTree<rclcpp::Parameter>::SharedPtr sensor_param_tree
    = get_parameter_tree(nh_sensor, "~");

  std::cout << "Param tree:" << std::endl;
  // std::cout << sensor_param_tree->name << std::endl;

  sensor_param_tree->print();
  
  if(!sensor_param_tree->exists("type"))
  {
    // ERROR!
    throw std::runtime_error("PARAM ERROR");
  }
  
  // fetch parameters that decide which implementation is loaded
  const std::string sensor_type = sensor_param_tree->at("type")->data->as_string();
  std::cout << "- Type: " << sensor_type << std::endl;
  const std::string model_source = sensor_param_tree->at("model_source")->data->as_string();  
  std::cout << "- Model Source: " << model_source << std::endl;
  const std::string data_source = sensor_param_tree->at("data_source")->data->as_string();
  std::cout << "- Data Source: " << data_source << std::endl;

  const std::string corr_backend = sensor_param_tree->at("correspondences")->at("backend")->data->as_string();
  std::cout << "- Correspondence Backend: " << corr_backend << std::endl;

  // model loader: if none , model is in data
  if(corr_backend == "embree" && sensor_type == "spherical")
  {
    sensor = std::make_shared<MICPSensorSphericalEmbree>();
  }

  if(data_source == "topic")
  {
    const std::string topic_name = sensor_param_tree->at("topic")->at("name")->data->as_string();
    const std::string topic_type = sensor_param_tree->at("topic")->at("type")->data->as_string();

    if(topic_type == "rmcl_msgs/msg/O1DnStamped")
    {
      auto bla = std::make_shared<dataloader::TopicSourceO1Dn>(nh_sensor, topic_name);

      // std::string map_filename = rmcl::get_parameter(nh_sensor, "/map_file", "");
      // std::cout << "TRY TO INJECT MAP IN TESTS!" << std::endl;
      // std::cout << map_filename << std::endl;
      // rm::EmbreeMapPtr map = rm::import_embree_map(map_filename);

      sensor->data_loader = bla;
    } else {
      // ERROR
      throw std::runtime_error("Topic type invalid or not implemented");
    }
  }

  return sensor;
}

MICPSensorPtr MICPLocalizationNode::loadSensor(
  ParamTree<rclcpp::Parameter>::SharedPtr sensor_params)
{
  std::string sensor_name = sensor_params->name;

  std::cout << "Loading '" << sensor_name << "'" << std::endl;

  rclcpp::Node::SharedPtr nh_sensor = this->create_sub_node("sensors/" + sensor_name);
  MICPSensorPtr sensor = make_sensor(nh_sensor);

  return sensor;
}

} // namespace rmcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::MICPLocalizationNode)
