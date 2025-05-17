#include "ros_helper.h"

namespace rmcl
{

template<typename T> 
T get_parameter(
    rclcpp::Node* node, 
    const std::string& param_name,
    const T& default_value)
{
  const std::string param_path = make_sub_parameter(node, param_name);
  T param_out;
  if(node->has_parameter(param_path))
  {
    param_out = node->get_parameter(param_path).get_value<T>();
  }
  else
  {
    param_out = node->declare_parameter<T>(param_path, default_value);
  }
  return param_out;
}

template<typename T> 
T get_parameter(
    rclcpp::Node::SharedPtr node, 
    const std::string& param_name,
    const T& default_value)
{
  return get_parameter(node.get(), param_name, default_value);
}

} // namespace rmcl