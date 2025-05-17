#include <rmcl_ros/util/ros_helper.h>
#include <sstream>
#include <filesystem>

namespace rmcl
{

std::string replace_char(
  const std::string& input, 
  char old_char, 
  char new_char) 
{
  std::string modified = input;
  for(char& c : modified) 
  {
    if(c == old_char) 
    {
      c = new_char; // Replace the character
    }
  }
  return modified;
}

void replace_char_inplace(
  std::string& input, 
  char old_char, 
  char new_char) 
{
  for(char& c : input) 
  {
    if(c == old_char) 
    {
      c = new_char; // Replace the character
    }
  }
}

std::string make_sub_parameter(
  rclcpp::Node* node,
  const std::string& param_name)
{
  if(param_name[0] == '~')
  {
    std::string param_prefix = node->get_sub_namespace();
    if(param_prefix != "")
    {
      replace_char_inplace(param_prefix, '/', '.');
      std::string param_postfix = param_name.substr(1);
      if(param_postfix != "")
      {
        param_prefix += "." + param_postfix;
      }
      return param_prefix;
    }
    else
    {
      return param_name.substr(1);
    }
  }
  else
  {
    return param_name;
  }
}

std::string make_sub_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string& param_name)
{
  return make_sub_parameter(node.get(), param_name);
}

// special case for char arrays to be converted properly
std::string get_parameter(
    rclcpp::Node* node,
    const std::string& param_name,
    const std::string& default_value)
{
  const std::string param_path = make_sub_parameter(node, param_name);
  if(node->has_parameter(param_path))
  {
    return node->get_parameter(param_path).as_string();
  }
  else
  {
    return node->declare_parameter<std::string>(param_path, default_value);
  }
}

// special case for char arrays to be converted properly
std::string get_parameter(
    rclcpp::Node::SharedPtr node,
    const std::string& param_name,
    const std::string& default_value)
{
  return get_parameter(node.get(), param_name, default_value);
}

std::optional<rclcpp::Parameter> get_parameter(
    rclcpp::Node::SharedPtr node, 
    const std::string& param_name)
{
  const std::string param_path = make_sub_parameter(node, param_name);
  if(node->has_parameter(param_path))
  {
    return node->get_parameter(param_path);
  }
  return std::nullopt;
}

std::map<std::string, rclcpp::Parameter> get_parameters(
    rclcpp::Node* node,
    std::string param_name)
{
  std::map<std::string, rclcpp::Parameter> param_map;

  const std::string param_path = make_sub_parameter(node, param_name);
  if(!node->get_parameters(param_path, param_map))
  {
    param_map.clear();
  }
  
  return param_map;
}

std::map<std::string, rclcpp::Parameter> get_parameters(
    rclcpp::Node::SharedPtr node,
    std::string prefix)
{
  return get_parameters(node.get(), prefix);
}

} // namespace rmcl
