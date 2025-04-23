#ifndef RMCL_UTIL_ROS_HELPER_H
#define RMCL_UTIL_ROS_HELPER_H

#include <rclcpp/rclcpp.hpp>
#include <optional>

namespace rmcl
{

std::string replace_char(
  const std::string& input, 
  char old_char, 
  char new_char);

void replace_char_inplace(
  std::string& input, 
  char old_char, 
  char new_char);

std::string make_sub_parameter(
  rclcpp::Node* node,
  const std::string& param_name);

std::string make_sub_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string& param_name);

// special case for char arrays to be converted properly
std::string get_parameter(
    rclcpp::Node* node,
    const std::string& param_name,
    const std::string& default_value);

// special case for char arrays to be converted properly
std::string get_parameter(
    rclcpp::Node::SharedPtr node,
    const std::string& param_name,
    const std::string& default_value);

template<typename T> 
T get_parameter(
    rclcpp::Node* node, 
    const std::string& param_name,
    const T& default_value);

template<typename T> 
T get_parameter(
    rclcpp::Node::SharedPtr node, 
    const std::string& param_name,
    const T& default_value);

std::optional<rclcpp::Parameter> get_parameter(
    rclcpp::Node::SharedPtr node, 
    const std::string& param_name);


std::map<std::string, rclcpp::Parameter> get_parameters(
    rclcpp::Node* node,
    std::string prefix);

std::map<std::string, rclcpp::Parameter> get_parameters(
    rclcpp::Node::SharedPtr node,
    std::string prefix);




// Forward declarations
template<typename ParamT>
class ParamTree;

template<typename ParamT>
using ParamTreePtr = std::shared_ptr<ParamTree<ParamT> >;


/**
 * This tree structure lets us simply parse dynamic parameter sets
 * It is required to have the correct node settings:
 *
 * @code 
 * rclcpp::NodeOptions options = rclcpp::NodeOptions()
 *       .allow_undeclared_parameters(true)
 *       .automatically_declare_parameters_from_overrides(true);
 * @endcode
 */
template<typename ParamT>
class ParamTree : public std::unordered_map<std::string, std::shared_ptr<ParamTree<ParamT> > >
{
private:
  using Base = std::unordered_map<std::string, std::shared_ptr<ParamTree<ParamT> > >;
  using ThisType = ParamTree<ParamT>;
public:
  using SharedPtr = std::shared_ptr<ParamTree<ParamT> >;

  std::shared_ptr<ParamT> data;
  std::string name; // uid

  inline void insert(ParamTree<ParamT>::SharedPtr subtree)
  {
    if(Base::find(subtree->name) != Base::end())
    {
      // exists, need to merge
      for(auto elem : *subtree)
      {
        Base::at(subtree->name)->insert(elem.second);
      }
    } else {
      Base::insert(std::make_pair(subtree->name, subtree));
    }
  }

  inline void insert(std::string param_name, ParamT param)
  {
    size_t pos = param_name.find(".");
    if(pos != std::string::npos)
    {
      std::string first_name = param_name.substr(0, pos);
      std::string last_name = param_name.substr(pos + 1);

      ParamTree<ParamT>::SharedPtr child = std::make_shared<ParamTree>();
      child->name = first_name;
      child->insert(last_name, param); // recursion
      this->insert(child);
    } else {
      // leaf
      ParamTree<ParamT>::SharedPtr child = std::make_shared<ParamTree>();
      child->name = param_name;
      child->data = std::make_shared<ParamT>(param);
      this->insert(child);
    }
  }

  inline void print(size_t indent = 0)
  {
    for(size_t i=0; i<indent; i++) 
    {
      std::cout << "  ";
    }
    std::cout << name << std::endl;
    
    for(auto elem : *this)
    {
      elem.second->print(indent + 1);
    }
  }

  inline bool exists(std::string key) const{
    return Base::find(key) != Base::end();
  }
};

inline ParamTree<rclcpp::Parameter>::SharedPtr get_parameter_tree(
    rclcpp::Node* node,
    std::string prefix)
{
  ParamTree<rclcpp::Parameter>::SharedPtr ret;

  const std::string prefix_path = make_sub_parameter(node, prefix);
  const std::map<std::string, rclcpp::Parameter> param_map 
      = get_parameters(node, prefix_path);

  ret = std::make_shared<ParamTree<rclcpp::Parameter> >();
  for(auto elem : param_map)
  {
    ret->insert(elem.first, elem.second);
  }
  ret->name = prefix_path;
  return ret;
}

inline ParamTree<rclcpp::Parameter>::SharedPtr get_parameter_tree(
    rclcpp::Node::SharedPtr node,
    std::string prefix)
{
  return get_parameter_tree(node.get(), prefix);
}

} // namespace rmcl

#include "ros_helper.tcc"

#endif // RMCL_UTIL_ROS_HELPER_H