#include <rclcpp/rclcpp.hpp>


namespace rmcl
{

// special case for char arrays to be converted properly
inline std::string get_parameter(
    rclcpp::Node::SharedPtr node,
    const std::string param_name,
    const std::string default_value)
{
    std::string param_out;
    if(node->has_parameter(param_name))
    {
        node->get_parameter(param_name, param_out);
    } else {
        param_out = node->declare_parameter(param_name, default_value);
    }
    return param_out;
}

template<typename T> 
T get_parameter(
    rclcpp::Node::SharedPtr node, 
    const std::string& param_name,
    const T& default_value)
{
    T param_out;
    if(node->has_parameter(param_name))
    {
        node->get_parameter(param_name, param_out);
    } else {
        param_out = node->declare_parameter(param_name, default_value);
    }
    return param_out;
}

} // namespace rmcl