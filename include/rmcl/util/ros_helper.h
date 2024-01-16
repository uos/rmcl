#ifndef RMCL_UTIL_ROS_HELPER_H
#define RMCL_UTIL_ROS_HELPER_H

#include <rclcpp/rclcpp.hpp>


namespace rmcl
{

// special case for char arrays to be converted properly
static std::string get_parameter(
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
static T get_parameter(
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

static std::optional<rclcpp::Parameter> get_parameter(
    rclcpp::Node::SharedPtr node, 
    const std::string& param_name)
{
    std::optional<rclcpp::Parameter> ret;

    if(node->has_parameter(param_name))
    {
        ret = node->get_parameter(param_name);
    }

    return ret;
}


// Forward declarations
template<typename ParamT>
class ParamTree;

template<typename ParamT>
using ParamTreePtr = std::shared_ptr<ParamTree<ParamT> >;

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

    inline void print(int indent = 0)
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
    rclcpp::Node::SharedPtr node,
    std::string prefix)
{
    ParamTree<rclcpp::Parameter>::SharedPtr ret;

    std::map<std::string, rclcpp::Parameter> param_map;
    if(node->get_parameters(prefix, param_map))
    {
        ret = std::make_shared<ParamTree<rclcpp::Parameter> >();
        for(auto elem : param_map)
        {
            ret->insert(elem.first, elem.second);
        }
    }
    ret->name = prefix;
    return ret;
}

} // namespace rmcl

#endif // RMCL_UTIL_ROS_HELPER_H