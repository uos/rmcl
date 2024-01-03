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

// Forward declarations
template<typename ParamT>
class ParamTree;

template<typename ParamT>
using ParamTreePtr = std::shared_ptr<ParamTree<ParamT> >;


// class ParamTree : public std::unordered_map<std::string, std::string >
// {

// };


// using ParamTree = 

inline std::vector<std::string> split_param_name(const std::string& s)
{
    std::vector<std::string> result;
    std::stringstream ss (s);
    std::string item;

    while(getline(ss, item, '.')) {
        result.push_back(item);
    }

    return result;
}

template<typename ParamT>
class ParamTree : public std::unordered_map<std::string, std::shared_ptr<ParamTree<ParamT> > >
{
private:
    using Base = std::unordered_map<std::string, std::shared_ptr<ParamTree<ParamT> > >;
public:
    using ThisType = ParamTree<ParamT>;
    using SharedPtr = std::shared_ptr<ParamTree<ParamT> >;

    std::unique_ptr<ParamT> data;
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
            child->data = std::make_unique<ParamT>(param);
            this->insert(child);
        }
    }


    inline void print(int indent = 0)
    {
        for(size_t i=0; i<indent; i++) 
        {
            std::cout << "  ";
        }
        std::cout << name;
        
        // if(data)
        // {
        //     std::cout << ": " << *data;
        // }

        std::cout << std::endl;
        
        for(auto elem : *this)
        {
            elem.second->print(indent + 1);
        }
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


// THANKS TO NAV2
// std::vector<std::vector<float> > parseVVF(const std::string & input, std::string & error_return)
// {
//   std::vector<std::vector<float>> result;

//   std::stringstream input_ss(input);
//   int depth = 0;
//   std::vector<float> current_vector;
//   while (!!input_ss && !input_ss.eof()) {
//     switch (input_ss.peek()) {
//       case EOF:
//         break;
//       case '[':
//         depth++;
//         if (depth > 2) {
//           error_return = "Array depth greater than 2";
//           return result;
//         }
//         input_ss.get();
//         current_vector.clear();
//         break;
//       case ']':
//         depth--;
//         if (depth < 0) {
//           error_return = "More close ] than open [";
//           return result;
//         }
//         input_ss.get();
//         if (depth == 1) {
//           result.push_back(current_vector);
//         }
//         break;
//       case ',':
//       case ' ':
//       case '\t':
//         input_ss.get();
//         break;
//       default:  // All other characters should be part of the numbers.
//         if (depth != 2) {
//           std::stringstream err_ss;
//           err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
//           error_return = err_ss.str();
//           return result;
//         }
//         float value;
//         input_ss >> value;
//         if (!!input_ss) {
//           current_vector.push_back(value);
//         }
//         break;
//     }
//   }

//   if (depth != 0) {
//     error_return = "Unterminated vector string.";
//   } else {
//     error_return = "";
//   }

//   return result;
// }


} // namespace rmcl