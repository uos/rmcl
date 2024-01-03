#include "rmcl/correction/MICP.hpp"
// how to port this?
// #include <ros/master.h>
#include <vector>


#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rmcl/util/conversions.h>
#include <rmcl/util/ros_helper.h>

#include <rmcl/math/math.h>

#ifdef RMCL_CUDA
#include <rmcl/math/math.cuh>
#include <rmagine/math/math.cuh>
#endif // RMCL_CUDA

#include <rmagine/util/StopWatch.hpp>

#include <rclcpp/wait_for_message.hpp>

#include <chrono>

#include <rclcpp/wait_for_message.hpp>

using namespace std::chrono_literals;

namespace rm = rmagine;

namespace rmcl
{

MICP::MICP(rclcpp::Node::SharedPtr node)
:m_nh(node)
,m_tf_buffer(new tf2_ros::Buffer(m_nh->get_clock()))
,m_tf_listener(new tf2_ros::TransformListener(*m_tf_buffer))
{
    std::cout << "MICP initiailized" << std::endl;

    std::cout << std::endl;
    std::cout << "-------------------------" << std::endl;
    std::cout << "    --- BACKENDS ---    " << std::endl;
    std::cout << "-------------------------" << std::endl;

    std::cout << "Available combining units:" << std::endl;
    std::cout << "- " << TC_BACKENDS << "CPU" << TC_END << std::endl;
    #ifdef RMCL_CUDA
    std::cout << "- " << TC_BACKENDS << "GPU" << TC_END << std::endl; 
    #endif // RMCL_CUDA

    std::string combining_unit_str = get_parameter(m_nh, "micp.combining_unit", "cpu");

    if(combining_unit_str == "cpu")
    {
        std::cout << "Selected Combining Unit: " << TC_BACKENDS << "CPU" << TC_END << std::endl;
    } else if(combining_unit_str == "gpu") {
        std::cout << "Selected Combining Unit: " << TC_BACKENDS << "GPU" << TC_END << std::endl;
    } else {
        // ERROR
        std::cout << "Combining Unit: " << TC_RED << combining_unit_str << " unknown!" << TC_END << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Available raytracing backends:" << std::endl;
    #ifdef RMCL_EMBREE
    std::cout << "- " << TC_BACKENDS << "Embree (CPU)" << TC_END << std::endl;
    #endif // RMCL_EMBREE

    #ifdef RMCL_OPTIX
    std::cout << "- " << TC_BACKENDS << "Optix (GPU)" << TC_END << std::endl;
    #endif // RMCL_OPTIX

    std::cout << std::endl;
}

MICP::~MICP()
{
    // std::cout << "MICP cleanup" << std::endl;
}

void MICP::loadParams()
{
    std::cout << "MICP load params" << std::endl;

    // loading frames
    m_base_frame = get_parameter(m_nh, "base_frame", "base_link");
    m_map_frame = get_parameter(m_nh, "map_frame", "map");

    m_odom_frame = get_parameter(m_nh, "odom_frame", "");
    m_use_odom_frame = (m_odom_frame != "");
    // check frames

    m_map_filename = get_parameter(m_nh, "map_file", "");
    if(m_map_filename == "")
    {
        RCLCPP_ERROR(m_nh->get_logger(), "User must provide ~map_file");
        throw std::runtime_error("User must provide ~map_file");
    }

    checkTF(true);

    loadMap(m_map_filename);

    // XmlRpc::XmlRpcValue sensors_xml;
    // rclcpp::Parameter sensors_param;
    // std::vector<rclcpp::Parameter> sensors_param;


    // FROM MESH TOOLS TESTS:
    // std::cout << "Searching parameters for: " << node->get_fully_qualified_name() << std::endl;

    // std::map<std::string, rclcpp::Parameter> plugin_params;
    // node->get_parameters("rviz_mesh_tools_plugins", plugin_params);

    // std::cout << "PLUGIN PARAMS:" << std::endl;
    // for(auto elem : plugin_params)
    // {
    //   std::cout << "- " << elem.first << std::endl;
    // }

    std::map<std::string, rclcpp::Parameter> sensors_param;

    if(m_nh->get_parameters("sensors", sensors_param))
    {
        std::cout << std::endl;
        std::cout << "-------------------------" << std::endl;
        std::cout << "     --- SENSORS ---     " << std::endl;
        std::cout << "-------------------------" << std::endl;

        ParamTree<rclcpp::Parameter>::SharedPtr sensors_param_tree = get_parameter_tree(m_nh, "sensors");
        for(auto elem : *sensors_param_tree)
        {
            std::cout << "LOAD SENSOR " << elem.first << std::endl;
            loadSensor(elem.second);
        }

        throw std::runtime_error("TODO: parse new param");
        // for(auto sensor_xml : sensors_xml)
        // {
        //     loadSensor(sensor_xml.first, sensor_xml.second);
        // }
    } else {
        std::cout << "ERROR: NO SENSORS" << std::endl;
    }

    std::cout << "MICP load params - done. Valid Sensors: " << m_sensors.size() << std::endl;
}

bool MICP::loadSensor(
    ParamTree<rclcpp::Parameter>::SharedPtr sensor_params)
{
    std::string sensor_name = sensor_params->name;

    MICPRangeSensorPtr sensor = std::make_shared<MICPRangeSensor>();

    bool loading_error = false;

    // std::string sensor_name = sensor_xml.first;
    std::string sensor_type;
    bool        sensor_type_found = false;
    // std::string topic_msg;
    // bool        topic_valid = false;

    sensor->name = sensor_name;
    sensor->base_frame = m_base_frame;

    // std::cout << "PARAMS: " << std::endl;
    // for(auto param : *sensor_params)
    // {
    //     std::cout << param.first << std::endl;
    // }

    if(sensor_params->find("type") != sensor_params->end())
    {
        auto param = sensor_params->at("type")->data;

        // std::cout << "Searching for parameter: " << ss.str() << std::endl;
        sensor_type = param->as_string();
        if(sensor_type == "spherical") {
            sensor->type = 0;
        } else if(sensor_type == "pinhole") {
            sensor->type = 1;
        } else if(sensor_type == "o1dn") {
            sensor->type = 2;
        } else if(sensor_type == "ondn") {
            sensor->type = 3;
        } else {
            std::cout << "ERROR sensor type unknown: " << sensor_type << std::endl;
            std::cout << "- supported: spherical, pinhole, o1dn, ondn" << std::endl;
            std::cout << "- try to reconstruct type from other params" << std::endl; 
        }
        sensor_type_found = true;
    }

    if(sensor_params->find("frame") != sensor_params->end())
    {
        auto param = sensor_params->at("frame")->data;
        sensor->frame = param->as_string();
    }

    std::cout << "- " << TC_SENSOR << sensor_name << TC_END << std::endl;

    // load data or connect to a data topic
    if(sensor_params->find("topic") != sensor_params->end())
    {
        std::cout << "  - data:\t\tTopic" << std::endl;
        // std::cout << "has topic" << std::endl;
        std::string topic_name;
        {
            topic_name = sensor_params->at("topic")->data->as_string();

            if(topic_name[0] != '/')
            {
                topic_name = m_nh->get_namespace() + topic_name;
            }
        }
        
        sensor->data_topic.name = topic_name;
        std::cout << "    - topic:\t\t" << TC_TOPIC << sensor->data_topic.name << TC_END << std::endl;


        if(sensor_params->find("topic_type") != sensor_params->end())
        {
            sensor->data_topic.msg = sensor_params->at("topic_type")->data->as_string();
            std::cout << "FOUND TOPIC TYPE: " << sensor->data_topic.msg << std::endl;
        }
        

        std::map<std::string, std::vector<std::string> > topic_map = m_nh->get_topic_names_and_types();
        
        // for(auto elem : topic_map)
        // {
        //     std::cout << elem.first << ": "  << std::endl;
        //     for(auto topic_type : elem.second)
        //     {
        //         std::cout << "-- " << topic_type << std::endl; 
        //     }
        // }

        if(topic_map.find(sensor->data_topic.name) != topic_map.end())
        {
            // found topic
            auto topic_types = topic_map[sensor->data_topic.name];
            
            if(topic_types.size() > 1)
            {
                std::stringstream ss;
                ss << "Cannot handle topics that have more than one type!\n";
                ss << sensor->data_topic.name << ":\n";
                for(auto topic_type : topic_types)
                {
                    ss << "-- " << topic_type << "\n";
                }
                throw std::runtime_error(ss.str());
            }

            std::string topic_type = topic_types[0];

            if(sensor->data_topic.msg != "")
            {
                // compare actual topic type with user input
                // if they dont match. act accordingly
                if(sensor->data_topic.msg != topic_type)
                {
                    // WARNING
                    std::cout << "WARNING: Topic type mismatch found:" << std::endl;
                    std::cout << "-- user input: " << sensor->data_topic.msg << std::endl;
                    std::cout << "-- topic type: " << topic_type << std::endl;
                    std::cout << "Using actual topic type" << std::endl;
                }
            }

            sensor->data_topic.msg = topic_type;

        } else {
            // could not find topic. maybe its not existing yet?
            // suggestion how to handle it: (TODO implement properly)
            if(sensor->data_topic.msg == "")
            {
                // empty type
                std::cout << "ERROR: TOPIC '" << sensor->data_topic.name << "' IS NOT EXISTING" << std::endl;
            } else {
                std::cout << "WAITING FOR TOPIC '" << sensor->data_topic.name << "' TO APPEAR" << std::endl;
            }
        }

        if(sensor->data_topic.msg != "")
        {
            if(sensor->data_topic.msg == "sensor_msgs/msg/LaserScan" 
                || sensor->data_topic.msg == "rmcl_msgs/msg/ScanStamped")
            {
                sensor_type = "spherical";
                sensor_type_found = true;
                sensor->type = 0;
            } else if(sensor->data_topic.msg == "sensor_msgs/msg/Image" 
                || sensor->data_topic.msg == "rmcl_msgs/msg/DepthStamped") 
            {
                // is image always pinhole? counterexample: cylindrical image e.g. panorama
                sensor_type = "pinhole";
                sensor_type_found = true;
                sensor->type = 1;
            }

            std::cout << "    - msg:\t\t" << TC_MSG << sensor->data_topic.msg << TC_END << std::endl;
            // check if topic is valid

            checkTopic(sensor->data_topic, rclcpp::Duration(5s));

            if(sensor->data_topic.data)
            {
                sensor->frame = sensor->data_topic.frame;
                std::cout << "    - data:\t\t" << TC_GREEN << "yes" << TC_END << std::endl;
                std::cout << "    - frame:\t\t" << TC_FRAME << sensor->frame << TC_END << std::endl;
            } else {
                std::cout << "    - data:\t\t" << TC_RED << "no" << TC_END << std::endl;
            }
        } else {
            std::cout << "    - msg:\t\t" << TC_RED << "not found" << TC_END << std::endl;
            loading_error = true;
        }
    } else if(sensor_params->find("ranges") != sensor_params->end()) {
        // std::cout << "  - topic:\t\t" << TC_RED << "not found" << TC_END << std::endl;
        // check if there is data in the parameters instead

        std::cout << "  - data:\t\tParams" << std::endl;

        std::vector<double> ranges = sensor_params->at("ranges")->data->as_double_array();

        sensor->ranges.resize(ranges.size());
        for(size_t i=0; i<ranges.size(); i++)
        {
            sensor->ranges[i] = ranges[i];
        }
        #ifdef RMCL_CUDA
        sensor->ranges_gpu = sensor->ranges;
        #endif // RMCL_CUDA
        sensor->data_received_once = true;

    } else {
        std::cout << "Where is the data?" << std::endl;
        loading_error = true;
    }

    // std::cout << "LOADING MODEL" << std::endl;

    // Loading model params
    // 1. Params: model parameters are listed as ROS parameters (static)
    // 2. Topic: model parameters are received of a special info topic (dynamic)
    // 3. Data: model parameters are included in the data

    bool model_loaded = false;
    if(sensor_params->find("model") != sensor_params->end()) /// PARAMS
    {
        std::cout << "  - model source:\t\tParams" << std::endl;

        auto model_params = sensor_params->at("model");

        if(sensor_type == "spherical") {
            rm::SphericalModel model;

            // fill
            model.theta.min = model_params->at("theta_min")->data->as_double();
            model.theta.inc = model_params->at("theta_inc")->data->as_double();
            model.theta.size = model_params->at("theta_n")->data->as_int();

            model.phi.min = model_params->at("phi_min")->data->as_double();;
            model.phi.inc = model_params->at("phi_inc")->data->as_double();;
            model.phi.size = model_params->at("phi_n")->data->as_int();;

            model.range.min = model_params->at("range_min")->data->as_double();
            model.range.max = model_params->at("range_max")->data->as_double();

            sensor->model = model;
            model_loaded = true;
        } else if(sensor_type == "pinhole") {
            rm::PinholeModel model;

            model.width  = model_params->at("width")->data->as_int();
            model.height = model_params->at("height")->data->as_int();

            model.f[0] = model_params->at("f")->data->as_double_array()[0];
            model.f[1] = model_params->at("f")->data->as_double_array()[1];
            model.c[0] = model_params->at("c")->data->as_double_array()[0];
            model.c[1] = model_params->at("c")->data->as_double_array()[1];

            model.range.min = model_params->at("range_min")->data->as_double();
            model.range.max = model_params->at("range_max")->data->as_double();

            sensor->model = model;
            model_loaded = true;
        } else if(sensor_type == "o1dn") {
            rm::O1DnModel model;
            bool model_loading_error = false;

            model.width  = model_params->at("width")->data->as_int();
            model.height = model_params->at("height")->data->as_int();

            model.range.min = model_params->at("range_min")->data->as_double();
            model.range.max = model_params->at("range_max")->data->as_double();


            std::vector<double> orig = model_params->at("orig")->data->as_double_array();

            if(orig.size() != 3)
            {
                // error
                std::cout << "ERROR: origin point is not 3D" << std::endl;
            }

            model.orig.x = orig[0];
            model.orig.y = orig[1];
            model.orig.z = orig[2];

            std::vector<double> dirs = model_params->at("dirs")->data->as_double_array();
            model.dirs.resize(dirs.size() / 3);
            for(size_t i=0; i<dirs.size() / 3; i++)
            {
                model.dirs[i]  = {dirs[i * 3 + 0], dirs[i * 3 + 1], dirs[i * 3 + 2]};
            }

            sensor->model = model;
            model_loaded = !model_loading_error;
        } else if(sensor_type == "ondn") {
            rm::OnDnModel model;

            bool model_loading_error = false;

            model.width  = model_params->at("width")->data->as_int();
            model.height = model_params->at("height")->data->as_int();

            model.range.min = model_params->at("range_min")->data->as_double();
            model.range.max = model_params->at("range_max")->data->as_double();

            std::vector<double> origs = model_params->at("origs")->data->as_double_array();
            std::vector<double> dirs = model_params->at("dirs")->data->as_double_array();

            model.origs.resize(origs.size() / 3);
            model.dirs.resize(dirs.size() / 3);
            for(size_t i=0; i<origs.size() / 3; i++)
            {
                model.origs[i] = {origs[i * 3 + 0], origs[i * 3 + 1], origs[i * 3 + 2]};
                model.dirs[i]  = {dirs[i * 3 + 0], dirs[i * 3 + 1], dirs[i * 3 + 2]};
            }
           
            sensor->model = model;
            model_loaded = !model_loading_error;
        } else {
            // ERROR
            std::cout << "Model type '" << sensor->type << "' not supported." << std::endl;
            loading_error = true;
        }

    } else if(sensor_params->find("model_topic") != sensor_params->end()) { /// TOPIC

        std::cout << "  - model source:\t\tTopic" << std::endl;

        sensor->has_info_topic = true;

        std::string info_topic_name = sensor_params->at("model_topic")->data->as_string();
        if(info_topic_name[0] != '/')
        {
            info_topic_name = m_nh->get_namespace() + info_topic_name;
        }
        sensor->info_topic.name = info_topic_name;

        std::cout << "    - topic:\t\t" << TC_TOPIC << sensor->info_topic.name  << TC_END << std::endl;


        std::map<std::string, std::vector<std::string> > topic_map = m_nh->get_topic_names_and_types();
        
        if(topic_map.find(sensor->info_topic.name) != topic_map.end())
        {
            auto topic_types = topic_map[sensor->info_topic.name];
            
            if(topic_types.size() > 1)
            {
                std::stringstream ss;
                ss << "Cannot handle topics that have more than one type!\n";
                ss << sensor->info_topic.name << ":\n";
                for(auto topic_type : topic_types)
                {
                    ss << "-- " << topic_type << "\n";
                }
                throw std::runtime_error(ss.str());
            }

            std::string topic_type = topic_types[0];

            if(sensor->info_topic.msg != "")
            {
                // compare actual topic type with user input
                // if they dont match. act accordingly
                if(sensor->info_topic.msg != topic_type)
                {
                    // WARNING
                    std::cout << "WARNING: Topic type mismatch found:" << std::endl;
                    std::cout << "-- user input: " << sensor->info_topic.msg << std::endl;
                    std::cout << "-- topic type: " << topic_type << std::endl;
                    std::cout << "Using actual topic type" << std::endl;
                }
            }

            sensor->info_topic.msg = topic_type;
        }

        



        // // std::vector<ros::master::TopicInfo> topic_infos;
        // // ros::master::getTopics(topic_infos);
        // for(auto topic_info : topic_infos)
        // {
        //     if(topic_info.name == info_topic_name)
        //     {
        //         sensor->info_topic.msg = topic_info.datatype;
        //         break;
        //     }
        // }
        // throw std::runtime_error("TODO");

        if(sensor->info_topic.msg != "")
        {
            std::cout << "    - msg:\t\t" << TC_MSG << sensor->info_topic.msg << TC_END << std::endl;
        } else {
            std::cout << "    - msg:\t\t" << TC_RED << "not found" << TC_END << std::endl;
            loading_error = true;
        }

        if(!sensor_type_found)
        {
            if(sensor->info_topic.msg == "sensor_msgs/CameraInfo")
            {
                sensor_type = "pinhole";
                sensor_type_found = true;
                sensor->type = 1;
            }
        }

        // at this point the sensor type must be known
        if(!sensor_type_found)
        {
            loading_error = true;
        }

        if(sensor_type == "spherical")
        {
            if(sensor->info_topic.msg == "rmcl_msgs/msg/ScanInfo")
            {
                rmcl_msgs::msg::ScanInfo msg;
                if(rclcpp::wait_for_message(msg, m_nh, sensor->info_topic.name, 3s))
                {
                    rm::SphericalModel model;
                    convert(msg, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "ERROR: Could not receive initial pinhole model!" << std::endl;
                }
            }
        } else if(sensor_type == "pinhole") {
            
            if(sensor->info_topic.msg == "sensor_msgs/msg/CameraInfo")
            {
                // std::cout << "Waiting for message on topic: " << sensor->info_topic.name << std::endl;
                sensor_msgs::msg::CameraInfo msg;
                if(rclcpp::wait_for_message(msg, m_nh, sensor->info_topic.name, 3s))
                {
                    if(msg.header.frame_id != sensor->frame)
                    {
                        std::cout << "WARNING: Image and CameraInfo are not in the same frame" << std::endl;
                    }
                    
                    rm::PinholeModel model;
                    convert(msg, model);

                    // manually setting range limits
                    // TODO: change this
                    model.range.min = 0.3;
                    model.range.max = 8.0;

                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "ERROR: Could not receive initial pinhole model!" << std::endl;
                }
            }

            if(sensor->info_topic.msg == "rmcl_msgs/msg/DepthInfo")
            {
                rmcl_msgs::msg::DepthInfo msg;
                if(rclcpp::wait_for_message(msg, m_nh, sensor->info_topic.name, 3s))
                {
                    rm::PinholeModel model;
                    convert(msg, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "ERROR: Could not receive initial pinhole model!" << std::endl;
                }
            }


        } else if(sensor_type == "o1dn") {
            
            if(sensor->info_topic.msg == "rmcl_msgs/msg/O1DnInfo")
            {
                rmcl_msgs::msg::O1DnInfo msg;
                if(rclcpp::wait_for_message(msg, m_nh, sensor->info_topic.name, 3s))
                {
                    rm::O1DnModel model;
                    convert(msg, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "ERROR: Could not receive initial o1dn model!" << std::endl;
                }
            }

        } else if(sensor_type == "ondn") {
            if(sensor->info_topic.msg == "rmcl_msgs/msg/OnDnInfo")
            {
                rmcl_msgs::msg::OnDnInfo msg;
                if(rclcpp::wait_for_message(msg, m_nh, sensor->info_topic.name, 3s))
                {
                    rm::OnDnModel model;
                    convert(msg, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "ERROR: Could not receive initial ondn model!" << std::endl;
                }
            }
        }

    } else { /// DATA
        std::cout << "  - model source:\t\tData" << std::endl;

        if(sensor_type == "spherical")
        {
            if(!model_loaded && sensor->data_topic.msg == "sensor_msgs/msg/LaserScan")
            {
                sensor_msgs::msg::LaserScan msg;
                if(rclcpp::wait_for_message(msg, m_nh, sensor->data_topic.name, 3s))
                {
                    rm::SphericalModel model;
                    convert(msg, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "error: get sensor_msgs/msg/LaserScan to init model" << std::endl;
                }
            }
            
            if(!model_loaded && sensor->data_topic.msg == "rmcl_msgs/msg/ScanStamped")
            {
                rmcl_msgs::msg::ScanStamped msg;
                if(rclcpp::wait_for_message(msg, m_nh, sensor->data_topic.name, 3s))
                {
                    rm::SphericalModel model;
                    convert(msg.scan.info, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "error: get rmcl_msgs/msg/ScanStamped to init model" << std::endl;
                }
            }
        } else if(sensor_type == "pinhole") {

            if(!model_loaded && sensor->data_topic.msg == "rmcl_msgs/msg/DepthStamped")
            {
                rmcl_msgs::msg::DepthStamped msg;
                if(rclcpp::wait_for_message(msg, m_nh, sensor->data_topic.name, 3s))
                {
                    rm::PinholeModel model;
                    convert(msg.depth.info, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "error: get rmcl_msgs/msg/DepthStamped to init model" << std::endl;
                }
            }

        } else if(sensor_type == "o1dn") {

            if(!model_loaded && sensor->data_topic.msg == "rmcl_msgs/msg/O1DnStamped")
            {
                rmcl_msgs::msg::O1DnStamped msg;
                if(rclcpp::wait_for_message(msg, m_nh, sensor->data_topic.name, 3s))
                {
                    rm::O1DnModel model;
                    convert(msg.o1dn.info, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "error: get rmcl_msgs/msg/DepthStamped to init model" << std::endl;
                }
            }
        } else if(sensor_type == "ondn") {

            if(!model_loaded && sensor->data_topic.msg == "rmcl_msgs/msg/OnDnStamped")
            {
                rmcl_msgs::msg::OnDnStamped msg;
                if(rclcpp::wait_for_message(msg, m_nh, sensor->data_topic.name, 3s))
                {
                    rm::OnDnModel model;
                    convert(msg.ondn.info, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "error: get rmcl_msgs/msg/DepthStamped to init model" << std::endl;
                }
            }
        }
    }

    // print results
    if(sensor_type_found)
    {
        if(model_loaded)
        {
            std::cout << "  - type:\t\t" << sensor_type << " - loaded" << std::endl;
        } else {
            std::cout << "  - type:\t\t" << sensor_type << " - " << TC_RED << "loading error" << TC_END << std::endl;
        }
    } else {
        std::cout << "  - type:\t\t" << TC_RED << "unknown" << TC_END << std::endl;
    }

    // MICP params
    if(sensor_params->find("micp") != sensor_params->end())
    {
        std::cout << "  - micp:" << std::endl;
        auto micp_params = sensor_params->at("micp");

        if(micp_params->find("backend") != micp_params->end())
        {
            std::string backend_name = micp_params->at("backend")->data->as_string();
            if(backend_name == "embree")
            {
                sensor->backend = 0;
                std::cout << "    - backend:\t\t" << TC_BLUE << "embree" << TC_END << std::endl;
            } else if(backend_name == "optix") {
                sensor->backend = 1;
                std::cout << "    - backend:\t\t" << TC_BLUE << "optix" << TC_END << std::endl;
            } else {
                // error
                std::cout << "    - backend:\t\t" << TC_RED << backend_name << " - unknown" << TC_END << std::endl;
            }
        } else {
            #ifdef RMCL_EMBREE
            sensor->backend = 0;
            #endif // RMCL_EMBREE

            #ifdef RMCL_OPTIX
            sensor->backend = 1;
            #endif // RMCL_OPTIX
        }

        if(micp_params->find("max_dist") != micp_params->end())
        {
            sensor->corr_params_init.max_distance = micp_params->at("max_dist")->data->as_double();
        }
        sensor->corr_params = sensor->corr_params_init;


        if(micp_params->find("adaptive_max_dist_min") != micp_params->end())
        {
            sensor->adaptive_max_dist_min = micp_params->at("adaptive_max_dist_min")->data->as_double();
        }

        if(micp_params->find("weight") != micp_params->end())
        {
            sensor->corr_weight = micp_params->at("weight")->data->as_double();
        } else {
            sensor->corr_weight = 1.0;
        }
        
    } else {
        // taking fastest
        // order speed descending here
        #ifdef RMCL_EMBREE
        sensor->backend = 0;
        #endif // RMCL_EMBREE

        #ifdef RMCL_OPTIX
        sensor->backend = 1;
        #endif // RMCL_OPTIX
    }

    sensor->optical_coordinates = (sensor->frame.find("_optical") != std::string::npos);
    
    if(sensor->optical_coordinates)
    {
        std::cout << "  - optical frame" << std::endl;
    }

    if(loading_error)
    {
        return false;
    }
    
    // std::cout << "POSTPROCESS" << std::endl;


    // ////////////////////////
    // //// 2. POSTPROCESS ////
    // ////////////////////////

    // // postcheck
    // // check if optical: _optical suffix
    // // std::cout << "Searching for '_optical' in " << sensor->data_topic.frame << std::endl;
    
    // // connect sensor to ROS
    // sensor->nh = m_nh;
    // sensor->tf_buffer = m_tf_buffer; 

    // // load additional params: duplicated from above
    // sensor->fetchMICPParams();
    // // connect to sensor topics
    // sensor->connect();

    // // add sensor to class
    // m_sensors[sensor->name] = sensor;

    // #ifdef RMCL_EMBREE
    // if(sensor->type == 0) // spherical
    // {
    //     sensor->corr_sphere_embree = std::make_shared<SphereCorrectorEmbree>(m_map_embree);
    // } else if(sensor->type == 1) {
    //     sensor->corr_pinhole_embree = std::make_shared<PinholeCorrectorEmbree>(m_map_embree);;
    // } else if(sensor->type == 2) {
    //     sensor->corr_o1dn_embree = std::make_shared<O1DnCorrectorEmbree>(m_map_embree);
    // } else if(sensor->type == 3) {
    //     sensor->corr_ondn_embree = std::make_shared<OnDnCorrectorEmbree>(m_map_embree);
    // }
    // #endif // RMCL_EMBREE

    // #ifdef RMCL_OPTIX
    // if(sensor->type == 0) // spherical
    // {
    //     sensor->corr_sphere_optix = std::make_shared<SphereCorrectorOptix>(m_map_optix);
    // } else if(sensor->type == 1) {
    //     sensor->corr_pinhole_optix = std::make_shared<PinholeCorrectorOptix>(m_map_optix);
    // } else if(sensor->type == 2) {
    //     sensor->corr_o1dn_optix = std::make_shared<O1DnCorrectorOptix>(m_map_optix);
    // } else if(sensor->type == 3) {
    //     sensor->corr_ondn_optix = std::make_shared<OnDnCorrectorOptix>(m_map_optix);
    // }
    // #endif // RMCL_OPTIX

    
    // sensor->fetchTF();
    // sensor->updateCorrectors();
    
    throw std::runtime_error("TODO");

    return true;
}

void MICP::loadMap(std::string filename)
{
    #ifdef RMCL_EMBREE
    m_map_embree = rm::import_embree_map(filename);
    setMap(m_map_embree);
    #else 
    m_corr_cpu = std::make_shared<Correction>();
    #endif // RMCL_EMBREE

    #ifdef RMCL_OPTIX
    m_map_optix = rm::import_optix_map(filename);
    setMap(m_map_optix);
    #else
    #ifdef RMCL_CUDA
    // initialize cuda correction without optix
    m_corr_gpu = std::make_shared<CorrectionCuda>(); 
    #endif // RMCL_CUDA
    #endif // RMCL_OPTIX
}

#ifdef RMCL_EMBREE
void MICP::setMap(rmagine::EmbreeMapPtr map)
{
    m_map_embree = map;
    m_corr_cpu = std::make_shared<Correction>();

    // update sensors
    for(auto elem : m_sensors)
    {
        elem.second->setMap(map);
    }
}
#endif // RMCL_EMBREE

#ifdef RMCL_OPTIX
void MICP::setMap(rmagine::OptixMapPtr map)
{
    m_map_optix = map;
    m_corr_gpu = std::make_shared<CorrectionCuda>(m_map_optix->scene()->stream());
    
    // update sensors
    for(auto elem : m_sensors)
    {
        elem.second->setMap(map);
    }
}
#endif // RMCL_OPTIX


#ifdef RMCL_CUDA
void MICP::correct(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbm,
    const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm_,
    CorrectionPreResults<rm::VRAM_CUDA>& pre_res,
    rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& dT)
{
    // std::cout << "correct runtimes: " << std::endl;
    // rm::StopWatch sw;
    // double el;
    // double el_total = 0.0;


    // sw();
    // extra memory
    std::vector<CorrectionPreResults<rm::VRAM_CUDA> > results(m_sensors.size());
    float weight_sum = 0.0;
    std::vector<float> weights(m_sensors.size());

    for(auto& elem : results)
    {
        elem.ms.resize(Tbm.size());
        elem.ds.resize(Tbm.size());
        elem.Cs.resize(Tbm.size());
        elem.Ncorr.resize(Tbm.size());
    }

    // sw();
    size_t id = 0;
    for(auto elem : m_sensors)
    {
        CorrectionPreResults<rm::VRAM_CUDA>& res_ = results[id];

        if(elem.second->data_received_once)
        {
            #ifdef RMCL_EMBREE
            if(elem.second->backend == 0)
            {
                CorrectionPreResults<rm::RAM> res;
                res.ms.resize(Tbm.size());
                res.ds.resize(Tbm.size());
                res.Cs.resize(Tbm.size());
                res.Ncorr.resize(Tbm.size());

                // compute
                elem.second->computeCovs(Tbm, res);

                // upload
                res_.ms = res.ms;
                res_.ds = res.ds;
                res_.Cs = res.Cs;
                res_.Ncorr = res.Ncorr;
            } else 
            #endif // RMCL_EMBREE
            #ifdef RMCL_OPTIX
            if(elem.second->backend == 1) {
                // use preuploaded poses as input
                elem.second->computeCovs(Tbm_, res_);
            } else
            #endif // RMCL_OPTIX
            {
                std::cout << "backend " << elem.second->backend << " unknown" << std::endl;
            }

            // dynamic weights
            float w = elem.second->corr_weight;
            weight_sum += w;
            weights[id] = w;
        } else {
            std::cout << "WARNING: still waiting for sensor data of " << elem.second->name << std::endl; 
            weights[id] = 0.0;
        }

        id++;
    }
    
    if(results.size() > 0)
    {
        // normalize weights
        // sw();
        for(size_t i=0; i<weights.size(); i++)
        {
            weights[i] /= weight_sum;
        }

        // CorrectionPreResults<rm::VRAM_CUDA> results_combined;
        // pre_res;
        if(pre_res.ms.size() < Tbm.size())
        {
            pre_res.ms.resize(Tbm.size());
            pre_res.ds.resize(Tbm.size());
            pre_res.Cs.resize(Tbm.size());
            pre_res.Ncorr.resize(Tbm.size());
        }

        weighted_average(
            results,
            weights,
            pre_res);

        m_corr_gpu->correction_from_covs(pre_res, dT);
    } else {
        std::cout << "0 sensors" << std::endl;
        // set identity
        rm::setIdentity(dT);
    }
}
#endif // RMCL_CUDA

#ifdef RMCL_CUDA
void MICP::correct(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbm,
    const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm_,
    CorrectionPreResults<rmagine::RAM>& pre_res,
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& dT)
{
    // extra memory
    std::vector<CorrectionPreResults<rm::RAM> > results(m_sensors.size());
    float weight_sum = 0.0;
    std::vector<float> weights(m_sensors.size());

    for(auto& elem : results)
    {
        elem.ms.resize(Tbm.size());
        elem.ds.resize(Tbm.size());
        elem.Cs.resize(Tbm.size());
        elem.Ncorr.resize(Tbm.size());
    }

    size_t id = 0;
    for(auto elem : m_sensors)
    {
        CorrectionPreResults<rm::RAM>& res = results[id];
        if(elem.second->data_received_once)
        {
            #ifdef RMCL_EMBREE
            if(elem.second->backend == 0)
            {
                // compute
                elem.second->computeCovs(Tbm, res);
            } else 
            #endif // RMCL_EMBREE
            #ifdef RMCL_OPTIX
            if(elem.second->backend == 1) {
                CorrectionPreResults<rm::VRAM_CUDA> res_;
                res_.ms.resize(Tbm.size());
                res_.ds.resize(Tbm.size());
                res_.Cs.resize(Tbm.size());
                res_.Ncorr.resize(Tbm.size());

                // use preuploaded poses as input
                elem.second->computeCovs(Tbm_, res_);

                // download
                res.ms = res_.ms;
                res.ds = res_.ds;
                res.Cs = res_.Cs;
                res.Ncorr = res_.Ncorr;
            } else
            #endif // RMCL_OPTIX
            {
                std::cout << "backend " << elem.second->backend << " unknown" << std::endl;
            }


            // dynamic weights
            float w = elem.second->corr_weight;
            weight_sum += w;
            weights[id] = w;
        } else {
            std::cout << "WARNING: still waiting for sensor data of " << elem.second->name << std::endl; 
            weights[id] = 0.0;
        }

        id++;
    }
    
    if(results.size() > 0)
    {
        // normalize weights
        // sw();
        for(size_t i=0; i<weights.size(); i++)
        {
            weights[i] /= weight_sum;
        }


        if(pre_res.ms.size() < Tbm.size())
        {
            pre_res.ms.resize(Tbm.size());
            pre_res.ds.resize(Tbm.size());
            pre_res.Cs.resize(Tbm.size());
            pre_res.Ncorr.resize(Tbm.size());
        }

        // TODO: 

        // el = sw();
        // el_total += el;
        // std::cout << "- merging preprocessing: " << el * 1000.0 << " ms" << std::endl;

        // sw();
        weighted_average(
            results,
            weights,
            pre_res);

        // el = sw();
        // el_total += el;

        // std::cout << "- weighted average: " << el * 1000.0 << " ms" << std::endl;

        // sw();
        m_corr_cpu->correction_from_covs(pre_res, dT);

        // std::cout << "don" << std::endl;
        // el = sw();
        // el_total += el;

        // std::cout << "- C -> dT: " << el * 1000.0 << " ms" << std::endl;
        // std::cout << "- total: " << el_total * 1000.0 << " ms" << std::endl;
    } else {
        std::cout << "0 sensors" << std::endl;
        // set identity
        for(size_t i=0; i<dT.size(); i++)
        {
            dT[i] = rm::Transform::Identity();
        }
    }
}
#endif // RMCL_CUDA

#ifdef RMCL_CUDA
void MICP::correct(
    const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm,
    CorrectionPreResults<rmagine::VRAM_CUDA>& pre_res,
    rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& dT)
{
    #ifdef RMCL_EMBREE
    rm::Memory<rm::Transform, rm::RAM> Tbm_ = Tbm;
    #endif // RMCL_EMBREE

    std::vector<CorrectionPreResults<rm::VRAM_CUDA> > results(m_sensors.size());
    float weight_sum = 0.0;
    std::vector<float> weights(m_sensors.size());

    for(auto& elem : results)
    {
        elem.ms.resize(Tbm.size());
        elem.ds.resize(Tbm.size());
        elem.Cs.resize(Tbm.size());
        elem.Ncorr.resize(Tbm.size());
    }


    size_t id = 0;
    for(auto elem : m_sensors)
    {
        if(elem.second->data_received_once)
        {
            CorrectionPreResults<rm::VRAM_CUDA>& res = results[id];

            #ifdef RMCL_EMBREE
            if(elem.second->backend == 0)
            {
                CorrectionPreResults<rm::RAM> res_;

                res_.ms.resize(Tbm.size());
                res_.ds.resize(Tbm.size());
                res_.Cs.resize(Tbm.size());
                res_.Ncorr.resize(Tbm.size());

                elem.second->computeCovs(Tbm_, res_);

                // download
                res.ms = res_.ms;
                res.ds = res_.ds;
                res.Cs = res_.Cs;
                res.Ncorr = res_.Ncorr;
            } else
            #endif // RMCL_EMBREE
            #ifdef RMCL_OPTIX
            if(elem.second->backend == 1) {
                std::cout << "Compute Covs" << std::endl;
                // use preuploaded poses as input
                elem.second->computeCovs(Tbm, res);
            } else
            #endif // RMCL_OPTIX
            {
                std::cout << "backend " << elem.second->backend << " unknown" << std::endl;
            }

            // dynamic weights
            float w = elem.second->corr_weight;
            weight_sum += w;
            weights[id] = w;
        } else {
            weights[id] = 0.0;
            std::cout << "WARNING: " << elem.second->name << " still not received data" << std::endl;
        }

        id++;
    }


    if(results.size() > 0)
    {
        // normalize weights
        // sw();
        for(size_t i=0; i<weights.size(); i++)
        {
            weights[i] /= weight_sum;
        }

        
        if(pre_res.ms.size() < Tbm.size())
        {
            pre_res.ms.resize(Tbm.size());
            pre_res.ds.resize(Tbm.size());
            pre_res.Cs.resize(Tbm.size());
            pre_res.Ncorr.resize(Tbm.size());
        }
        
        // el = sw();
        // el_total += el;
        // std::cout << "- merging preprocessing: " << el * 1000.0 << " ms" << std::endl;

        // sw();
        weighted_average(
            results,
            weights,
            pre_res);

        // TODO: adjust parameters if enabled

        // el = sw();
        // el_total += el;

        // std::cout << "- weighted average: " << el * 1000.0 << " ms" << std::endl;

        // sw();
        m_corr_gpu->correction_from_covs(pre_res, dT);
        // el = sw();
        // el_total += el;

        // std::cout << "- C -> dT: " << el * 1000.0 << " ms" << std::endl;
    } else {
        // std::cout << "0 sensors" << std::endl;
        // set identity
        for(size_t i=0; i<dT.size(); i++)
        {
            dT[i] = rm::Transform::Identity();
        }
    }
}
#endif // RMCL_CUDA

void MICP::correct(
    const rm::MemoryView<rm::Transform, rm::RAM>& Tbm,
    CorrectionPreResults<rm::RAM>& pre_res,
    rm::MemoryView<rm::Transform, rm::RAM>& dT)
{
    // rm::StopWatch sw;
    // double el;
    // double el_total = 0.0;

    // std::cout << "-----------------" << std::endl;
    
    // sw();
    #ifdef RMCL_OPTIX
    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbm_ = Tbm;
    #endif // RMCL_OPTIX

    std::vector<CorrectionPreResults<rm::RAM> > results(m_sensors.size());
    float weight_sum = 0.0;
    std::vector<float> weights(m_sensors.size());

    for(auto& elem : results)
    {
        elem.ms.resize(Tbm.size());
        elem.ds.resize(Tbm.size());
        elem.Cs.resize(Tbm.size());
        elem.Ncorr.resize(Tbm.size());
    }

    // el = sw();
    // el_total += el;
    
    // std::cout << "- preprocessing: " << el * 1000.0 << " ms" << std::endl;

    // sw();
    size_t id = 0;
    for(auto elem : m_sensors)
    {
        if(elem.second->data_received_once)
        {
            CorrectionPreResults<rm::RAM>& res = results[id];

            #ifdef RMCL_EMBREE
            if(elem.second->backend == 0)
            {
                elem.second->computeCovs(Tbm, res);
            } else
            #endif // RMCL_EMBREE
            #ifdef RMCL_OPTIX
            if(elem.second->backend == 1) {

                CorrectionPreResults<rm::VRAM_CUDA> res_;
                res_.ms.resize(Tbm.size());
                res_.ds.resize(Tbm.size());
                res_.Cs.resize(Tbm.size());
                res_.Ncorr.resize(Tbm.size());
                
                // use preuploaded poses as input
                elem.second->computeCovs(Tbm_, res_);

                // download
                res.ms = res_.ms;
                res.ds = res_.ds;
                res.Cs = res_.Cs;
                res.Ncorr = res_.Ncorr;
            } else
            #endif // RMCL_OPTIX
            {
                std::cout << elem.second->name << " - backend " << elem.second->backend << " unknown" << std::endl;
            }

            // dynamic weights
            float w = elem.second->corr_weight;
            weight_sum += w;
            weights[id] = w;
        } else {
            weights[id] = 0.0;
            std::cout << "WARNING: " << elem.second->name << " still not received data" << std::endl;
        }

        id++;
    }
    // el = sw();
    // el_total += el;
    // std::cout << "- computing covs (" << results.size() << " sensors): " << el * 1000.0 << " ms" << std::endl;

    if(results.size() > 0)
    {
        // normalize weights
        // sw();
        for(size_t i=0; i<weights.size(); i++)
        {
            weights[i] /= weight_sum;
        }

        if(pre_res.ms.size() < Tbm.size())
        {
            pre_res.ms.resize(Tbm.size());
            pre_res.ds.resize(Tbm.size());
            pre_res.Cs.resize(Tbm.size());
            pre_res.Ncorr.resize(Tbm.size());
        }
        
        // el = sw();
        // el_total += el;
        // std::cout << "- merging preprocessing: " << el * 1000.0 << " ms" << std::endl;

        // sw();
        weighted_average(
            results,
            weights,
            pre_res);

        // TODO: adjust parameters if enabled

        // el = sw();
        // el_total += el;

        // std::cout << "- weighted average: " << el * 1000.0 << " ms" << std::endl;

        // sw();
        m_corr_cpu->correction_from_covs(pre_res, dT);
        // el = sw();
        // el_total += el;

        // std::cout << "- C -> dT: " << el * 1000.0 << " ms" << std::endl;
    } else {
        // std::cout << "0 sensors" << std::endl;
        // set identity
        for(size_t i=0; i<dT.size(); i++)
        {
            dT[i] = rm::Transform::Identity();
        }
    }
    // std::cout << "- total: " << el_total * 1000.0 << " ms" << std::endl;
}

bool MICP::checkTF(bool prints)
{
    std::cout << std::endl;
    std::cout << "-------------------------" << std::endl;
    std::cout << "     --- FRAMES ---      " << std::endl;
    std::cout << "-------------------------" << std::endl;
    std::cout << "- base:\t\t\t" << TC_FRAME << m_base_frame << TC_END << std::endl;
    
    if(m_use_odom_frame)
    {
        std::cout << "- odom:\t\t\t" << TC_FRAME << m_odom_frame << TC_END << std::endl;

        // check connection to base
        bool odom_to_base_available = false;
        
        int num_tries = 10;

        rclcpp::ExecutorOptions opts;
        rclcpp::executors::SingleThreadedExecutor executor(opts);
        executor.add_node(m_nh);
        rclcpp::Rate r(20);

        while(rclcpp::ok() && num_tries > 0 && !odom_to_base_available )
        {
            try {
                auto T = m_tf_buffer->lookupTransform(m_odom_frame, m_base_frame, tf2::TimePointZero);
                odom_to_base_available = true;
            } catch (tf2::TransformException &ex) {
                odom_to_base_available = false;
            }

            r.sleep();
            executor.spin_once();
            num_tries--;
        }

        if(odom_to_base_available)
        {
            std::cout << "  - base -> odom:\t" << TC_GREEN << "yes" << TC_END << std::endl;
        } else {
            std::cout << "  - base -> odom:\t" << TC_RED << "no" << TC_END << std::endl;
        }
    } else {
        std::cout << "- odom:\t\t\tdisabled" << std::endl; 
    }

    std::cout << "- map:\t\t\t" << TC_FRAME << m_map_frame << TC_END << std::endl;

    std::cout << "Estimating: " << TC_FRAME << m_base_frame << TC_END << " -> " << TC_FRAME << m_map_frame << TC_END << std::endl;
    if(m_use_odom_frame)
    {
        std::cout << "Providing: " << TC_FRAME << m_odom_frame << TC_END << " -> " << TC_FRAME << m_map_frame << TC_END << std::endl;
    } else {
        std::cout << "Providing: " << TC_FRAME << m_base_frame << TC_END << " -> " << TC_FRAME << m_map_frame << TC_END << std::endl;
    }

    return true;
}


void MICP::checkTopic(
    TopicInfo& info, 
    rclcpp::Duration timeout)
{
    info.data = false;
    info.frame = "";

    using ChronoDuration = std::chrono::duration<int64_t, std::milli>;

    if(info.msg == "sensor_msgs/msg/PointCloud2")
    {
        sensor_msgs::msg::PointCloud2 msg;
        if(rclcpp::wait_for_message(msg, m_nh, info.name, timeout.to_chrono<ChronoDuration>()))
        {
            info.data = true;
            info.frame = msg.header.frame_id;
        }
    } else if(info.msg == "sensor_msgs/msg/PointCloud") {
        sensor_msgs::msg::PointCloud msg;
        if(rclcpp::wait_for_message(msg, m_nh, info.name, timeout.to_chrono<ChronoDuration>()))
        {
            info.data = true;
            info.frame = msg.header.frame_id;
        }
    } else if(info.msg == "sensor_msgs/msg/LaserScan") {
        sensor_msgs::msg::LaserScan msg;
        if(rclcpp::wait_for_message(msg, m_nh, info.name, timeout.to_chrono<ChronoDuration>()))
        {
            info.data = true;
            info.frame = msg.header.frame_id;
        }
    } else if(info.msg == "sensor_msgs/msg/Image") {
        sensor_msgs::msg::Image msg;
        if(rclcpp::wait_for_message(msg, m_nh, info.name, timeout.to_chrono<ChronoDuration>()))
        {
            info.data = true;
            info.frame = msg.header.frame_id;
        }
    } else if(info.msg == "sensor_msgs/msg/CameraInfo") {
        sensor_msgs::msg::CameraInfo msg;
        if(rclcpp::wait_for_message(msg, m_nh, info.name, timeout.to_chrono<ChronoDuration>()))
        {
            info.data = true;
            info.frame = msg.header.frame_id;
        }
    } else if(info.msg == "rmcl_msgs/msg/ScanStamped") {
        rmcl_msgs::msg::ScanStamped msg;
        if(rclcpp::wait_for_message(msg, m_nh, info.name, timeout.to_chrono<ChronoDuration>()))
        {
            info.data = true;
            info.frame = msg.header.frame_id;
        }
    } else if(info.msg == "rmcl_msgs/msg/DepthStamped") {
        rmcl_msgs::msg::DepthStamped msg;
        if(rclcpp::wait_for_message(msg, m_nh, info.name, timeout.to_chrono<ChronoDuration>()))
        {
            info.data = true;
            info.frame = msg.header.frame_id;
        }
    } else if(info.msg == "rmcl_msgs/msg/O1DnStamped") {
        rmcl_msgs::msg::O1DnStamped msg;
        if(rclcpp::wait_for_message(msg, m_nh, info.name, timeout.to_chrono<ChronoDuration>()))
        {
            info.data = true;
            info.frame = msg.header.frame_id;
        }
    } else if(info.msg == "rmcl_msgs/msg/OnDnStamped") {
        rmcl_msgs::msg::OnDnStamped msg;
        if(rclcpp::wait_for_message(msg, m_nh, info.name, timeout.to_chrono<ChronoDuration>()))
        {
            info.data = true;
            info.frame = msg.header.frame_id;
        }
    } else {
        // unknown type
        return;
    }
}



} // namespace rmcl