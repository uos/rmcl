#include "rmcl/correction/MICP.hpp"
#include <ros/master.h>
#include <vector>


#include <geometry_msgs/TransformStamped.h>

#include <rmcl/util/conversions.h>

#include <rmcl/math/math.h>

#ifdef RMCL_CUDA
#include <rmcl/math/math.cuh>
#include <rmagine/math/math.cuh>
#endif // RMCL_CUDA

#include <rmagine/util/StopWatch.hpp>


namespace rm = rmagine;

namespace rmcl
{

MICP::MICP()
:m_nh(new ros::NodeHandle())
,m_nh_p(new ros::NodeHandle("~"))
,m_tf_buffer(new tf2_ros::Buffer)
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

    std::string combining_unit_str;
    m_nh_p->param<std::string>("micp/combining_unit", combining_unit_str, "cpu");

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
    m_nh_p->param<std::string>("base_frame", m_base_frame, "base_link");
    m_nh_p->param<std::string>("map_frame", m_map_frame, "map");

    m_use_odom_frame = m_nh_p->getParam("odom_frame", m_odom_frame);

    // check frames


    if(!m_nh_p->getParam("map_file", m_map_filename))
    {
        ROS_ERROR("User must provide ~map_file");
    }

    checkTF(true);

    loadMap(m_map_filename);

    XmlRpc::XmlRpcValue sensors_xml;
    if(m_nh_p->getParam("sensors", sensors_xml))
    {
        std::cout << std::endl;
        std::cout << "-------------------------" << std::endl;
        std::cout << "     --- SENSORS ---     " << std::endl;
        std::cout << "-------------------------" << std::endl;
        for(auto sensor_xml : sensors_xml)
        {
            loadSensor(sensor_xml.first, sensor_xml.second);
        }
    } else {
        std::cout << "ERROR: NO SENSORS" << std::endl;
    }

    std::cout << "MICP load params - done. Valid Sensors: " << m_sensors.size() << std::endl;
    
}

std::unordered_map<std::string, std::string> get_topic_type_map()
{
    std::unordered_map<std::string, std::string> ret;
    std::vector<ros::master::TopicInfo> topic_infos;
    ros::master::getTopics(topic_infos);
    for(auto topic_info : topic_infos)
    {
        ret[topic_info.name] = topic_info.datatype;
    }

    return ret;
}

bool MICP::loadSensor(std::string sensor_name, XmlRpc::XmlRpcValue sensor_params)
{
    MICPRangeSensorPtr sensor = std::make_shared<MICPRangeSensor>();

    bool loading_error = false;

    // std::string sensor_name = sensor_xml.first;
    std::string sensor_type;
    bool        sensor_type_found = false;
    // std::string topic_msg;
    // bool        topic_valid = false;

    sensor->name = sensor_name;
    sensor->base_frame = m_base_frame;

    if(sensor_params.hasMember("type"))
    {
        sensor_type = (std::string)sensor_params["type"];
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

    if(sensor_params.hasMember("frame"))
    {
        sensor->frame = (std::string)sensor_params["frame"];
    }

    std::cout << "- " << TC_SENSOR << sensor_name << TC_END << std::endl;

    // load data or connect to a data topic
    if(sensor_params.hasMember("topic"))
    {
        std::cout << "  - data:\t\tTopic" << std::endl;
        // std::cout << "has topic" << std::endl;
        std::string topic_name = (std::string)sensor_params["topic"];
        if(topic_name[0] != '/')
        {
            topic_name = m_nh->getNamespace() + topic_name;
        }

        sensor->data_topic.name = topic_name;
        std::cout << "    - topic:\t\t" << TC_TOPIC << sensor->data_topic.name << TC_END << std::endl;

        if(sensor_params.hasMember("topic_type"))
        {
            sensor->data_topic.msg = (std::string)sensor_params["topic_type"];
        }

        std::unordered_map<std::string, std::string> topic_map
            = get_topic_type_map();
        if(topic_map.find(sensor->data_topic.name) != topic_map.end())
        {
            auto topic_type = topic_map[sensor->data_topic.name];

            if(sensor->data_topic.msg != "")
            {
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
            if(sensor->data_topic.msg == "sensor_msgs/LaserScan" 
                || sensor->data_topic.msg == "rmcl_msgs/ScanStamped")
            {
                sensor_type = "spherical";
                sensor_type_found = true;
                sensor->type = 0;
            } else if(sensor->data_topic.msg == "sensor_msgs/Image" 
                || sensor->data_topic.msg == "rmcl_msgs/DepthStamped") 
            {
                // is image always pinhole? counterexample: cylindrical image e.g. panorama
                sensor_type = "pinhole";
                sensor_type_found = true;
                sensor->type = 1;
            }

            std::cout << "    - msg:\t\t" << TC_MSG << sensor->data_topic.msg << TC_END << std::endl;
            // check if topic is valid
            checkTopic(sensor->data_topic, ros::Duration(5.0));

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
    } else if(sensor_params.hasMember("ranges")) {
        // std::cout << "  - topic:\t\t" << TC_RED << "not found" << TC_END << std::endl;
        // check if there is data in the parameters instead

        std::cout << "  - data:\t\tParams" << std::endl;
        // fixed ranges are in params!
        auto ranges_xml = sensor_params["ranges"];

        if(ranges_xml.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            sensor->ranges.resize(ranges_xml.size());
            for(size_t i=0; i<ranges_xml.size(); i++)
            {
                sensor->ranges[i] = (double)ranges_xml[i];
            }
            #ifdef RMCL_CUDA
            sensor->ranges_gpu = sensor->ranges;
            #endif // RMCL_CUDA
            sensor->data_received_once = true;
        } else {
            std::cout << "error: 'data/ranges' param is not an array" << std::endl;
        }
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
    if(sensor_params.hasMember("model")) /// PARAMS
    {
        std::cout << "  - model:\t\tParams" << std::endl;

        auto model_xml = sensor_params["model"];

        if(sensor_type == "spherical") {
            rm::SphericalModel model;

            // fill
            model.theta.min = (double)model_xml["theta_min"];
            model.theta.inc = (double)model_xml["theta_inc"];
            model.theta.size = (int)model_xml["theta_N"];

            model.phi.min = (double)model_xml["phi_min"];
            model.phi.inc = (double)model_xml["phi_inc"];
            model.phi.size = (int)model_xml["phi_N"];

            model.range.min = (double)model_xml["range_min"];
            model.range.max = (double)model_xml["range_max"];

            sensor->model = model;
            model_loaded = true;
        } else if(sensor_type == "pinhole") {
            rm::PinholeModel model;

            model.width = (int)model_xml["width"];
            model.height = (int)model_xml["height"];

            model.f[0] = (double)(model_xml["f"][0]);
            model.f[1] = (double)(model_xml["f"][1]);
            model.c[0] = (double)(model_xml["c"][0]);
            model.c[1] = (double)(model_xml["c"][1]);

            model.range.min = (double)model_xml["range_min"];
            model.range.max = (double)model_xml["range_max"];

            sensor->model = model;
            model_loaded = true;
        } else if(sensor_type == "o1dn") {
            rm::O1DnModel model;
            bool model_loading_error = false;

            // could also be loaded by data using rmcl_msgs
            model.range.min = (double)model_xml["range_min"];
            model.range.max = (double)model_xml["range_max"];

            model.orig = {0.0, 0.0, 0.0};
            if(model_xml.hasMember("orig"))
            {
                auto orig_xml = model_xml["orig"];
                if(orig_xml.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {
                    model.orig.x = (double)orig_xml[0];
                    model.orig.y = (double)orig_xml[1];
                    model.orig.z = (double)orig_xml[2];
                } else {
                    std::cout << "reading o1dn model error: orig muste be a list of three numbers (x,y,z)" << std::endl;
                    model_loading_error = true;
                }
            }

            if(model_xml.hasMember("width"))
            {
                model.width = (int)model_xml["width"];
            } else {
                // loading actual width from sensor data
                model.width = 0;
            }

            if(model_xml.hasMember("height"))
            {
                model.width = (int)model_xml["height"];
            } else {
                // loading actual height from sensor data
                model.width = 0;
            }
            
            if(model_xml.hasMember("dirs"))
            {
                auto dirs_xml = model_xml["dirs"];
                if(dirs_xml.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {   
                    model.dirs.resize(dirs_xml.size());
                    for(size_t i=0; i<dirs_xml.size(); i++)
                    {
                        auto dir_xml = dirs_xml[i];
                        if(dir_xml.getType() == XmlRpc::XmlRpcValue::TypeArray 
                            && dir_xml.size() == 3) 
                        {
                            rm::Vector dir;
                            dir.x = (double)dir_xml[0];
                            dir.y = (double)dir_xml[1];
                            dir.z = (double)dir_xml[2];
                            model.dirs[i] = dir;
                        } else {
                            // better error message
                            std::cout << "ERROR: malformed vector in parameters (dirs, " << i << ")" << std::endl;
                        }
                    }
                } 
                else 
                {
                    model_loading_error = true;
                    std::cout << "o1dn model - dirs: is no array" << std::endl;
                }
            } else {
                model.dirs.resize(0);
            }
            
            sensor->model = model;
            model_loaded = !model_loading_error;
        } else if(sensor_type == "ondn") {
            rm::OnDnModel model;

            bool model_loading_error = false;

            model.range.min = (double)model_xml["range_min"];
            model.range.max = (double)model_xml["range_max"];
            
            if(model_xml.hasMember("origs"))
            {
                auto origs_xml = model_xml["origs"];
                if(origs_xml.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {
                    model.origs.resize(origs_xml.size());

                    for(size_t i=0; i<origs_xml.size(); i++)
                    {
                        auto orig_xml = origs_xml[i];
                        if(orig_xml.getType() == XmlRpc::XmlRpcValue::TypeArray
                            && orig_xml.size() == 3) 
                        {
                            rm::Vector orig;
                            orig.x = (double)orig_xml[0];
                            orig.y = (double)orig_xml[1];
                            orig.z = (double)orig_xml[2];
                            model.origs[i] = orig;
                        } else {
                            // better error message
                            std::cout << "ERROR: malformed vector in parameters (origs, " << i << ")" << std::endl;
                        }
                    }
                } else {
                    model_loading_error = true;
                    std::cout << "ondn model - origs: is no array" << std::endl;
                }
            } else {
                model.origs.resize(0);
            }

            if(model_xml.hasMember("dirs"))
            {
                auto dirs_xml = model_xml["dirs"];
                if(dirs_xml.getType() == XmlRpc::XmlRpcValue::TypeArray)
                {   
                    model.dirs.resize(dirs_xml.size());
                    for(size_t i=0; i<dirs_xml.size(); i++)
                    {
                        auto dir_xml = dirs_xml[i];
                        if(dir_xml.getType() == XmlRpc::XmlRpcValue::TypeArray 
                            && dir_xml.size() == 3)
                        {
                            rm::Vector dir;
                            dir.x = (double)dir_xml[0];
                            dir.y = (double)dir_xml[1];
                            dir.z = (double)dir_xml[2];
                            model.dirs[i] = dir;
                        } else {
                            // better error message
                            std::cout << "ERROR: malformed vector in parameters (dirs, " << i << ")" << std::endl;
                        }
                    }
                } 
                else 
                {
                    model_loading_error = true;
                    std::cout << "ondn model - dirs: is no array" << std::endl;
                }
            } else {
                model.dirs.resize(0);
            }

            if(model_xml.hasMember("width"))
            {
                model.width = (int)model_xml["width"];
            } else {
                // loading actual width from sensor data
                model.width = 0;
            }

            if(model_xml.hasMember("height"))
            {
                model.width = (int)model_xml["height"];
            } else {
                // loading actual height from sensor data
                model.width = 0;
            }

            sensor->model = model;
            model_loaded = !model_loading_error;
        } else {
            // ERROR
            std::cout << "Model type '" << sensor->type << "' not supported." << std::endl;
            loading_error = true;
        }

    } else if(sensor_params.hasMember("model_topic")) { /// TOPIC

        std::cout << "  - model:\t\tTopic" << std::endl;

        sensor->has_info_topic = true;

        std::string info_topic_name = sensor_params["model_topic"];
        if(info_topic_name[0] != '/')
        {
            info_topic_name = m_nh->getNamespace() + info_topic_name;
        }
        sensor->info_topic.name = info_topic_name;

        std::cout << "    - topic:\t\t" << TC_TOPIC << sensor->info_topic.name  << TC_END << std::endl;

        if(sensor_params.hasMember("topic_type"))
        {
            sensor->data_topic.msg = (std::string)sensor_params["topic_type"];
        }

        std::unordered_map<std::string, std::string> topic_map
            = get_topic_type_map();
        if(topic_map.find(sensor->info_topic.name) != topic_map.end())
        {
            std::string topic_type = topic_map[sensor->info_topic.name];
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
            if(sensor->info_topic.msg == "rmcl_msgs/ScanInfo")
            {
                auto msg = ros::topic::waitForMessage<rmcl_msgs::ScanInfo>(sensor->info_topic.name, *m_nh, ros::Duration(3.0));

                if(msg)
                {   
                    rm::SphericalModel model;
                    convert(*msg, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "ERROR: Could not receive initial pinhole model!" << std::endl;
                }
            }
        } else if(sensor_type == "pinhole") {
            
            if(sensor->info_topic.msg == "sensor_msgs/CameraInfo")
            {
                // std::cout << "Waiting for message on topic: " << sensor->info_topic.name << std::endl;
                auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(sensor->info_topic.name, *m_nh, ros::Duration(3.0));
            
                if(msg)
                {
                    if(msg->header.frame_id != sensor->frame)
                    {
                        std::cout << "WARNING: Image and CameraInfo are not in the same frame" << std::endl;
                    }
                    
                    rm::PinholeModel model;
                    convert(*msg, model);
                    

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

            if(sensor->info_topic.msg == "rmcl_msgs/DepthInfo")
            {
                auto msg = ros::topic::waitForMessage<rmcl_msgs::DepthInfo>(sensor->info_topic.name, *m_nh, ros::Duration(3.0));

                if(msg)
                {   
                    rm::PinholeModel model;
                    convert(*msg, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "ERROR: Could not receive initial pinhole model!" << std::endl;
                }
            }


        } else if(sensor_type == "o1dn") {
            
            if(sensor->info_topic.msg == "rmcl_msgs/O1DnInfo")
            {
                auto msg = ros::topic::waitForMessage<rmcl_msgs::O1DnInfo>(sensor->info_topic.name, *m_nh, ros::Duration(3.0));

                if(msg)
                {   
                    rm::O1DnModel model;
                    convert(*msg, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "ERROR: Could not receive initial o1dn model!" << std::endl;
                }
            }

        } else if(sensor_type == "ondn") {
            if(sensor->info_topic.msg == "rmcl_msgs/OnDnInfo")
            {
                auto msg = ros::topic::waitForMessage<rmcl_msgs::OnDnInfo>(sensor->info_topic.name, *m_nh, ros::Duration(3.0));

                if(msg)
                {   
                    rm::OnDnModel model;
                    convert(*msg, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "ERROR: Could not receive initial ondn model!" << std::endl;
                }
            }
        }

    } else { /// DATA
        std::cout << "  - model:\t\tData" << std::endl;

        if(sensor_type == "spherical")
        {
            if(!model_loaded && sensor->data_topic.msg == "sensor_msgs/LaserScan")
            {
                auto msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(sensor->data_topic.name, *m_nh, ros::Duration(3.0));

                if(msg)
                {
                    rm::SphericalModel model;
                    convert(*msg, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "error: get sensor_msgs/LaserScan to init model" << std::endl;
                }
            }
            
            if(!model_loaded && sensor->data_topic.msg == "rmcl_msgs/ScanStamped")
            {
                auto msg = ros::topic::waitForMessage<rmcl_msgs::ScanStamped>(sensor->data_topic.name, *m_nh, ros::Duration(3.0));

                if(msg)
                {
                    rm::SphericalModel model;
                    convert(msg->scan.info, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "error: get rmcl_msgs/ScanStamped to init model" << std::endl;
                }
            }
        } else if(sensor_type == "pinhole") {

            if(!model_loaded && sensor->data_topic.msg == "rmcl_msgs/DepthStamped")
            {
                auto msg = ros::topic::waitForMessage<rmcl_msgs::DepthStamped>(sensor->data_topic.name, *m_nh, ros::Duration(3.0));

                if(msg)
                {
                    rm::PinholeModel model;
                    convert(msg->depth.info, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "error: get rmcl_msgs/DepthStamped to init model" << std::endl;
                }
            }

        } else if(sensor_type == "o1dn") {

            if(!model_loaded && sensor->data_topic.msg == "rmcl_msgs/O1DnStamped")
            {
                auto msg = ros::topic::waitForMessage<rmcl_msgs::O1DnStamped>(sensor->data_topic.name, *m_nh, ros::Duration(3.0));

                if(msg)
                {
                    rm::O1DnModel model;
                    convert(msg->o1dn.info, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "error: get rmcl_msgs/DepthStamped to init model" << std::endl;
                }
            }

        } else if(sensor_type == "ondn") {

            if(!model_loaded && sensor->data_topic.msg == "rmcl_msgs/OnDnStamped")
            {
                auto msg = ros::topic::waitForMessage<rmcl_msgs::OnDnStamped>(sensor->data_topic.name, *m_nh, ros::Duration(3.0));

                if(msg)
                {
                    rm::OnDnModel model;
                    convert(msg->ondn.info, model);
                    sensor->model = model;
                    model_loaded = true;
                } else {
                    std::cout << "error: get rmcl_msgs/DepthStamped to init model" << std::endl;
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
    if(sensor_params.hasMember("micp"))
    {
        std::cout << "  - micp:" << std::endl;
        auto micp_xml = sensor_params["micp"];

        if(micp_xml.hasMember("backend"))
        {
            std::string backend_name = micp_xml["backend"];
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

        if(micp_xml.hasMember("max_dist"))
        {
            auto max_dist_xml = micp_xml["max_dist"];
            if(max_dist_xml.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                sensor->corr_params_init.max_distance = (double)max_dist_xml;
            } else if(max_dist_xml.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                sensor->corr_params_init.max_distance = (int)max_dist_xml;
            } else {
                // ERROR
                std::cout << "Could not load 'micp/max_dist'" << std::endl;
            }
        }
        sensor->corr_params = sensor->corr_params_init;


        if(micp_xml.hasMember("adaptive_max_dist_min"))
        {
            auto ada_max_dist_min_xml = micp_xml["adaptive_max_dist_min"];
            if(ada_max_dist_min_xml.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                sensor->adaptive_max_dist_min = (double)ada_max_dist_min_xml;
            } else 
            if(ada_max_dist_min_xml.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                sensor->adaptive_max_dist_min = (int)ada_max_dist_min_xml;
            } else {
                // ERROR
                std::cout << "Could not load 'micp/adaptive_max_dist_min'" << std::endl;
            }
        }

        if(micp_xml.hasMember("weight"))
        {
            auto weight_xml = micp_xml["weight"];

            if(weight_xml.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                sensor->corr_weight = (double)micp_xml["weight"];
            } else if(weight_xml.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                sensor->corr_weight = (int)micp_xml["weight"];
            } else {
                // ERROR
                std::cout << "Could not load 'micp/weight'" << std::endl;
            }
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


    ////////////////////////
    //// 2. POSTPROCESS ////
    ////////////////////////

    // postcheck
    // check if optical: _optical suffix
    // std::cout << "Searching for '_optical' in " << sensor->data_topic.frame << std::endl;
    
    // connect sensor to ROS
    sensor->nh = m_nh;
    sensor->nh_p = m_nh_p;
    sensor->tf_buffer = m_tf_buffer; 

    // load additional params: duplicated from above
    sensor->fetchMICPParams();
    // connect to sensor topics
    sensor->connect();


    #ifdef RMCL_EMBREE
    if(sensor->type == 0) // spherical
    {
        sensor->corr_sphere_embree = std::make_shared<SphereCorrectorEmbree>(m_map_embree);
    } else if(sensor->type == 1) {
        sensor->corr_pinhole_embree = std::make_shared<PinholeCorrectorEmbree>(m_map_embree);;
    } else if(sensor->type == 2) {
        sensor->corr_o1dn_embree = std::make_shared<O1DnCorrectorEmbree>(m_map_embree);
    } else if(sensor->type == 3) {
        sensor->corr_ondn_embree = std::make_shared<OnDnCorrectorEmbree>(m_map_embree);
    }
    #endif // RMCL_EMBREE

    #ifdef RMCL_OPTIX
    if(sensor->type == 0) // spherical
    {
        sensor->corr_sphere_optix = std::make_shared<SphereCorrectorOptix>(m_map_optix);
    } else if(sensor->type == 1) {
        sensor->corr_pinhole_optix = std::make_shared<PinholeCorrectorOptix>(m_map_optix);
    } else if(sensor->type == 2) {
        sensor->corr_o1dn_optix = std::make_shared<O1DnCorrectorOptix>(m_map_optix);
    } else if(sensor->type == 3) {
        sensor->corr_ondn_optix = std::make_shared<OnDnCorrectorOptix>(m_map_optix);
    }
    #endif // RMCL_OPTIX

    
    sensor->fetchTF();
    sensor->updateCorrectors();

    // add sensor to class
    m_sensors[sensor->name] = sensor;
    
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
        ros::Rate r(20);

        while(ros::ok() && num_tries > 0 && !odom_to_base_available )
        {
            try {
                auto T = m_tf_buffer->lookupTransform(m_odom_frame, m_base_frame, ros::Time(0));
                odom_to_base_available = true;
            } catch (tf2::TransformException &ex) {
                odom_to_base_available = false;
            }

            r.sleep();
            ros::spinOnce();
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
    ros::Duration timeout)
{
    info.data = false;
    info.frame = "";

    ros::Time curr = ros::Time::now();

    // returns zero on startup
    while(curr == ros::Time(0))
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
        curr = ros::Time::now();
    }

    ros::Time end = curr + timeout;
    // std::cout << curr.toSec() << " -> " << end.toSec() << std::endl;

    ros::Duration timeout_inner(0.2);

    while(ros::ok() && curr < end && !info.data)
    {
        if(info.msg == "sensor_msgs/PointCloud2")
        {
            auto msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(info.name, *m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "sensor_msgs/PointCloud") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::PointCloud>(info.name, *m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "sensor_msgs/LaserScan") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(info.name, *m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "sensor_msgs/Image") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::Image>(info.name, *m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "sensor_msgs/CameraInfo") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(info.name, *m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "rmcl_msgs/ScanStamped") {
            auto msg = ros::topic::waitForMessage<rmcl_msgs::ScanStamped>(info.name, *m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "rmcl_msgs/DepthStamped") {
            auto msg = ros::topic::waitForMessage<rmcl_msgs::DepthStamped>(info.name, *m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "rmcl_msgs/O1DnStamped") {
            auto msg = ros::topic::waitForMessage<rmcl_msgs::O1DnStamped>(info.name, *m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "rmcl_msgs/OnDnStamped") {
            auto msg = ros::topic::waitForMessage<rmcl_msgs::OnDnStamped>(info.name, *m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else {
            // unknown type
            return;
        }

        ros::spinOnce();
        timeout_inner.sleep();
        curr = ros::Time::now();
    }
}



} // namespace rmcl
