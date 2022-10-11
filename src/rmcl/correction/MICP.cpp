#include "rmcl/correction/MICP.hpp"
#include <ros/master.h>
#include <vector>


#include <geometry_msgs/TransformStamped.h>

#include <image_transport/image_transport.h>

#include <rmcl/util/conversions.h>

#include <rmcl/math/math.h>

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

    std::cout << "Available backends:" << std::endl;

    #ifdef RMCL_EMBREE
    std::cout << "- " << TC_BACKENDS << "CPU (Embree)" << TC_END << std::endl;
    #endif

    #ifdef RMCL_OPTIX
    std::cout << "- " << TC_BACKENDS << "GPU (Optix)" << TC_END << std::endl;
    #endif
}

MICP::~MICP()
{
    std::cout << "MICP cleanup" << std::endl;
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

template<typename T>
inline T get_as(const XmlRpc::XmlRpcValue& v)
{
    if(v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        double tmp = v;
        return static_cast<T>(tmp);
    } else if(v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        int tmp = v;
        return static_cast<T>(tmp);
    }
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

        std::vector<ros::master::TopicInfo> topic_infos;
        ros::master::getTopics(topic_infos);
        for(auto topic_info : topic_infos)
        {
            if(topic_info.name == sensor->data_topic.name)
            {
                sensor->data_topic.msg = topic_info.datatype;
                break;
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
            sensor->ranges_gpu = sensor->ranges;
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

            model.width = (int)model_xml["width"];
            model.height = (int)model_xml["height"];

            model.range.min = (double)model_xml["range_min"];
            model.range.max = (double)model_xml["range_max"];


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

            sensor->model = model;
            model_loaded = !model_loading_error;
        } else if(sensor_type == "ondn") {
            rm::OnDnModel model;

            bool model_loading_error = false;

            model.width = (int)model_xml["width"];
            model.height = (int)model_xml["height"];
            model.range.min = (double)model_xml["range_min"];
            model.range.max = (double)model_xml["range_max"];
            
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

        std::vector<ros::master::TopicInfo> topic_infos;
        ros::master::getTopics(topic_infos);
        for(auto topic_info : topic_infos)
        {
            if(topic_info.name == info_topic_name)
            {
                sensor->info_topic.msg = topic_info.datatype;
                break;
            }
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
                sensor->corr_params.max_distance = (double)max_dist_xml;
            } else if(max_dist_xml.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                sensor->corr_params.max_distance = (int)max_dist_xml;
            } else {
                // ERROR
                std::cout << "Could not load 'micp/max_dist'" << std::endl;
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
    sensor->tf_buffer = m_tf_buffer; 
    sensor->connect();

    // add sensor to class
    m_sensors[sensor->name] = sensor;

    #ifdef RMCL_EMBREE
    if(sensor->type == 0) // spherical
    {
        SphereCorrectorEmbreePtr corr = std::make_shared<SphereCorrectorEmbree>(m_map_embree);
        corr->setParams(sensor->corr_params);
        corr->setModel(std::get<0>(sensor->model));
        sensor->corr_sphere_embree = corr;
    } else if(sensor->type == 1) {
        PinholeCorrectorEmbreePtr corr = std::make_shared<PinholeCorrectorEmbree>(m_map_embree);
        corr->setParams(sensor->corr_params);
        corr->setModel(std::get<1>(sensor->model));
        corr->setOptical(sensor->optical_coordinates);
        sensor->corr_pinhole_embree = corr;
    } else if(sensor->type == 2) {
        O1DnCorrectorEmbreePtr corr = std::make_shared<O1DnCorrectorEmbree>(m_map_embree);
        corr->setParams(sensor->corr_params);
        corr->setModel(std::get<2>(sensor->model));
        sensor->corr_o1dn_embree = corr;
    } else if(sensor->type == 3) {
        OnDnCorrectorEmbreePtr corr = std::make_shared<OnDnCorrectorEmbree>(m_map_embree);
        corr->setParams(sensor->corr_params);
        corr->setModel(std::get<3>(sensor->model));
        sensor->corr_ondn_embree = corr;
    }
    #endif // RMCL_EMBREE

    #ifdef RMCL_OPTIX
    if(sensor->type == 0) // spherical
    {
        SphereCorrectorOptixPtr corr = std::make_shared<SphereCorrectorOptix>(m_map_optix);
        corr->setParams(sensor->corr_params);
        corr->setModel(std::get<0>(sensor->model));
        sensor->corr_sphere_optix = corr;
    } else if(sensor->type == 1) {
        PinholeCorrectorOptixPtr corr = std::make_shared<PinholeCorrectorOptix>(m_map_optix);
        corr->setParams(sensor->corr_params);
        corr->setModel(std::get<1>(sensor->model));
        corr->setOptical(sensor->optical_coordinates);
        sensor->corr_pinhole_optix = corr;
    } else if(sensor->type == 2) {
        O1DnCorrectorOptixPtr corr = std::make_shared<O1DnCorrectorOptix>(m_map_optix);
        corr->setParams(sensor->corr_params);
        corr->setModel(std::get<2>(sensor->model));
        sensor->corr_o1dn_optix = corr;
    } else if(sensor->type == 3) {
        OnDnCorrectorOptixPtr corr = std::make_shared<OnDnCorrectorOptix>(m_map_optix);
        corr->setParams(sensor->corr_params);
        corr->setModel(std::get<3>(sensor->model));
        sensor->corr_ondn_optix = corr;
    }
    #endif // RMCL_OPTIX

    sensor->fetchTF();
    sensor->updateCorrectors();
    
    return true;
}

void MICP::loadMap(std::string filename)
{
    #ifdef RMCL_EMBREE
    m_map_embree = rm::importEmbreeMap(filename);
    #endif // RMCL_EMBREE

    #ifdef RMCL_OPTIX
    m_map_optix = rm::importOptixMap(filename);
    #endif // RMCL_OPTIX
}

void MICP::correct(
    const rm::MemoryView<rm::Transform, rm::RAM>& Tbm,
    rm::MemoryView<rm::Transform, rm::RAM>& dT)
{
    rm::StopWatch sw;
    double el;
    double el_total = 0.0;

    // std::cout << "-----------------" << std::endl;
    
    sw();
    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbm_ = Tbm;

    CorrectionPreResults<rm::VRAM_CUDA> res_;
    res_.ms.resize(Tbm.size());
    res_.ds.resize(Tbm.size());
    res_.Cs.resize(Tbm.size());
    res_.Ncorr.resize(Tbm.size());
    

    std::vector<CorrectionPreResults<rm::RAM> > results;
    float weight_sum = 0.0;
    std::vector<float> weights;
    el = sw();
    el_total += el;
    
    // std::cout << "- preprocessing: " << el * 1000.0 << " ms" << std::endl;

    sw();
    for(auto elem : m_sensors)
    {
        if(elem.second->data_received_once)
        {
            CorrectionPreResults<rm::RAM> res;

            if(elem.second->backend == 0)
            {
                res.ms.resize(Tbm.size());
                res.ds.resize(Tbm.size());
                res.Cs.resize(Tbm.size());
                res.Ncorr.resize(Tbm.size());

                elem.second->computeCovs(Tbm, res);
            } else if(elem.second->backend == 1) {
                // use preuploaded poses as input
                elem.second->computeCovs(Tbm_, res_);

                // download
                res.ms = res_.ms;
                res.ds = res_.ds;
                res.Cs = res_.Cs;
                res.Ncorr = res_.Ncorr;
            }

            results.push_back(res);

            // dynamic weights
            float w = elem.second->corr_weight;

            weight_sum += w;
            weights.push_back(w);
        } else {
            std::cout << "WARNING: " << elem.second->name << " still not received data" << std::endl;
        }
    }
    el = sw();
    el_total += el;
    // std::cout << "- computing covs (" << results.size() << " sensors): " << el * 1000.0 << " ms" << std::endl;

    if(results.size() > 0)
    {
        // normalize weights
        sw();
        for(size_t i=0; i<weights.size(); i++)
        {
            weights[i] /= weight_sum;
        }

        CorrectionPreResults<rm::RAM> results_combined;
        results_combined.ms.resize(Tbm.size());
        results_combined.ds.resize(Tbm.size());
        results_combined.Cs.resize(Tbm.size());
        results_combined.Ncorr.resize(Tbm.size());
        
        el = sw();
        el_total += el;
        // std::cout << "- merging preprocessing: " << el * 1000.0 << " ms" << std::endl;

        sw();
        weighted_average(
            results,
            weights,
            results_combined);
        el = sw();
        el_total += el;

        // std::cout << "- weighted average: " << el * 1000.0 << " ms" << std::endl;

        sw();
        static Correction corr;
        corr.correction_from_covs(results_combined, dT);
        el = sw();
        el_total += el;

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

rmagine::Memory<rmagine::Transform, rmagine::RAM> MICP::correct(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbm)
{
    rm::Memory<rm::Transform, rm::RAM> dT(Tbm.size());
    correct(Tbm, dT);
    return dT;
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

    ros::Duration timeout_inner(0.1);

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
        } else {
            // unknown type
            return;
        }

        ros::spinOnce();
        timeout_inner.sleep();
        curr = ros::Time::now();
    }
}

void MICPRangeSensor::connect()
{
    if(type == 0) { // Spherical
        if(data_topic.msg == "rmcl_msgs/ScanStamped") {
            data_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<rmcl_msgs::ScanStamped>(
                        data_topic.name, 1, 
                        &MICPRangeSensor::sphericalCB, this
                    )
                );
        } else if(data_topic.msg == "sensor_msgs/PointCloud2") {
            data_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<sensor_msgs::PointCloud2>(
                        data_topic.name, 1, 
                        &MICPRangeSensor::pclSphericalCB, this
                    )
                );
        } else if(data_topic.msg == "sensor_msgs/LaserScan") {
            data_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<sensor_msgs::LaserScan>(
                        data_topic.name, 1, 
                        &MICPRangeSensor::laserCB, this
                    )
                );
        } else {
            // TODO proper error msg
            std::cout << data_topic.msg << " message unknown for sensor type " << type << std::endl;
        }
    } else if(type == 1) { // Pinhole
        
        if(data_topic.msg == "rmcl_msgs/DepthStamped") {
            data_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<rmcl_msgs::DepthStamped>(
                        data_topic.name, 1, 
                        &MICPRangeSensor::pinholeCB, this
                    )
                );
        } else if(data_topic.msg == "sensor_msgs/PointCloud2") {
            data_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<sensor_msgs::PointCloud2>(
                        data_topic.name, 1, 
                        &MICPRangeSensor::pclPinholeCB, this
                    )
                );
        } else if(data_topic.msg == "sensor_msgs/Image") {
            // std::cout << "Connecting to depth image" << std::endl;
            // create image transport if not yet done

            if(!it)
            {
                it.reset(new image_transport::ImageTransport(*nh));
            }

            img_sub = std::make_shared<image_transport::Subscriber>(
                    it->subscribe(
                        data_topic.name, 1,
                        &MICPRangeSensor::imageCB, this)
                );
        }

    } else if(type == 2) { // O1Dn
        if(data_topic.msg == "rmcl_msgs/O1DnStamped") {
            data_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<rmcl_msgs::O1DnStamped>(
                        data_topic.name, 1, 
                        &MICPRangeSensor::o1dnCB, this
                    )
                );
        }
    } else if(type == 3) { // OnDn
        if(data_topic.msg == "rmcl_msgs/OnDnStamped") {
            data_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<rmcl_msgs::OnDnStamped>(
                        data_topic.name, 1, 
                        &MICPRangeSensor::ondnCB, this
                    )
                );
        }
    }

    if(has_info_topic)
    {
        // connect to info topic
        if(info_topic.msg == "sensor_msgs/CameraInfo")
        {
            info_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<sensor_msgs::CameraInfo>(
                        info_topic.name, 1, 
                        &MICPRangeSensor::cameraInfoCB, this
                    )
                );
        } else if(info_topic.msg == "rmcl_msgs/ScanInfo") {
            info_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<rmcl_msgs::ScanInfo>(
                        info_topic.name, 1, 
                        &MICPRangeSensor::sphericalModelCB, this
                    )
                );
        } else if(info_topic.msg == "rmcl_msgs/DepthInfo") {
            info_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<rmcl_msgs::DepthInfo>(
                        info_topic.name, 1, 
                        &MICPRangeSensor::pinholeModelCB, this
                    )
                );
        } else if(info_topic.msg == "rmcl_msgs/O1DnInfo") {
            info_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<rmcl_msgs::O1DnInfo>(
                        info_topic.name, 1, 
                        &MICPRangeSensor::o1dnModelCB, this
                    )
                );
        } else if(info_topic.msg == "rmcl_msgs/OnDnInfo") {
            info_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<rmcl_msgs::OnDnInfo>(
                        info_topic.name, 1, 
                        &MICPRangeSensor::ondnModelCB, this
                    )
                );
        } else {
            std::cout << "info topic message " << info_topic.msg << " not supported" << std::endl; 
        }
        
    }
}


void MICPRangeSensor::fetchTF()
{
    geometry_msgs::TransformStamped T_sensor_base;
    
    if(frame != base_frame)
    {
        try 
        {
            T_sensor_base = tf_buffer->lookupTransform(base_frame, frame, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ROS_WARN_STREAM("Source: " << frame << ", Target: " << base_frame);

            return;
        }
    } else {
        T_sensor_base.header.frame_id = base_frame;
        T_sensor_base.child_frame_id = frame;
        T_sensor_base.transform.translation.x = 0.0;
        T_sensor_base.transform.translation.y = 0.0;
        T_sensor_base.transform.translation.z = 0.0;
        T_sensor_base.transform.rotation.x = 0.0;
        T_sensor_base.transform.rotation.y = 0.0;
        T_sensor_base.transform.rotation.z = 0.0;
        T_sensor_base.transform.rotation.w = 1.0;
    }

    convert(T_sensor_base.transform, Tsb);
}

void MICPRangeSensor::updateCorrectors()
{
    #ifdef RMCL_EMBREE
    if(corr_sphere_embree)
    {
        corr_sphere_embree->setParams(corr_params);
        corr_sphere_embree->setModel(std::get<0>(model));
        corr_sphere_embree->setInputData(ranges);
        corr_sphere_embree->setTsb(Tsb);
    } else if(corr_pinhole_embree) {
        corr_pinhole_embree->setParams(corr_params);
        corr_pinhole_embree->setModel(std::get<1>(model));
        corr_pinhole_embree->setInputData(ranges);
        corr_pinhole_embree->setTsb(Tsb);
    } else if(corr_o1dn_embree) {
        corr_o1dn_embree->setParams(corr_params);
        corr_o1dn_embree->setModel(std::get<2>(model));
        corr_o1dn_embree->setInputData(ranges);
        corr_o1dn_embree->setTsb(Tsb);
    } else if(corr_ondn_embree) {
        corr_ondn_embree->setParams(corr_params);
        corr_ondn_embree->setModel(std::get<3>(model));
        corr_ondn_embree->setInputData(ranges);
        corr_ondn_embree->setTsb(Tsb);
    }
    #endif // RMCL_EMBREE
    
    #ifdef RMCL_OPTIX
    if(corr_sphere_optix)
    {
        corr_sphere_optix->setParams(corr_params);
        corr_sphere_optix->setModel(std::get<0>(model));
        corr_sphere_optix->setInputData(ranges_gpu);
        corr_sphere_optix->setTsb(Tsb);
    } else if(corr_pinhole_optix) {
        corr_pinhole_optix->setParams(corr_params);
        corr_pinhole_optix->setModel(std::get<1>(model));
        corr_pinhole_optix->setInputData(ranges_gpu);
        corr_pinhole_optix->setTsb(Tsb);
    } else if(corr_o1dn_optix) {
        corr_o1dn_optix->setParams(corr_params);
        corr_o1dn_optix->setModel(std::get<2>(model));
        corr_o1dn_optix->setInputData(ranges_gpu);
        corr_o1dn_optix->setTsb(Tsb);
    } else if(corr_ondn_optix) {
        corr_ondn_optix->setParams(corr_params);
        corr_ondn_optix->setModel(std::get<3>(model));
        corr_ondn_optix->setInputData(ranges_gpu);
        corr_ondn_optix->setTsb(Tsb);
    }
    #endif // RMCL_OPTIX
}

void MICPRangeSensor::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    CorrectionPreResults<rmagine::RAM>& res)
{
    // this is bad. maybe we must go away from having a completely generic sensor

    // std::cout << "Compute Covs - CPU" << std::endl;

    #ifdef RMCL_EMBREE
    if(backend == 0)
    {
        if(type == 0) {
            corr_sphere_embree->compute_covs(Tbms, res);
        } else if(type == 1) {
            corr_pinhole_embree->compute_covs(Tbms, res);   
        } else if(type == 2) {
            corr_o1dn_embree->compute_covs(Tbms, res);
        } else if(type == 3) {
            corr_ondn_embree->compute_covs(Tbms, res);
        }
    }
    #endif // RMCL_EMBREE
    
    #ifdef RMCL_OPTIX
    if(backend == 1)
    {
        // upload
        rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbms_ = Tbms;
        CorrectionPreResults<rm::VRAM_CUDA> res_;
        res_.Cs.resize(Tbms.size());
        res_.ms.resize(Tbms.size());
        res_.ds.resize(Tbms.size());
        res_.Ncorr.resize(Tbms.size());

        // compute
        if(type == 0) {
            corr_sphere_optix->compute_covs(Tbms_, res_);
        } else if(type == 1) {
            corr_pinhole_optix->compute_covs(Tbms_, res_);   
        } else if(type == 2) {
            corr_o1dn_optix->compute_covs(Tbms_, res_);
        } else if(type == 3) {
            corr_ondn_optix->compute_covs(Tbms_, res_);
        }

        // download
        res.Cs = res_.Cs;
        res.ms = res_.ms;
        res.ds = res_.ds;
        res.Ncorr = res_.Ncorr;
    }
    #endif // RMCL_OPTIX

    // std::cout << "Compute Covs - end" << std::endl;

}

void MICPRangeSensor::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms,
    CorrectionPreResults<rmagine::VRAM_CUDA>& res)
{
    // this is bad. maybe we must go away from having a completely generic sensor
    // std::cout << "Compute Covs" << std::endl;

    #ifdef RMCL_EMBREE
    if(backend == 0)
    {
        // download
        rm::Memory<rm::Transform, rm::RAM> Tbms_ = Tbms;
        CorrectionPreResults<rm::RAM> res_;
        res_.Cs.resize(Tbms.size());
        res_.ms.resize(Tbms.size());
        res_.ds.resize(Tbms.size());
        res_.Ncorr.resize(Tbms.size());

        if(type == 0) {
            corr_sphere_embree->compute_covs(Tbms_, res_);
        } else if(type == 1) {
            corr_pinhole_embree->compute_covs(Tbms_, res_);   
        } else if(type == 2) {
            corr_o1dn_embree->compute_covs(Tbms_, res_);
        } else if(type == 3) {
            corr_ondn_embree->compute_covs(Tbms_, res_);
        }

        // upload
        res.Cs = res_.Cs;
        res.ms = res_.ms;
        res.ds = res_.ds;
        res.Ncorr = res_.Ncorr;
    }
    #endif // RMCL_EMBREE
    
    #ifdef RMCL_OPTIX
    if(backend == 1)
    {
        // compute
        if(type == 0) {
            corr_sphere_optix->compute_covs(Tbms, res);
        } else if(type == 1) {
            corr_pinhole_optix->compute_covs(Tbms, res);   
        } else if(type == 2) {
            corr_o1dn_optix->compute_covs(Tbms, res);
        } else if(type == 3) {
            corr_ondn_optix->compute_covs(Tbms, res);
        }
    }
    #endif // RMCL_OPTIX

    // std::cout << "Compute Covs. done" << std::endl;
}


void MICPRangeSensor::sphericalCB(
    const rmcl_msgs::ScanStamped::ConstPtr& msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
    fetchTF();

    // model
    rm::SphericalModel model_;
    convert(msg->scan.info, model_);
    model = model_;

    // data
    if(ranges.size() < msg->scan.ranges.size())
    {
        ranges.resize(msg->scan.ranges.size());
    }
    std::copy(msg->scan.ranges.begin(), msg->scan.ranges.end(), ranges.raw());
    // upload
    ranges_gpu = ranges;

    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
}

void MICPRangeSensor::pinholeCB(
    const rmcl_msgs::DepthStamped::ConstPtr& msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
    fetchTF();

    // model
    rm::PinholeModel model_;
    convert(msg->depth.info, model_);
    model = model_;

    // data
    if(ranges.size() < msg->depth.ranges.size())
    {
        ranges.resize(msg->depth.ranges.size());
    }
    std::copy(msg->depth.ranges.begin(), msg->depth.ranges.end(), ranges.raw());
    // upload
    ranges_gpu = ranges;

    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
}

void MICPRangeSensor::o1dnCB(
    const rmcl_msgs::O1DnStamped::ConstPtr& msg)
{
    fetchTF();

    // model
    rm::O1DnModel model_;
    convert(msg->o1dn.info, model_);
    model = model_;

    // data
    if(ranges.size() < msg->o1dn.ranges.size())
    {
        ranges.resize(msg->o1dn.ranges.size());
    }
    std::copy(msg->o1dn.ranges.begin(), msg->o1dn.ranges.end(), ranges.raw());
    // upload
    ranges_gpu = ranges;

    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
}

void MICPRangeSensor::ondnCB(
    const rmcl_msgs::OnDnStamped::ConstPtr& msg)
{
    fetchTF();

    // model
    rm::OnDnModel model_;
    convert(msg->ondn.info, model_);
    model = model_;

    // data
    if(ranges.size() < msg->ondn.ranges.size())
    {
        ranges.resize(msg->ondn.ranges.size());
    }
    std::copy(msg->ondn.ranges.begin(), msg->ondn.ranges.end(), ranges.raw());
    // upload
    ranges_gpu = ranges;

    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
}

void MICPRangeSensor::pclSphericalCB(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
    fetchTF();
    
    
    auto model_ = std::get<0>(model);

    if(ranges.size() < msg->width * msg->height)
    {
        ranges.resize(msg->width * msg->height);
        // fill with nans
        for(size_t i=0; i < ranges.size(); i++)
        {
            ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    sensor_msgs::PointField field_x;
    sensor_msgs::PointField field_y;
    sensor_msgs::PointField field_z;

    for(size_t i=0; i<msg->fields.size(); i++)
    {
        if(msg->fields[i].name == "x")
        {
            field_x = msg->fields[i];
        }
        if(msg->fields[i].name == "y")
        {
            field_y = msg->fields[i];
        }
        if(msg->fields[i].name == "z")
        {
            field_z = msg->fields[i];
        }
    }


    for(size_t i=0; i<msg->width * msg->height; i++)
    {
        const uint8_t* data_ptr = &msg->data[i * msg->point_step];

        // rmagine::Vector point;
        
        float x,y,z;

        if(field_x.datatype == sensor_msgs::PointField::FLOAT32)
        {
            // Float
            x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
            y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
            z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
        } else if(field_x.datatype == sensor_msgs::PointField::FLOAT64) {
            // Double
            x = *reinterpret_cast<const double*>(data_ptr + field_x.offset);
            y = *reinterpret_cast<const double*>(data_ptr + field_y.offset);
            z = *reinterpret_cast<const double*>(data_ptr + field_z.offset);
        } else {
            throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
        }

        if(!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
        {
            float range_est = sqrt(x*x + y*y + z*z);
            float theta_est = atan2(y, x);
            float phi_est = atan2(z, range_est);
            
            unsigned int phi_id = ((phi_est - model_.phi.min) / model_.phi.inc) + 0.5;
            unsigned int theta_id = ((theta_est - model_.theta.min) / model_.theta.inc) + 0.5;
            unsigned int p_id = model_.getBufferId(phi_id, theta_id);

            if(p_id >= 0 && p_id < model_.size())
            {
                ranges[p_id] = range_est;
            }
        }
    }

    // upload
    ranges_gpu = ranges;

    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
}

void MICPRangeSensor::pclPinholeCB(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
    fetchTF();

    auto model_ = std::get<1>(model);

    if(ranges.size() < msg->width * msg->height)
    {
        ranges.resize(msg->width * msg->height);
    }

    sensor_msgs::PointField field_x;
    sensor_msgs::PointField field_y;
    sensor_msgs::PointField field_z;

    for(size_t i=0; i<msg->fields.size(); i++)
    {
        if(msg->fields[i].name == "x")
        {
            field_x = msg->fields[i];
        }
        if(msg->fields[i].name == "y")
        {
            field_y = msg->fields[i];
        }
        if(msg->fields[i].name == "z")
        {
            field_z = msg->fields[i];
        }
    }

    // what to do if order is different?
    for(size_t i=0; i<msg->width * msg->height; i++)
    {
        const uint8_t* data_ptr = &msg->data[i * msg->point_step];

        float x,y,z;
        if(field_x.datatype == sensor_msgs::PointField::FLOAT32)
        {
            // Float
            x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
            y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
            z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
        } else if(field_x.datatype == sensor_msgs::PointField::FLOAT64) {
            // Double
            x = *reinterpret_cast<const double*>(data_ptr + field_x.offset);
            y = *reinterpret_cast<const double*>(data_ptr + field_y.offset);
            z = *reinterpret_cast<const double*>(data_ptr + field_z.offset);
        } else {
            throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
        }

        if(!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
        {
            float range_est = sqrt(x*x + y*y + z*z);
            ranges[i] = range_est;
        } else {
            ranges[i] = model_.range.max + 1.0;
            // this does not work ( check if it is working - should do)
            // ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    // upload
    ranges_gpu = ranges;

    // meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
}

void MICPRangeSensor::laserCB(
    const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
    fetchTF();

    // model
    rm::SphericalModel model_ = std::get<0>(model);
    convert(*msg, model_);
    model = model_;

    // data
    if(ranges.size() < msg->ranges.size())
    {
        ranges.resize(msg->ranges.size());
    }
    std::copy(msg->ranges.begin(), msg->ranges.end(), ranges.raw());
    ranges_gpu = ranges;


    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
}

void MICPRangeSensor::imageCB(
    const sensor_msgs::Image::ConstPtr& msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
    fetchTF();
    
    unsigned int bytes = msg->step / msg->width;
    
    if(msg->encoding == "32FC1")
    {
        if(bytes != 4)
        {
            std::cout << "32FC1 should have 4 bytes. Calculated " << bytes << std::endl;
        }

        // Problem: 
        // floating point value in pixel is not the range in meters!
        // it is the scale of the camera vector intersecting the pixel
        // with z=1
        // the solution to that is in the for loop
        // -- Tested on simulated depth images --
        if(ranges.size() < msg->width * msg->height)
        {
            ranges.resize(msg->width * msg->height);
        }

        auto model_ = std::get<1>(model);

        for(unsigned int vid = 0; vid < model_.getHeight(); vid++)
        {
            for(unsigned int hid = 0; hid < model_.getWidth(); hid++)
            {
                unsigned int loc_id = model_.getBufferId(vid, hid);
                const float wrong_range = *reinterpret_cast<const float*>(&msg->data[loc_id * sizeof(float)]);

                rm::Vector dir;
                float real_range;

                if(optical_coordinates)
                {
                    // std::cout << "OPTICAL" << std::endl;
                    dir = model_.getDirectionOptical(vid, hid);
                    real_range = wrong_range / dir.z;
                } else {
                    // std::cout << "NO OPTICAL" << std::endl;
                    dir = model_.getDirection(vid, hid);
                    real_range = wrong_range / dir.x;
                }

                ranges[loc_id] = real_range;
            }
        }
        
    } else {
        ROS_WARN_STREAM("Could not convert image of encoding " << msg->encoding);
    }

    ranges_gpu = ranges;

    // meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    // update corrector
    updateCorrectors();
}

// info callbacks

void MICPRangeSensor::sphericalModelCB(
    const rmcl_msgs::ScanInfo::ConstPtr& msg)
{
    rm::SphericalModel model_ = std::get<0>(model);
    convert(*msg, model_);
    model = model_;
}

void MICPRangeSensor::pinholeModelCB(
    const rmcl_msgs::DepthInfo::ConstPtr& msg)
{
    rm::PinholeModel model_ = std::get<1>(model);
    convert(*msg, model_);
    model = model_;
}

void MICPRangeSensor::o1dnModelCB(
    const rmcl_msgs::O1DnInfo::ConstPtr& msg)
{
    rm::O1DnModel model_ = std::get<2>(model);
    convert(*msg, model_);
    model = model_;
}

void MICPRangeSensor::ondnModelCB(
    const rmcl_msgs::OnDnInfo::ConstPtr& msg)
{
    rm::OnDnModel model_ = std::get<3>(model);
    convert(*msg, model_);
    model = model_;
}

void MICPRangeSensor::cameraInfoCB(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    // ROS_INFO_STREAM("sensor - info: " << name << " received " << data_topic.msg << " message");

    if(msg->header.frame_id != frame)
    {
        std::cout << "WARNING: Image and CameraInfo are not in the same frame" << std::endl;
    }

    rm::PinholeModel model_ = std::get<1>(model);
    convert(*msg, model_);
    model = model_;
}


} // namespace rmcl