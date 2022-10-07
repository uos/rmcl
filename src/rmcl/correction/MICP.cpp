#include "rmcl/correction/MICP.hpp"
#include <ros/master.h>
#include <vector>


#include <geometry_msgs/TransformStamped.h>






namespace rm = rmagine;

namespace rmcl
{

MICP::MICP()
:m_nh()
,m_nh_p("~")
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
    m_nh_p.param<std::string>("base_frame", m_base_frame, "base_link");
    m_nh_p.param<std::string>("map_frame", m_map_frame, "map");

    m_use_odom_frame = m_nh_p.getParam("odom_frame", m_odom_frame);

    // check frames


    if(!m_nh_p.getParam("map_file", m_map_filename))
    {
        ROS_ERROR("User must provide ~map_file");
    }

    checkTF(true);

    loadMap(m_map_filename);

    XmlRpc::XmlRpcValue sensors_xml;
    if(m_nh_p.getParam("sensors", sensors_xml))
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

    std::cout << "MICP load params - done." << std::endl;
}

bool MICP::loadSensor(std::string sensor_name, XmlRpc::XmlRpcValue sensor_params)
{
    MICPRangeSensor sensor;

    // std::string sensor_name = sensor_xml.first;
    std::string sensor_type;
    bool        sensor_type_found = false;
    // std::string topic_msg;
    // bool        topic_valid = false;

    sensor.name = sensor_name;

    if(sensor_params.hasMember("type"))
    {
        sensor_type = (std::string)sensor_params["type"];
        if(sensor_type == "spherical") {
            sensor.type = 0;
        } else if(sensor_type == "pinhole") {
            sensor.type = 1;
        } else if(sensor_type == "o1dn") {
            sensor.type = 2;
        } else if(sensor_type == "ondn") {
            sensor.type = 3;
        } else {
            std::cout << "ERROR sensor type unknown: " << sensor_type << std::endl;
            std::cout << "- supported: spherical, pinhole, o1dn, ondn" << std::endl;
            std::cout << "- try to reconstruct type from other params" << std::endl; 
        }
        sensor_type_found = true;
    }

    std::cout << "- " << TC_SENSOR << sensor_name << TC_END << std::endl;
    // XmlRpc::XmlRpcValue sensor_params = sensor_xml.second;

    if(sensor_params.hasMember("topic"))
    {
        std::string topic_name = (std::string)sensor_params["topic"];
        if(topic_name[0] != '/')
        {
            topic_name = m_nh.getNamespace() + topic_name;
        }

        sensor.data_topic.name = topic_name;
        std::cout << "  - topic:\t\t" << TC_TOPIC << sensor.data_topic.name << TC_END << std::endl;
    } else {
        std::cout << "  - topic:\t\t" << TC_RED << "not found" << TC_END << std::endl;
        return false;
    }

    std::vector<ros::master::TopicInfo> topic_infos;
    ros::master::getTopics(topic_infos);
    for(auto topic_info : topic_infos)
    {
        if(topic_info.name == sensor.data_topic.name)
        {
            sensor.data_topic.msg = topic_info.datatype;
            break;
        }
    }

    if(sensor.data_topic.msg != "")
    {
        if(sensor.data_topic.msg == "sensor_msgs/LaserScan" 
            || sensor.data_topic.msg == "rmcl_msgs/ScanStamped")
        {
            sensor_type = "spherical";
            sensor_type_found = true;
        } else if(sensor.data_topic.msg == "sensor_msgs/Image" 
            || sensor.data_topic.msg == "rmcl_msgs/DepthStamped") 
        {
            // is image always pinhole? counterexample: cylindrical image e.g. panorama
            sensor_type = "pinhole";
            sensor_type_found = true;
        }

        std::cout << "    - msg:\t\t" << TC_MSG << sensor.data_topic.msg << TC_END << std::endl;
        // check if topic is valid
        checkTopic(sensor.data_topic, ros::Duration(5.0));

        if(sensor.data_topic.data)
        {
            std::cout << "    - data:\t\t" << TC_GREEN << "yes" << TC_END << std::endl;
            std::cout << "    - frame:\t\t" << TC_FRAME << sensor.data_topic.frame << TC_END << std::endl;
        } else {
            std::cout << "    - data:\t\t" << TC_RED << "no" << TC_END << std::endl;
        }
    } else {
        std::cout << "    - msg:\t\t" << TC_RED << "not found" << TC_END << std::endl;
        return false;
    }

    if(sensor_params.hasMember("info_topic"))
    {
        sensor.has_info_topic = true;
        std::string info_topic_name = sensor_params["info_topic"];
        if(info_topic_name[0] != '/')
        {
            info_topic_name = m_nh.getNamespace() + info_topic_name;
        }
        sensor.info_topic.name = info_topic_name;

        std::cout << "  - info topic:\t\t" << TC_TOPIC << sensor.info_topic.name  << TC_END << std::endl;

        for(auto topic_info : topic_infos)
        {
            if(topic_info.name == info_topic_name)
            {
                sensor.info_topic.msg = topic_info.datatype;
                break;
            }
        }

        if(sensor.info_topic.msg != "")
        {
            std::cout << "    - msg:\t\t" << TC_MSG << sensor.info_topic.msg << TC_END << std::endl;
        } else {
            std::cout << "    - msg:\t\t" << TC_RED << "not found" << TC_END << std::endl;
            return false;
        }

        if(!sensor_type_found)
        {
            if(sensor.info_topic.msg == "sensor_msgs/CameraInfo")
            {
                sensor_type = "pinhole";
                sensor_type_found = true;
            }
        }
    }

    
    // load
    bool model_loaded = false;

    if(sensor_type == "spherical") {
        rm::SphericalModel model;
        
        // fill
        if(sensor_params.hasMember("model"))
        {
            auto model_xml = sensor_params["model"];

            model.theta.min = (double)model_xml["theta_min"];
            model.theta.inc = (double)model_xml["theta_inc"];
            model.theta.size = (int)model_xml["theta_N"];

            model.phi.min = (double)model_xml["phi_min"];
            model.phi.inc = (double)model_xml["phi_inc"];
            model.phi.size = (int)model_xml["phi_N"];

            model.range.min = (double)model_xml["range_min"];
            model.range.max = (double)model_xml["range_max"];

            model_loaded = true;
        } else {
            // if()
            // loadSensorInfo<rm::SphericalModel>();
        }

        sensor.model = model;
    } else if(sensor_type == "pinhole") {
        rm::PinholeModel model;

        sensor.model = model;
    } else if(sensor_type == "o1dn") {
        rm::O1DnModel model;

        sensor.model = model;
    } else if(sensor_type == "ondn") {
        rm::OnDnModel model;

        sensor.model = model;
    }


    if(sensor_type_found)
    {
        if(model_loaded)
        {
            std::cout << "  - type:\t\t" << sensor_type << " - loaded" << std::endl;
        } else {
            std::cout << "  - type:\t\t" << sensor_type << " - loading error" << std::endl;
        }
    } else {
        std::cout << "  - type:\t\t" << TC_RED << "unknown" << std::endl;
    }   

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
            auto msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(info.name, m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "sensor_msgs/PointCloud") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::PointCloud>(info.name, m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "sensor_msgs/LaserScan") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(info.name, m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "sensor_msgs/Image") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::Image>(info.name, m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "sensor_msgs/CameraInfo") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(info.name, m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "rmcl_msgs/ScanStamped") {
            auto msg = ros::topic::waitForMessage<rmcl_msgs::ScanStamped>(info.name, m_nh, timeout_inner);
            if(msg)
            {
                info.data = true;
                info.frame = msg->header.frame_id;
            }
        } else if(info.msg == "rmcl_msgs/DepthStamped") {
            auto msg = ros::topic::waitForMessage<rmcl_msgs::DepthStamped>(info.name, m_nh, timeout_inner);
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