#include "rmcl/correction/MICP.hpp"
#include <ros/master.h>
#include <vector>



// supported sensor data
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <rmcl_msgs/ScanStamped.h>
#include <rmcl_msgs/DepthStamped.h>

namespace rm = rmagine;

namespace rmcl
{

MICP::MICP()
:m_nh()
,m_nh_p("~")
{
    std::cout << "MICP initiailized" << std::endl;

    std::cout << "Available backends:" << std::endl;

    #ifdef RMCL_EMBREE
    std::cout << "- CPU (Embree)" << std::endl;
    #endif

    #ifdef RMCL_OPTIX
    std::cout << "- GPU (Optix)" << std::endl;
    #endif
}

MICP::~MICP()
{
    std::cout << "MICP cleanup" << std::endl;
}

struct TopicDataInfo
{
    bool available = false;
    std::string frame = "";
    // double frequency;
};

TopicDataInfo check_topic_data(
    std::string name, 
    std::string type, 
    ros::NodeHandle nh,
    ros::Duration timeout = ros::Duration(5.0))
{
    TopicDataInfo info = {};



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

    while(ros::ok() && curr < end && !info.available)
    {
        if(type == "sensor_msgs/PointCloud2")
        {
            auto msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(name, nh, timeout_inner);
            if(msg)
            {
                info.available = true;
                info.frame = msg->header.frame_id;
            }
        } else if(type == "sensor_msgs/PointCloud") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::PointCloud>(name, nh, timeout_inner);
            if(msg)
            {
                info.available = true;
                info.frame = msg->header.frame_id;
            }
        } else if(type == "sensor_msgs/LaserScan") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(name, nh, timeout_inner);
            if(msg)
            {
                info.available = true;
                info.frame = msg->header.frame_id;
            }
        } else if(type == "sensor_msgs/Image") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::Image>(name, nh, timeout_inner);
            if(msg)
            {
                info.available = true;
                info.frame = msg->header.frame_id;
            }
        } else if(type == "sensor_msgs/CameraInfo") {
            auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(name, nh, timeout_inner);
            if(msg)
            {
                info.available = true;
                info.frame = msg->header.frame_id;
            }
        } else if(type == "rmcl_msgs/ScanStamped") {
            auto msg = ros::topic::waitForMessage<rmcl_msgs::ScanStamped>(name, nh, timeout_inner);
            if(msg)
            {
                info.available = true;
                info.frame = msg->header.frame_id;
            }
        } else if(type == "rmcl_msgs/DepthStamped") {
            auto msg = ros::topic::waitForMessage<rmcl_msgs::DepthStamped>(name, nh, timeout_inner);
            if(msg)
            {
                info.available = true;
                info.frame = msg->header.frame_id;
            }
        } else {
            // unknown type
            return info;
        }

        ros::spinOnce();
        timeout_inner.sleep();
        curr = ros::Time::now();
    }

    return info;
}

void MICP::loadParams()
{
    std::cout << "MICP load params" << std::endl;


    if(!m_nh_p.getParam("map_file", m_map_filename))
    {
        ROS_ERROR("User must provide ~map_file");
    }

    loadMap(m_map_filename);

    XmlRpc::XmlRpcValue sensors_xml;
    if(m_nh_p.getParam("sensors", sensors_xml))
    {
        std::cout << "LOADING SENSORS..." << std::endl;
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
    // std::string sensor_name = sensor_xml.first;
    std::string topic_name;
    std::string topic_msg;
    bool        topic_valid = false;

    std::cout << "- \033[1;33m" << sensor_name << "\033[0m" << std::endl;
    // XmlRpc::XmlRpcValue sensor_params = sensor_xml.second;

    if(sensor_params.hasMember("topic"))
    {
        topic_name = (std::string)sensor_params["topic"];
        if(topic_name[0] != '/')
        {
            topic_name = m_nh.getNamespace() + topic_name;
        }
        std::cout << "  - topic: \033[1;36m" << topic_name << "\033[0m" << std::endl;
    } else {
        std::cout << "  - topic: \033[1;31mnot found\033[0m" << std::endl;
        return false;
    }

    std::vector<ros::master::TopicInfo> topic_infos;
    ros::master::getTopics(topic_infos);
    for(auto topic_info : topic_infos)
    {
        if(topic_info.name == topic_name)
        {
            topic_msg = topic_info.datatype;
        }
    }

    if(topic_msg != "")
    {
        std::cout << "    - msg: " << topic_msg  << std::endl;
        // check if topic is valid
        auto topic_data_info = check_topic_data(topic_name, topic_msg, m_nh, ros::Duration(5.0));
        topic_valid = topic_data_info.available;

        if(topic_valid)
        {
            std::cout << "    - data: \033[1;32myes (" << topic_data_info.frame << ")\033[0m" << std::endl;
        } else {
            std::cout << "    - data: \033[1;31mno\033[0m" << std::endl;
        }
    } else {
        std::cout << "    - msg: \033[1;31mnot found\033[0m" << std::endl;
        return false;
    }

    if(sensor_params.hasMember("info_topic"))
    {
        std::string info_topic_name = sensor_params["info_topic"];
        if(info_topic_name[0] != '/')
        {
            info_topic_name = m_nh.getNamespace() + info_topic_name;
        }

        std::cout << "  - info topic: \033[1;36m" << info_topic_name << "\033[0m" << std::endl;

        std::string info_topic_msg = "";
        for(auto topic_info : topic_infos)
        {
            if(topic_info.name == info_topic_name)
            {
                info_topic_msg = topic_info.datatype;
            }
        }

        if(info_topic_msg != "")
        {
            std::cout << "    - msg: " << info_topic_msg << std::endl;
        } else {
            std::cout << "    - msg: \033[1;31mnot found\033[0m"  << std::endl;
            return false;
        }

        // std::cout << "-- info: " << 
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

} // namespace rmcl