#ifndef RMCL_CORRECTION_MICP_HPP
#define RMCL_CORRECTION_MICP_HPP

#include <ros/ros.h>
#include <rmagine/types/sensor_models.h>
#include <memory>
#include <unordered_map>



#ifdef RMCL_EMBREE
#include <rmagine/map/EmbreeMap.hpp>

#include <rmcl/correction/SphereCorrectorEmbree.hpp>
#include <rmcl/correction/PinholeCorrectorEmbree.hpp>
#include <rmcl/correction/O1DnCorrectorEmbree.hpp>
#include <rmcl/correction/OnDnCorrectorEmbree.hpp>
#endif // RMCL_EMBREE


#ifdef RMCL_OPTIX
#include <rmagine/map/OptixMap.hpp>

#include <rmcl/correction/SphereCorrectorOptix.hpp>
#include <rmcl/correction/PinholeCorrectorOptix.hpp>
#include <rmcl/correction/O1DnCorrectorOptix.hpp>
#include <rmcl/correction/OnDnCorrectorOptix.hpp>
#endif // RMCL_OPTIX


// supported sensor data
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <rmcl_msgs/ScanStamped.h>
#include <rmcl_msgs/DepthStamped.h>

#include <tf2_ros/transform_listener.h>

#include <variant>


using SensorModelV = std::variant<
    rmagine::SphericalModel,
    rmagine::PinholeModel,
    rmagine::O1DnModel,
    rmagine::OnDnModel
    >;


namespace rmcl
{

struct TopicInfo
{
    std::string     name;
    std::string     msg;
    bool            data;
    std::string     frame;
};

struct MICPRangeSensor
{
    std::string     name;

    TopicInfo       data_topic;
    // optional
    bool            has_info_topic;
    TopicInfo       info_topic;

    unsigned int         type; // 0: spherical, 1: pinhole, 2: O1Dn, 3: OnDn 
    SensorModelV         model;

    // data
    ros::Time   data_last_update;
    rmagine::Memory<float, rmagine::VRAM_CUDA>  ranges;
};

class MICP
{
public:
    MICP();
    ~MICP();

    void loadParams();

    bool loadSensor(std::string sensor_name, 
        XmlRpc::XmlRpcValue sensor_params);

    void loadMap(std::string filename);

protected:
    // callbacks: TODO
    // void pclSphericalCB(
    //     const sensor_msgs::PointCloud2::ConstPtr& msg, 
    //     std::string sensor_name);

    // void pclDepthCB(
    //     const sensor_msgs::PointCloud2::ConstPtr& msg, 
    //     std::string sensor_name
    // );

private:

    bool checkTF(bool prints = false);

    void checkTopic(
        TopicInfo& info, 
        ros::Duration timeout = ros::Duration(5.0));

    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_p;

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;


    std::string m_base_frame;
    std::string m_map_frame;

    std::string m_odom_frame;
    bool        m_use_odom_frame;

    // MAP
    std::string m_map_filename;
    
    // 
    #ifdef RMCL_EMBREE
    std::unordered_map<std::string, SphereCorrectorEmbreePtr> m_spheres_embree;
    std::unordered_map<std::string, PinholeCorrectorEmbreePtr> m_pinholes_embree;
    std::unordered_map<std::string, O1DnCorrectorEmbreePtr> m_o1dn_embree;
    std::unordered_map<std::string, OnDnCorrectorEmbreePtr> m_ondn_embree;

    rmagine::EmbreeMapPtr m_map_embree;
    #endif // RMCL_EMBREE

    #ifdef RMCL_OPTIX
    std::unordered_map<std::string, SphereCorrectorOptixPtr> m_spheres_optix;
    std::unordered_map<std::string, PinholeCorrectorOptixPtr> m_pinholes_optix;
    std::unordered_map<std::string, O1DnCorrectorOptixPtr> m_o1dn_optix;
    std::unordered_map<std::string, OnDnCorrectorOptixPtr> m_ondn_optix;

    rmagine::OptixMapPtr m_map_optix;
    #endif
};

using MICPPtr = std::shared_ptr<MICP>;

} // namespace rmcl

// TODO: move this to proper header

// TEXT COLORS
#define TC_BLACK    "\033[1;30m"
#define TC_RED      "\033[1;31m"
#define TC_GREEN    "\033[1;32m"
#define TC_YELLOW   "\033[1;33m"
#define TC_BLUE     "\033[1;34m"
#define TC_MAGENTA  "\033[1;35m"
#define TC_CYAN     "\033[1;36m"
#define TC_WHITE    "\033[1;37m"

#define TC_END      "\033[0m"



#define TC_SENSOR   TC_YELLOW
#define TC_TOPIC    TC_CYAN
#define TC_FRAME    TC_MAGENTA
#define TC_MSG      TC_WHITE
#define TC_BACKENDS TC_BLUE


// inline std::ostream& operator<<(
//     std::ostream& os,
//     const rmcl::MICPRangeSensor& sensor)
// {
//     // if(sensor.data_topic.name != "")
//     // {
//     //     os << "  - topic:\t\t" << TC_TOPIC << sensor.data_topic.name << TC_END << std::endl;
//     // }
    


//     return os;
// }

#endif // RMCL_CORRECTION_MICP_HPP