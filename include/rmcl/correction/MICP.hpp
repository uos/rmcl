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
#include <rmcl_msgs/ScanStamped.h>
#include <rmcl_msgs/DepthStamped.h>
#include <rmcl_msgs/O1DnStamped.h>
#include <rmcl_msgs/OnDnStamped.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


#include <tf2_ros/transform_listener.h>

#include <variant>

#include <image_transport/image_transport.h>




namespace rmcl
{

// ROS related renamings
using NodeHandlePtr = std::shared_ptr<ros::NodeHandle>;
using SubscriberPtr = std::shared_ptr<ros::Subscriber>;
using ImageTransportPtr = std::shared_ptr<image_transport::ImageTransport>;
using ITSubscriberPtr = std::shared_ptr<image_transport::Subscriber>;
using TFBufferPtr = std::shared_ptr<tf2_ros::Buffer>;
using TFListenerPtr = std::shared_ptr<tf2_ros::TransformListener>;



using SensorModelV = std::variant<
    rmagine::SphericalModel,
    rmagine::PinholeModel,
    rmagine::O1DnModel,
    rmagine::OnDnModel
>;


struct TopicInfo
{
    std::string     name;
    std::string     msg;
    bool            data;
    std::string     frame;
};

struct MICPRangeSensor 
: std::enable_shared_from_this<MICPRangeSensor>
{
    std::string     name;

    TopicInfo       data_topic;
    // optional
    bool            has_info_topic;
    TopicInfo       info_topic;

    // robots base frame
    std::string          frame;
    std::string          base_frame;
    rmagine::Transform   Tsb;

    // computing backend
    unsigned int         backend = 0;

    // model
    // 0: spherical, 1: pinhole, 2: O1Dn, 3: OnDn 
    unsigned int         type;
    SensorModelV         model;
    // model meta
    bool                 model_received_once = false;
    ros::Time            model_last_update;
    float                model_frequency_est; // currently unused
    


    // data    
    rmagine::Memory<float, rmagine::RAM>        ranges;
    rmagine::Memory<float, rmagine::VRAM_CUDA>  ranges_gpu;

    // data meta
    bool        data_received_once = false;
    ros::Time   data_last_update;
    float       data_frequency_est; // currently unused

    
    // subscriber to data
    NodeHandlePtr nh;
    SubscriberPtr data_sub;
    SubscriberPtr info_sub;

    ImageTransportPtr it;
    ITSubscriberPtr img_sub;
    bool optical_coordinates = false;

    TFBufferPtr  tf_buffer;


    CorrectionParams            corr_params;
    float                       corr_weight = 1.0;

    // correction: TODO better
    #ifdef RMCL_EMBREE
    SphereCorrectorEmbreePtr     corr_sphere_embree;
    PinholeCorrectorEmbreePtr    corr_pinhole_embree;
    O1DnCorrectorEmbreePtr       corr_o1dn_embree;
    OnDnCorrectorEmbreePtr       corr_ondn_embree;
    #endif // RMCL_EMBREE

    #ifdef RMCL_OPTIX
    SphereCorrectorOptixPtr     corr_sphere_optix;
    PinholeCorrectorOptixPtr    corr_pinhole_optix;
    O1DnCorrectorOptixPtr       corr_o1dn_optix;
    OnDnCorrectorOptixPtr       corr_ondn_optix;
    #endif // RMCL_OPTIX

    void connect();

    // called once every new data message
    void fetchTF();
    void updateCorrectors();

    // do corrections depending on the current sensor state
    void computeCovs(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
        CorrectionPreResults<rmagine::RAM>& res);

    void computeCovs(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms,
        CorrectionPreResults<rmagine::VRAM_CUDA>& res);


    // callbacks
    // internal rmcl msgs
    void sphericalCB(
        const rmcl_msgs::ScanStamped::ConstPtr& msg);

    void pinholeCB(
        const rmcl_msgs::DepthStamped::ConstPtr& msg);

    // external commonly used messages
    void pclSphericalCB(
        const sensor_msgs::PointCloud2::ConstPtr& msg);

    void pclPinholeCB(
        const sensor_msgs::PointCloud2::ConstPtr& msg);

    void laserCB(
        const sensor_msgs::LaserScan::ConstPtr& msg);

    void imageCB(
        const sensor_msgs::Image::ConstPtr& msg);
    

    // info callbacks
    void cameraInfoCB(
        const sensor_msgs::CameraInfo::ConstPtr& msg);
};

using MICPRangeSensorPtr = std::shared_ptr<MICPRangeSensor>;

class MICP
{
public:
    MICP();
    ~MICP();

    void loadParams();

    bool loadSensor(std::string sensor_name, 
        XmlRpc::XmlRpcValue sensor_params);

    void loadMap(std::string filename);

    void correct(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbm,
        rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& dT);

    rmagine::Memory<rmagine::Transform, rmagine::RAM> correct(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbm);


    inline std::unordered_map<std::string, MICPRangeSensorPtr> sensors()
    {
        return m_sensors;
    }
protected:
    bool checkTF(bool prints = false);

    void checkTopic(
        TopicInfo& info, 
        ros::Duration timeout = ros::Duration(5.0));

    void initCorrectors();
private:
    // ROS
    NodeHandlePtr   m_nh;
    NodeHandlePtr   m_nh_p;
    TFBufferPtr     m_tf_buffer;
    TFListenerPtr   m_tf_listener;

    std::string m_base_frame;
    std::string m_map_frame;

    std::string m_odom_frame;
    bool        m_use_odom_frame;

    // MAP
    std::string m_map_filename;

    std::unordered_map<std::string, MICPRangeSensorPtr> m_sensors;
    
    #ifdef RMCL_EMBREE
    rmagine::EmbreeMapPtr m_map_embree;
    #endif // RMCL_EMBREE

    #ifdef RMCL_OPTIX
    rmagine::OptixMapPtr m_map_optix;
    #endif // RMCL_OPTIX
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