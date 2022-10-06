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


namespace rmcl
{

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
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_p;

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

#endif // RMCL_CORRECTION_MICP_HPP