/*
 * Copyright (c) 2022, University Osnabrück
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University Osnabrück nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL University Osnabrück BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * 
 * @brief MICPRangeSensor class describes a RangeSensor in a ROS context
 * 
 * - Range Sensor configuration (model)
 *      - from topic
 *      - from ROS params
 * - Range Sensor data (ranges)
 *      - from topic
 *      - from ROS params
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */
#ifndef RMCL_CORRECTION_MICP_RANGE_SENSOR_HPP
#define RMCL_CORRECTION_MICP_RANGE_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <variant>
#include <rmcl/util/ros_defines.h>
#include <rmagine/types/sensor_models.h>

#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <rmcl/correction/CorrectionParams.hpp>
#include <rmcl/correction/CorrectionResults.hpp>

#ifdef RMCL_EMBREE
#include <rmagine/map/EmbreeMap.hpp>

#include <rmcl/correction/SphereCorrectorEmbree.hpp>
#include <rmcl/correction/PinholeCorrectorEmbree.hpp>
#include <rmcl/correction/O1DnCorrectorEmbree.hpp>
#include <rmcl/correction/OnDnCorrectorEmbree.hpp>
#endif // RMCL_EMBREE

#ifdef RMCL_CUDA
#include <rmagine/types/MemoryCuda.hpp>
#endif // RMCL_CUDA

#ifdef RMCL_OPTIX
#include <rmagine/map/OptixMap.hpp>

#include <rmcl/correction/SphereCorrectorOptix.hpp>
#include <rmcl/correction/PinholeCorrectorOptix.hpp>
#include <rmcl/correction/O1DnCorrectorOptix.hpp>
#include <rmcl/correction/OnDnCorrectorOptix.hpp>
#endif // RMCL_OPTIX

#include <visualization_msgs/msg/marker.hpp>

// supported sensor data
#include <rmcl_msgs/msg/scan_stamped.hpp>
#include <rmcl_msgs/msg/depth_stamped.hpp>
#include <rmcl_msgs/msg/o1_dn_stamped.hpp>
#include <rmcl_msgs/msg/on_dn_stamped.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>



namespace rmcl
{

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

class MICPRangeSensor 
: std::enable_shared_from_this<MICPRangeSensor>
{
public: // TODO: dont have everything public
    std::string          name;

    TopicInfo            data_topic;
    // optional
    bool                 has_info_topic;
    TopicInfo            info_topic;

    // robots base frame
    std::string          frame;
    std::string          base_frame;
    rmagine::Transform   Tsb;

    // computing backend
    unsigned int         backend = 0;

    // model
    // 0: Spherical, 1: Pinhole, 2: O1Dn, 3: OnDn 
    unsigned int         type;
    SensorModelV         model;
    // model meta
    bool                 model_received_once = false;
    rclcpp::Time         model_last_update;
    float                model_frequency_est; // currently unused
    


    // data    
    rmagine::Memory<float, rmagine::RAM>        ranges;
    #ifdef RMCL_CUDA
    rmagine::Memory<float, rmagine::VRAM_CUDA>  ranges_gpu;
    #endif // RMCL_CUDA

    // data meta
    bool            data_received_once = false;
    rclcpp::Time    data_last_update;
    float           data_frequency_est; // currently unused
    bool            count_valid_ranges = false;
    bool            adaptive_max_dist = false;
    size_t          n_ranges_valid = 0;

    
    
    // subscriber to data
    
    // Main node nh
    rclcpp::Node::SharedPtr nh;
    // Subnodes nh_p, nh_sensor
    // private node. Publisher is publishing on /ns/node_name
    rclcpp::Node::SharedPtr nh_p;
    // private node. Publisher is publishing on /ns/node_name/sensors/sensor_name
    rclcpp::Node::SharedPtr nh_sensor;

    // subscriber to data
    rclcpp::SubscriptionBase::SharedPtr data_sub;
    rclcpp::SubscriptionBase::SharedPtr info_sub;
    

    ImageTransportPtr it;
    ITSubscriberPtr img_sub;
    bool optical_coordinates = false;

    TFBufferPtr  tf_buffer;

    CorrectionParams            corr_params_init;
    CorrectionParams            corr_params;
    float                       adaptive_max_dist_min = 0.15;
    float                       corr_weight = 1.0;

    // DEBUGGING
    bool            viz_corr = false;
    std_msgs::msg::ColorRGBA viz_corr_data_color;
    std_msgs::msg::ColorRGBA viz_corr_model_color;
    float viz_corr_scale = 0.005;
    int viz_corr_skip = 0;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_corr;

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

    // connect to topics
    void connect();

    void fetchMICPParams(bool init = true);

    // called once every new data message
    void fetchTF();
    void updateCorrectors();

    #ifdef RMCL_EMBREE
    void setMap(rmagine::EmbreeMapPtr map);
    #endif // RMCL_EMBREE

    #ifdef RMCL_OPTIX
    void setMap(rmagine::OptixMapPtr map);
    #endif // RMCL_OPTIX

    // do corrections depending on the current sensor state
    void computeCovs(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
        CorrectionPreResults<rmagine::RAM>& res);

    #ifdef RMCL_CUDA
    void computeCovs(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms,
        CorrectionPreResults<rmagine::VRAM_CUDA>& res);
    #endif // RMCL_CUDA

    void enableValidRangesCounting(bool enable = true);

    void enableVizCorrespondences(bool enable = true);

    void countValidRanges();

    void adaptCorrectionParams(float match_ratio, float adaption_rate);

protected:
    // callbacks
    // internal rmcl msgs
    void sphericalCB(
        const rmcl_msgs::msg::ScanStamped::SharedPtr msg);

    void pinholeCB(
        const rmcl_msgs::msg::DepthStamped::SharedPtr msg);

    void o1dnCB(
        const rmcl_msgs::msg::O1DnStamped::SharedPtr msg);

    void ondnCB(
        const rmcl_msgs::msg::OnDnStamped::SharedPtr msg);

    // external commonly used messages
    void pclSphericalCB(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void pclPinholeCB(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void pclO1DnCB(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // void pclOnDnCB(
    //     const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void laserCB(
        const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void imageCB(
        const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    // info callbacks
    // internal
    void sphericalModelCB(
        const rmcl_msgs::msg::ScanInfo::SharedPtr msg);

    void pinholeModelCB(
        const rmcl_msgs::msg::DepthInfo::SharedPtr msg);

    void o1dnModelCB(
        const rmcl_msgs::msg::O1DnInfo::SharedPtr msg);

    void ondnModelCB(
        const rmcl_msgs::msg::OnDnInfo::SharedPtr msg);
    
    // external commonly used
    void cameraInfoCB(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg);
};

using MICPRangeSensorPtr = std::shared_ptr<MICPRangeSensor>;

} // namespace rmcl

#endif // RMCL_CORRECTION_MICP_RANGE_SENSOR_HPP