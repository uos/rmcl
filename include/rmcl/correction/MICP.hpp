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
 * @brief MICP class for registering range sensors to a mesh in a ROS context
 * 
 * - Read Sensor configuration from ROS parameters
 * - Subscribe to Sensor Data
 * - Get a combined correction in base coordinates
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMCL_CORRECTION_MICP_HPP
#define RMCL_CORRECTION_MICP_HPP

#include <ros/ros.h>
#include <rmagine/types/sensor_models.h>
#include <memory>
#include <unordered_map>

// rmcl core
#include <rmcl/math/math.h>

// rmcl cuda
#ifdef RMCL_CUDA
#include <rmcl/math/math.cuh>
#endif // RMCL_CUDA

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
#include <image_transport/image_transport.h>

#include "MICPRangeSensor.hpp"

namespace rmcl
{

/**
 * @brief 
 * 
 */
class MICP
{
public:
    MICP();
    ~MICP();

    void loadParams();

    bool loadSensor(std::string sensor_name, 
        XmlRpc::XmlRpcValue sensor_params);

    void loadMap(std::string filename);
    
    #ifdef RMCL_EMBREE
    void setMap(rmagine::EmbreeMapPtr map);
    #endif // RMCL_EMBREE

    #ifdef RMCL_OPTIX
    void setMap(rmagine::OptixMapPtr map);
    #endif // RMCL_OPTIX

    #ifdef RMCL_CUDA
    void correct(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbm,
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm_,
        CorrectionPreResults<rmagine::VRAM_CUDA>& pre_res,
        rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& dT);
    #endif // RMCL_CUDA

    #ifdef RMCL_CUDA
    void correct(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbm,
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm_,
        CorrectionPreResults<rmagine::RAM>& pre_res,
        rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& dT);
    #endif // RMCL_CUDA

    #ifdef RMCL_CUDA
    void correct(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm,
        CorrectionPreResults<rmagine::VRAM_CUDA>& pre_res,
        rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& dT);
    #endif // RMCL_CUDA

    void correct(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbm,
        CorrectionPreResults<rmagine::RAM>& pre_res,
        rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& dT);

    inline std::unordered_map<std::string, MICPRangeSensorPtr> sensors()
    {
        return m_sensors;
    }

    void useInThisThread()
    {
        #ifdef RMCL_OPTIX
        // TODO hold a context ptr at MICP object
        if(m_map_optix)
        {
            auto cuda_ctx = m_map_optix->scene()->stream()->context();
            if(!cuda_ctx->isActive())
            {
                std::cout << "MICP - reactivating CUDA for thread" << std::endl;
                cuda_ctx->use();
            }
        }
        #endif // RMCL_OPTIX
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


    // correctors: initialized once the map is available
    CorrectionPtr m_corr_cpu;
    #ifdef RMCL_CUDA
    CorrectionCudaPtr m_corr_gpu;
    #endif // RMCL_CUDA
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

#endif // RMCL_CORRECTION_MICP_HPP