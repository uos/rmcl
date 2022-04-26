#ifndef RMCL_LIDAR_CORRECTOR_OPTIX_ROS_HPP
#define RMCL_LIDAR_CORRECTOR_OPTIX_ROS_HPP

#include <rmcl/correction/LiDARCorrectorOptix.hpp>
#include <memory>

namespace rmcl
{

class LiDARCorrectorOptixROS : public LiDARCorrectorOptix
{
public:
    using Base = LiDARCorrectorOptix;

    using Base::Base;
    using Base::setParams;

    using Base::setModel;
    void setModel(const rmcl_msgs::ScanInfo& info);

    using Base::setInputData;
    void setInputData(const std::vector<float>& ranges);

    void setModelAndInputData(const rmcl_msgs::Scan& scan);

    using Base::setTsb;
    void setTsb(const geometry_msgs::Transform& Tsb);

    void setTsb(const std::string& sensor_frame, 
        const std::string& base_frame);

    void setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    using Base::correct;
};

} // namespace rmcl

#endif // RMCL_LIDAR_CORRECTOR_OPTIX_ROS_HPP