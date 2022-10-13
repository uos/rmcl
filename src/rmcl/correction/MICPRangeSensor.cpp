#include "rmcl/correction/MICPRangeSensor.hpp"

#include <ros/master.h>
#include <vector>


#include <geometry_msgs/TransformStamped.h>

#include <rmcl/util/conversions.h>

#include <rmcl/math/math.h>
#include <rmcl/math/math.cuh>

#include <rmagine/util/StopWatch.hpp>


#include <rmagine/math/math.cuh>


namespace rm = rmagine;

namespace rmcl
{

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

void MICPRangeSensor::countValidRanges()
{
    n_ranges_valid = 0;

    if(type == 0)
    {
        rm::SphericalModel model_ = std::get<0>(model);
        for(size_t i=0; i<ranges.size(); i++)
        {
            if(model_.range.inside(ranges[i]))
            {
                n_ranges_valid++;
            }
        }
    } else if(type == 1) {
        rm::PinholeModel model_ = std::get<1>(model);
        for(size_t i=0; i<ranges.size(); i++)
        {
            if(model_.range.inside(ranges[i]))
            {
                n_ranges_valid++;
            }
        }
    } else if(type == 2) {
        rm::O1DnModel model_ = std::get<2>(model);
        for(size_t i=0; i<ranges.size(); i++)
        {
            if(model_.range.inside(ranges[i]))
            {
                n_ranges_valid++;
            }
        }
    } else if(type == 3) {
        rm::OnDnModel model_ = std::get<3>(model);
        for(size_t i=0; i<ranges.size(); i++)
        {
            if(model_.range.inside(ranges[i]))
            {
                n_ranges_valid++;
            }
        }
    }
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
}

#ifdef RMCL_CUDA
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
#endif // RMCL_CUDA

void MICPRangeSensor::enableValidRangesCounting(bool enable)
{
    count_valid_ranges = enable;
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
    #ifdef RMCL_CUDA
    ranges_gpu = ranges;
    #endif // RMCL_CUDA

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
    #ifdef RMCL_CUDA
    ranges_gpu = ranges;
    #endif // RMCL_CUDA

    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
    if(count_valid_ranges)
    {
        countValidRanges();
    }
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
    #ifdef RMCL_CUDA
    ranges_gpu = ranges;
    #endif // RMCL_CUDA

    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
    if(count_valid_ranges)
    {
        countValidRanges();
    }
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
    #ifdef RMCL_CUDA
    ranges_gpu = ranges;
    #endif // RMCL_CUDA

    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
    if(count_valid_ranges)
    {
        countValidRanges();
    }
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
    #ifdef RMCL_CUDA
    ranges_gpu = ranges;
    #endif // RMCL_CUDA

    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
    if(count_valid_ranges)
    {
        countValidRanges();
    }
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
    #ifdef RMCL_CUDA
    ranges_gpu = ranges;
    #endif // RMCL_CUDA

    // meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
    if(count_valid_ranges)
    {
        countValidRanges();
    }
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

    #ifdef RMCL_CUDA
    ranges_gpu = ranges;
    #endif // RMCL_CUDA

    // data meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    updateCorrectors();
    if(count_valid_ranges)
    {
        countValidRanges();
    }
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

    #ifdef RMCL_CUDA
    ranges_gpu = ranges;
    #endif // RMCL_CUDA

    // meta
    data_last_update = msg->header.stamp;
    data_received_once = true;

    // update corrector
    updateCorrectors();
    if(count_valid_ranges)
    {
        countValidRanges();
    }
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