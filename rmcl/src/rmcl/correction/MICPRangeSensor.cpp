#include "rmcl/correction/MICPRangeSensor.hpp"

#include <ros/master.h>
#include <vector>


#include <geometry_msgs/TransformStamped.h>

#include <rmcl/util/conversions.h>

#include <rmcl/math/math.h>
#include <rmcl/math/math_batched.h>

#ifdef RMCL_CUDA
#include <rmcl/math/math.cuh>
#include <rmcl/math/math_batched.cuh>
#endif // RMCL_CUDA

#include <rmagine/util/StopWatch.hpp>


#include <rmagine/util/prints.h>

#include <visualization_msgs/Marker.h>



namespace rm = rmagine;

namespace rmcl
{

visualization_msgs::Marker make_marker(
    rm::MemoryView<rm::Point, rm::RAM> dataset_points,
    rm::MemoryView<rm::Point, rm::RAM> model_points,
    rm::MemoryView<unsigned int, rm::RAM> corr_valid,
    rm::Transform Tbm,
    std_msgs::ColorRGBA dcol,
    std_msgs::ColorRGBA mcol,
    float scale,
    unsigned int step)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;


    for(size_t j=0; j<dataset_points.size(); j += step)
    {
        if(corr_valid[j] > 0)
        {
            // transform from base coords to map coords
            rm::Point d = Tbm * dataset_points[j];
            rm::Point m = Tbm * model_points[j];
            
            geometry_msgs::Point dros;
            geometry_msgs::Point mros;

            dros.x = d.x;
            dros.y = d.y;
            dros.z = d.z;

            mros.x = m.x;
            mros.y = m.y;
            mros.z = m.z;

            marker.points.push_back(dros);
            marker.points.push_back(mros);

            marker.colors.push_back(dcol);
            marker.colors.push_back(mcol);
        }
    }

    return marker;
}

#ifdef RMCL_CUDA
visualization_msgs::Marker make_marker(
    rm::MemoryView<rm::Point, rm::VRAM_CUDA> dataset_points,
    rm::MemoryView<rm::Point, rm::VRAM_CUDA> model_points,
    rm::MemoryView<unsigned int, rm::VRAM_CUDA> corr_valid,
    rm::MemoryView<rm::Transform, rm::VRAM_CUDA> Tbm,
    std_msgs::ColorRGBA dcol,
    std_msgs::ColorRGBA mcol,
    float scale,
    unsigned int step)
{
    rm::Memory<rm::Point, rm::RAM> dataset_points_ = dataset_points;
    rm::Memory<rm::Point, rm::RAM> model_points_ = model_points;
    rm::Memory<unsigned int, rm::RAM> corr_valid_ = corr_valid;
    rm::Memory<rm::Transform, rm::RAM> Tbm_ = Tbm;

    return make_marker(dataset_points_, model_points_, corr_valid_, Tbm_[0], 
        dcol, mcol, scale, step);
}
#endif // RMCL_CUDA

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
        } else if(data_topic.msg == "sensor_msgs/PointCloud2") {
            data_sub = std::make_shared<ros::Subscriber>(
                    nh->subscribe<sensor_msgs::PointCloud2>(
                        data_topic.name, 1, 
                        &MICPRangeSensor::pclO1DnCB, this
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

void MICPRangeSensor::fetchMICPParams(bool init)
{
    if(!nh_sensor)
    {
        std::stringstream ss;
        ss << "sensors/" << name;
        nh_sensor = std::make_shared<ros::NodeHandle>(
            *nh_p, ss.str()
        );
    }

    // local settings
    if(!nh_sensor->getParam("micp/max_dist", corr_params_init.max_distance))
    {
        if(!nh_p->getParam("micp/max_dist", corr_params_init.max_distance))
        {
            corr_params_init.max_distance = 1.0;
        }
    }

    if(init)
    {
        corr_params = corr_params_init;
    }

    bool adaptive_max_dist;
    if(!nh_sensor->getParam("micp/adaptive_max_dist", adaptive_max_dist))
    {
        if(!nh_p->getParam("micp/adaptive_max_dist", adaptive_max_dist))
        {
            adaptive_max_dist = false;
        }
    }

    if(adaptive_max_dist)
    {
        enableValidRangesCounting(true);
    }
    

    std::string backend_str;
    if(!nh_sensor->getParam("micp/backend", backend_str))
    {
        backend_str = "embree";
    }

    if(backend_str == "embree")
    {
        backend = 0;
    } else if(backend_str == "optix") {
        backend = 1;
    }

    if(!nh_sensor->getParam("micp/weight", corr_weight))
    {
        corr_weight = 1.0;
    }

    // VIZ
    if(!nh_sensor->getParam("micp/viz_corr", viz_corr))
    {
        if(!nh_p->getParam("micp/viz_corr", viz_corr))
        {
            viz_corr = false;
        }
    }

    enableVizCorrespondences(viz_corr);

    { // viz cor dataset color
        std::vector<double> colors;
        if(!nh_sensor->getParam("micp/viz_corr_data_color", colors))
        {
            if(!nh_p->getParam("micp/viz_corr_data_color", colors))
            {
                colors = {1.0, 1.0, 1.0, 0.5};
            }
        }

        viz_corr_data_color.r = colors[0];
        viz_corr_data_color.g = colors[1];
        viz_corr_data_color.b = colors[2];
        viz_corr_data_color.a = colors[3];
    }

    { // viz cor model color
        std::vector<double> colors;
        if(!nh_sensor->getParam("micp/viz_corr_model_color", colors))
        {
            if(!nh_p->getParam("micp/viz_corr_model_color", colors))
            {
                colors = {0.2, 0.2, 0.2, 1.0};
            }
        }

        viz_corr_model_color.r = colors[0];
        viz_corr_model_color.g = colors[1];
        viz_corr_model_color.b = colors[2];
        viz_corr_model_color.a = colors[3];
    }
    
    if(!nh_sensor->getParam("micp/viz_corr_scale", viz_corr_scale))
    {
        if(!nh_p->getParam("micp/viz_corr_scale", viz_corr_scale))
        {
            viz_corr_scale = 0.008;
        }
    }

    if(!nh_sensor->getParam("micp/viz_corr_skip", viz_corr_scale))
    {
        if(!nh_p->getParam("micp/viz_corr_scale", viz_corr_scale))
        {
            viz_corr_scale = 0.008;
        }
    }

    if(!nh_sensor->getParam("micp/viz_corr_skip", viz_corr_skip))
    {
        if(!nh_p->getParam("micp/viz_corr_skip", viz_corr_skip))
        {
            viz_corr_skip = 0;
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
        corr_pinhole_embree->setOptical(optical_coordinates);
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
        corr_pinhole_optix->setOptical(optical_coordinates);
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

#ifdef RMCL_EMBREE
void MICPRangeSensor::setMap(rmagine::EmbreeMapPtr map)
{
    if(corr_sphere_embree)
    {
        corr_sphere_embree->setMap(map);
    } else if(corr_pinhole_embree) {
        corr_pinhole_embree->setMap(map);
    } else if(corr_o1dn_embree) {
        corr_o1dn_embree->setMap(map);
    } else if(corr_ondn_embree) {
        corr_ondn_embree->setMap(map);
    }
}
#endif // RMCL_EMBREE

#ifdef RMCL_OPTIX
void MICPRangeSensor::setMap(rmagine::OptixMapPtr map)
{
    if(corr_sphere_optix)
    {
        corr_sphere_optix->setMap(map);
    } else if(corr_pinhole_optix) {
        corr_pinhole_optix->setMap(map);
    } else if(corr_o1dn_optix) {
        corr_o1dn_optix->setMap(map);
    } else if(corr_ondn_optix) {
        corr_ondn_optix->setMap(map);
    }
}
#endif // RMCL_OPTIX

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

void MICPRangeSensor::adaptCorrectionParams(
    float match_ratio, 
    float adaption_rate)
{
    corr_params.max_distance = corr_params_init.max_distance + (adaptive_max_dist_min - corr_params_init.max_distance) * adaption_rate;
    // std::cout << "setting max corr distance to " << corr_params.max_distance << std::endl;
}

void MICPRangeSensor::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    CorrectionPreResults<rmagine::RAM>& res)
{
    // this is bad. maybe we must go away from having a completely generic sensor

    #ifdef RMCL_EMBREE
    if(backend == 0)
    {
        if(type == 0) 
        {
            if(viz_corr)
            {
                // draw correspondences of first pose
                auto Tbms0 = Tbms(0, 0+1);

                rm::Memory<rm::Point, rm::RAM> dataset_points;
                rm::Memory<rm::Point, rm::RAM> model_points;
                rm::Memory<unsigned int, rm::RAM> corr_valid;

                corr_sphere_embree->findSPC(Tbms0, 
                    dataset_points, model_points, corr_valid);

                auto marker = make_marker(
                    dataset_points, model_points, 
                    corr_valid, Tbms0[0], 
                    viz_corr_data_color, viz_corr_model_color,
                    viz_corr_scale, viz_corr_skip + 1);
               
                marker.header.stamp = ros::Time::now();
                if(pub_corr)
                {
                    pub_corr->publish(marker);
                }

                res.ds.resize(Tbms.size());
                res.ms.resize(Tbms.size());
                res.Cs.resize(Tbms.size());
                res.Ncorr.resize(Tbms.size());

                means_covs_online_batched(
                    dataset_points, model_points, corr_valid, // input
                    res.ds, res.ms, // outputs
                    res.Cs, res.Ncorr
                );

                // std::cout << "Valid Correspondences: " << res.Ncorr[0] << std::endl;
            } else {
                corr_sphere_embree->computeCovs(Tbms, res);
            }

            
        } else if(type == 1) {
            if(viz_corr)
            {
                // draw correspondences of first pose
                // only allowed for one pose
                auto Tbms0 = Tbms(0, 0+1);

                rm::Memory<rm::Point, rm::RAM> dataset_points;
                rm::Memory<rm::Point, rm::RAM> model_points;
                rm::Memory<unsigned int, rm::RAM> corr_valid;

                corr_pinhole_embree->findSPC(Tbms0, 
                    dataset_points, model_points, corr_valid);

                auto marker = make_marker(
                    dataset_points, model_points, 
                    corr_valid, Tbms0[0], 
                    viz_corr_data_color, viz_corr_model_color,
                    viz_corr_scale, viz_corr_skip + 1);
                
                marker.header.stamp = ros::Time::now();
                if(pub_corr)
                {
                    pub_corr->publish(marker);
                }

                res.ds.resize(Tbms.size());
                res.ms.resize(Tbms.size());
                res.Cs.resize(Tbms.size());
                res.Ncorr.resize(Tbms.size());

                means_covs_online_batched(
                    dataset_points, model_points, corr_valid, // input
                    res.ds, res.ms, // outputs
                    res.Cs, res.Ncorr
                );
            } else {
                corr_pinhole_embree->computeCovs(Tbms, res);
            }
            
        } else if(type == 2) {
            
            if(viz_corr)
            {
                // draw correspondences of first pose
                auto Tbms0 = Tbms(0, 0+1);

                rm::Memory<rm::Point, rm::RAM> dataset_points;
                rm::Memory<rm::Point, rm::RAM> model_points;
                rm::Memory<unsigned int, rm::RAM> corr_valid;

                corr_o1dn_embree->findSPC(Tbms0, 
                    dataset_points, model_points, corr_valid);

                auto marker = make_marker(
                    dataset_points, model_points, 
                    corr_valid, Tbms0[0], 
                    viz_corr_data_color, viz_corr_model_color,
                    viz_corr_scale, viz_corr_skip + 1);
                
                marker.header.stamp = ros::Time::now();
                if(pub_corr)
                {
                    pub_corr->publish(marker);
                }

                res.ds.resize(Tbms.size());
                res.ms.resize(Tbms.size());
                res.Cs.resize(Tbms.size());
                res.Ncorr.resize(Tbms.size());

                means_covs_online_batched(
                    dataset_points, model_points, corr_valid, // input
                    res.ds, res.ms, // outputs
                    res.Cs, res.Ncorr
                );
            } else {
                corr_o1dn_embree->computeCovs(Tbms, res);
            }

            
        } else if(type == 3) {

            if(viz_corr)
            {
                // draw correspondences of first pose
                auto Tbms0 = Tbms(0, 0+1);

                rm::Memory<rm::Point, rm::RAM> dataset_points;
                rm::Memory<rm::Point, rm::RAM> model_points;
                rm::Memory<unsigned int, rm::RAM> corr_valid;

                corr_ondn_embree->findSPC(Tbms0, 
                    dataset_points, model_points, corr_valid);

                auto marker = make_marker(
                    dataset_points, model_points, 
                    corr_valid, Tbms0[0], 
                    viz_corr_data_color, viz_corr_model_color,
                    viz_corr_scale, viz_corr_skip + 1);
                
                marker.header.stamp = ros::Time::now();
                if(pub_corr)
                {
                    pub_corr->publish(marker);
                }

                res.ds.resize(Tbms.size());
                res.ms.resize(Tbms.size());
                res.Cs.resize(Tbms.size());
                res.Ncorr.resize(Tbms.size());

                means_covs_online_batched(
                    dataset_points, model_points, corr_valid, // input
                    res.ds, res.ms, // outputs
                    res.Cs, res.Ncorr
                );
            } else {
                corr_ondn_embree->computeCovs(Tbms, res);
            }
        }
    }
    #endif // RMCL_EMBREE
    
    #ifdef RMCL_OPTIX
    if(backend == 1)
    {
        // upload
        rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbms_ = Tbms;
        CorrectionPreResults<rm::VRAM_CUDA> res_;
        res_.ds.resize(Tbms.size());
        res_.ms.resize(Tbms.size());
        res_.Cs.resize(Tbms.size());
        res_.Ncorr.resize(Tbms.size());

        // compute
        if(type == 0) {
            corr_sphere_optix->computeCovs(Tbms_, res_);
        } else if(type == 1) {
            corr_pinhole_optix->computeCovs(Tbms_, res_);   
        } else if(type == 2) {
            corr_o1dn_optix->computeCovs(Tbms_, res_);
        } else if(type == 3) {
            corr_ondn_optix->computeCovs(Tbms_, res_);
        }

        // download
        res.ds = res_.ds;
        res.ms = res_.ms;
        res.Cs = res_.Cs;
        res.Ncorr = res_.Ncorr;
    }
    #endif // RMCL_OPTIX
}

#ifdef RMCL_CUDA
void MICPRangeSensor::computeCovs(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbms,
    CorrectionPreResults<rm::VRAM_CUDA>& res)
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
            corr_sphere_embree->computeCovs(Tbms_, res_);
        } else if(type == 1) {
            corr_pinhole_embree->computeCovs(Tbms_, res_);   
        } else if(type == 2) {
            corr_o1dn_embree->computeCovs(Tbms_, res_);
        } else if(type == 3) {
            corr_ondn_embree->computeCovs(Tbms_, res_);
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
            // std::cout << "SPHERE GPU" << std::endl;

            if(viz_corr)
            {
                // draw correspondences of first pose
                auto Tbms0 = Tbms(0, 0+1);

                rm::Memory<rm::Point, rm::VRAM_CUDA> dataset_points;
                rm::Memory<rm::Point, rm::VRAM_CUDA> model_points;
                rm::Memory<unsigned int, rm::VRAM_CUDA> corr_valid;

                corr_sphere_optix->findSPC(Tbms0, 
                    dataset_points, model_points, corr_valid);

                auto marker = make_marker(
                    dataset_points, model_points, 
                    corr_valid, Tbms0,
                    viz_corr_data_color, viz_corr_model_color,
                    viz_corr_scale, viz_corr_skip + 1);
                
                marker.header.stamp = ros::Time::now();
                if(pub_corr)
                {
                    pub_corr->publish(marker);
                }

                res.ds.resize(Tbms.size());
                res.ms.resize(Tbms.size());
                res.Cs.resize(Tbms.size());
                res.Ncorr.resize(Tbms.size());

                means_covs_online_batched(
                    dataset_points, model_points, corr_valid, // input
                    res.ds, res.ms, // outputs
                    res.Cs, res.Ncorr
                );
            } else {
                corr_sphere_optix->computeCovs(Tbms, res);
            }
            
        } else if(type == 1) {
            
            if(viz_corr)
            {
                // draw correspondences of first pose
                auto Tbms0 = Tbms(0, 0+1);

                rm::Memory<rm::Point, rm::VRAM_CUDA> dataset_points;
                rm::Memory<rm::Point, rm::VRAM_CUDA> model_points;
                rm::Memory<unsigned int, rm::VRAM_CUDA> corr_valid;

                corr_pinhole_optix->findSPC(Tbms0, 
                    dataset_points, model_points, corr_valid);

                auto marker = make_marker(
                    dataset_points, model_points, 
                    corr_valid, Tbms0,
                    viz_corr_data_color, viz_corr_model_color,
                    viz_corr_scale, viz_corr_skip + 1);
                
                marker.header.stamp = ros::Time::now();
                if(pub_corr)
                {
                    pub_corr->publish(marker);
                }
            }

            corr_pinhole_optix->computeCovs(Tbms, res);   
        } else if(type == 2) {

            if(viz_corr)
            {
                // draw correspondences of first pose
                auto Tbms0 = Tbms(0, 0+1);

                rm::Memory<rm::Point, rm::VRAM_CUDA> dataset_points;
                rm::Memory<rm::Point, rm::VRAM_CUDA> model_points;
                rm::Memory<unsigned int, rm::VRAM_CUDA> corr_valid;

                corr_o1dn_optix->findSPC(Tbms0, 
                    dataset_points, model_points, corr_valid);

                auto marker = make_marker(
                    dataset_points, model_points, 
                    corr_valid, Tbms0,
                    viz_corr_data_color, viz_corr_model_color,
                    viz_corr_scale, viz_corr_skip + 1);
                
                marker.header.stamp = ros::Time::now();
                if(pub_corr)
                {
                    pub_corr->publish(marker);
                }
            }

            corr_o1dn_optix->computeCovs(Tbms, res);
        } else if(type == 3) {

            if(viz_corr)
            {
                // draw correspondences of first pose
                auto Tbms0 = Tbms(0, 0+1);

                rm::Memory<rm::Point, rm::VRAM_CUDA> dataset_points;
                rm::Memory<rm::Point, rm::VRAM_CUDA> model_points;
                rm::Memory<unsigned int, rm::VRAM_CUDA> corr_valid;

                corr_ondn_optix->findSPC(Tbms0, 
                    dataset_points, model_points, corr_valid);

                auto marker = make_marker(
                    dataset_points, model_points, 
                    corr_valid, Tbms0,
                    viz_corr_data_color, viz_corr_model_color,
                    viz_corr_scale, viz_corr_skip + 1);
                
                marker.header.stamp = ros::Time::now();
                if(pub_corr)
                {
                    pub_corr->publish(marker);
                }
            }

            corr_ondn_optix->computeCovs(Tbms, res);
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

void MICPRangeSensor::enableVizCorrespondences(bool enable)
{
    viz_corr = enable;

    if(viz_corr)
    {
        if(!nh_sensor)
        {
            std::stringstream ss;
            ss << "sensors/" << name;
            nh_sensor = std::make_shared<ros::NodeHandle>(
                *nh_p, ss.str()
            );
        }

        pub_corr = std::make_shared<ros::Publisher>(
            nh_sensor->advertise<visualization_msgs::Marker>("correspondences", 1)
        );
    }
}

void MICPRangeSensor::sphericalCB(
    const rmcl_msgs::ScanStamped::ConstPtr& msg)
{
    fetchTF();

    // model
    rm::SphericalModel model_;
    convert(msg->scan.info, model_);
    model = model_;

    // data
    if(ranges.size() < msg->scan.data.ranges.size())
    {
        ranges.resize(msg->scan.data.ranges.size());
    }
    std::copy(msg->scan.data.ranges.begin(), msg->scan.data.ranges.end(), ranges.raw());
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
    if(ranges.size() < msg->depth.data.ranges.size())
    {
        ranges.resize(msg->depth.data.ranges.size());
    }
    std::copy(msg->depth.data.ranges.begin(), msg->depth.data.ranges.end(), ranges.raw());
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
    if(ranges.size() < msg->o1dn.data.ranges.size())
    {
        ranges.resize(msg->o1dn.data.ranges.size());
    }
    std::copy(msg->o1dn.data.ranges.begin(), msg->o1dn.data.ranges.end(), ranges.raw());
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
    if(ranges.size() < msg->ondn.data.ranges.size())
    {
        ranges.resize(msg->ondn.data.ranges.size());
    }
    std::copy(msg->ondn.data.ranges.begin(), msg->ondn.data.ranges.end(), ranges.raw());
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

    rm::Transform T = rm::Transform::Identity();

    if(frame != msg->header.frame_id)
    {
        try {
            auto Tros = tf_buffer->lookupTransform(frame, msg->header.frame_id,
                               ros::Time(0));
            convert(Tros.transform, T);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

    
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
        
        rm::Point p;

        if(field_x.datatype == sensor_msgs::PointField::FLOAT32)
        {
            // Float
            p.x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
            p.y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
            p.z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
        } else if(field_x.datatype == sensor_msgs::PointField::FLOAT64) {
            // Double
            p.x = *reinterpret_cast<const double*>(data_ptr + field_x.offset);
            p.y = *reinterpret_cast<const double*>(data_ptr + field_y.offset);
            p.z = *reinterpret_cast<const double*>(data_ptr + field_z.offset);
        } else {
            throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
        }

        if(!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
        {
            p = T * p;

            float range_est = p.l2norm();
            float theta_est = atan2(p.y, p.x);
            float phi_est = atan2(p.z, range_est);
            
            int phi_id = ((phi_est - model_.phi.min) / model_.phi.inc) + 0.5;
            int theta_id = ((theta_est - model_.theta.min) / model_.theta.inc) + 0.5;
            
            if(phi_id >= 0 && phi_id < model_.phi.size
                && theta_id >= 0 && theta_id < model_.theta.size)
            {
                unsigned int p_id = model_.getBufferId(phi_id, theta_id);
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

    rm::Transform T = rm::Transform::Identity();

    if(frame != msg->header.frame_id)
    {
        try {
            auto Tros = tf_buffer->lookupTransform(frame, msg->header.frame_id,
                               ros::Time(0));
            convert(Tros.transform, T);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

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

        rm::Vector p;

        if(field_x.datatype == sensor_msgs::PointField::FLOAT32)
        {
            // Float
            p.x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
            p.y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
            p.z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
        } else if(field_x.datatype == sensor_msgs::PointField::FLOAT64) {
            // Double
            p.x = *reinterpret_cast<const double*>(data_ptr + field_x.offset);
            p.y = *reinterpret_cast<const double*>(data_ptr + field_y.offset);
            p.z = *reinterpret_cast<const double*>(data_ptr + field_z.offset);
        } else {
            throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
        }

        

        // transform point if required
        if(!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
        {
            p = T * p;
            float range_est = p.l2norm();
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

void MICPRangeSensor::pclO1DnCB(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
    fetchTF();

    rm::Transform T = rm::Transform::Identity();

    if(frame != msg->header.frame_id)
    {
        try {
            auto Tros = tf_buffer->lookupTransform(frame, msg->header.frame_id,
                               ros::Time(0));
            convert(Tros.transform, T);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }
    
    auto model_ = std::get<2>(model);

    if(ranges.size() < msg->width * msg->height)
    {
        ranges.resize(msg->width * msg->height);
    }
    
    // I'm no sure here:
    // Either
    // We let the data define the sensor model
    // or we trust the existing sensor model
    // TODO: make a decision and a proper documentation
    model_.orig   = T.t;
    model_.width  = msg->width;
    model_.height = msg->height;
    if(model_.dirs.size() < msg->width * msg->height)
    {
        model_.dirs.resize(msg->width * msg->height);
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

        rm::Point p;

        if(field_x.datatype == sensor_msgs::PointField::FLOAT32)
        {
            // Float
            p.x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
            p.y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
            p.z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
        } else if(field_x.datatype == sensor_msgs::PointField::FLOAT64) {
            // Double
            p.x = *reinterpret_cast<const double*>(data_ptr + field_x.offset);
            p.y = *reinterpret_cast<const double*>(data_ptr + field_y.offset);
            p.z = *reinterpret_cast<const double*>(data_ptr + field_z.offset);
        } else {
            throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
        }

        if(!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
        {
            // transform to actual sensor frame
            p = T * p;
            // O1Dn model can have a move sensor origin.
            // that means the ray goes from this origin to the target p. so we have to subtract:
            p = p - model_.orig;
            // set range and dir
            ranges[i] = p.l2norm();
            model_.dirs[i] = p.normalize();
            // so that the following equation is satisfied:
            // p = range * dir + orig
        } else {
            ranges[i] = model_.range.max + 1.0;
            model_.dirs[i].x = 1.0;
            model_.dirs[i].y = 0.0;
            model_.dirs[i].z = 0.0;
        }
    }

    model = model_;

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
