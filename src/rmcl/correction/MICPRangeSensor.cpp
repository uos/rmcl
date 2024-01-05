#include "rmcl/correction/MICPRangeSensor.hpp"

// how to port this?
// #include <ros/master.h>
#include <vector>


#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rmcl/util/conversions.h>

#include <rmcl/math/math.h>
#include <rmcl/math/math_batched.h>

#ifdef RMCL_CUDA
#include <rmcl/math/math.cuh>
#include <rmcl/math/math_batched.cuh>
#endif // RMCL_CUDA

#include <rmagine/util/StopWatch.hpp>


#include <rmagine/util/prints.h>

#include <visualization_msgs/msg/marker.hpp>



namespace rm = rmagine;

namespace rmcl
{

visualization_msgs::msg::Marker make_marker(
    rm::MemoryView<rm::Point, rm::RAM> dataset_points,
    rm::MemoryView<rm::Point, rm::RAM> model_points,
    rm::MemoryView<unsigned int, rm::RAM> corr_valid,
    rm::Transform Tbm,
    std_msgs::msg::ColorRGBA dcol,
    std_msgs::msg::ColorRGBA mcol,
    float scale,
    unsigned int step)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;


    for(size_t i=0; i<dataset_points.size(); i += step)
    {
        if(corr_valid[i] > 0)
        {
            // transform from base coords to map coords
            rm::Point d = Tbm * dataset_points[i];
            rm::Point m = Tbm * model_points[i];
            
            geometry_msgs::msg::Point dros;
            geometry_msgs::msg::Point mros;

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
visualization_msgs::msg::Marker make_marker(
    rm::MemoryView<rm::Point, rm::VRAM_CUDA> dataset_points,
    rm::MemoryView<rm::Point, rm::VRAM_CUDA> model_points,
    rm::MemoryView<unsigned int, rm::VRAM_CUDA> corr_valid,
    rm::MemoryView<rm::Transform, rm::VRAM_CUDA> Tbm,
    std_msgs::msg::ColorRGBA dcol,
    std_msgs::msg::ColorRGBA mcol,
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
        if(data_topic.msg == "rmcl_msgs/msg/ScanStamped") {
            data_sub = nh->create_subscription<rmcl_msgs::msg::ScanStamped>(
                    data_topic.name, 1, 
                    std::bind(&MICPRangeSensor::sphericalCB, this, std::placeholders::_1)
                );

        } else if(data_topic.msg == "sensor_msgs/msg/PointCloud2") {
            data_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
                    data_topic.name, 1, 
                    std::bind(&MICPRangeSensor::pclSphericalCB, this, std::placeholders::_1)
                );
        } else if(data_topic.msg == "sensor_msgs/msg/LaserScan") {
            data_sub = nh->create_subscription<sensor_msgs::msg::LaserScan>(
                    data_topic.name, 1, 
                    std::bind(&MICPRangeSensor::laserCB, this, std::placeholders::_1)
                );
        } else {
            // TODO proper error msg
            std::cout << data_topic.msg << " message unknown for sensor type " << type << std::endl;
        }
    } else if(type == 1) { // Pinhole
        
        if(data_topic.msg == "rmcl_msgs/msg/DepthStamped") {
            data_sub = nh->create_subscription<rmcl_msgs::msg::DepthStamped>(
                    data_topic.name, 1,
                    std::bind(&MICPRangeSensor::pinholeCB, this, std::placeholders::_1)
                );
        } else if(data_topic.msg == "sensor_msgs/msg/PointCloud2") {
            data_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
                    data_topic.name, 1, 
                    std::bind(&MICPRangeSensor::pclPinholeCB, this, std::placeholders::_1)
                );
        } else if(data_topic.msg == "sensor_msgs/msg/Image") {
            // std::cout << "Connecting to depth image" << std::endl;
            // create image transport if not yet done

            if(!it)
            {
                it = std::make_shared<image_transport::ImageTransport>(nh);
            }

            img_sub = std::make_shared<image_transport::Subscriber>(
                    it->subscribe(
                        data_topic.name, 1,
                        std::bind(&MICPRangeSensor::imageCB, this, 
                            std::placeholders::_1)
                    )
                );
        }

    } else if(type == 2) { // O1Dn
        if(data_topic.msg == "rmcl_msgs/msg/O1DnStamped") {
            data_sub = nh->create_subscription<rmcl_msgs::msg::O1DnStamped>(
                    data_topic.name, 1, 
                    std::bind(&MICPRangeSensor::o1dnCB, this, std::placeholders::_1)
                );
        } else if(data_topic.msg == "sensor_msgs/msg/PointCloud2") {
            data_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
                data_topic.name, 1, 
                std::bind(&MICPRangeSensor::pclO1DnCB, this, std::placeholders::_1)
            );
        }
    } else if(type == 3) { // OnDn
        if(data_topic.msg == "rmcl_msgs/msg/OnDnStamped") {
            data_sub = nh->create_subscription<rmcl_msgs::msg::OnDnStamped>(
                    data_topic.name, 1, 
                    std::bind(&MICPRangeSensor::ondnCB, this, std::placeholders::_1)
                );
        }
    }

    if(has_info_topic)
    {
        // connect to info topic
        if(info_topic.msg == "sensor_msgs/msg/CameraInfo")
        {
            info_sub = nh->create_subscription<sensor_msgs::msg::CameraInfo>(
                    info_topic.name, 1, 
                    std::bind(&MICPRangeSensor::cameraInfoCB, this, std::placeholders::_1)
                );
        } else if(info_topic.msg == "rmcl_msgs/msg/ScanInfo") {
            info_sub = nh->create_subscription<rmcl_msgs::msg::ScanInfo>(
                    info_topic.name, 1, 
                    std::bind(&MICPRangeSensor::sphericalModelCB, this, std::placeholders::_1)
                );
        } else if(info_topic.msg == "rmcl_msgs/msg/DepthInfo") {
            info_sub = nh->create_subscription<rmcl_msgs::msg::DepthInfo>(
                    info_topic.name, 1, 
                    std::bind(&MICPRangeSensor::pinholeModelCB, this, std::placeholders::_1)
                );
        } else if(info_topic.msg == "rmcl_msgs/msg/O1DnInfo") {
            info_sub = nh->create_subscription<rmcl_msgs::msg::O1DnInfo>(
                    info_topic.name, 1, 
                    std::bind(&MICPRangeSensor::o1dnModelCB, this, std::placeholders::_1)
                );
        } else if(info_topic.msg == "rmcl_msgs/msg/OnDnInfo") {
            info_sub = nh->create_subscription<rmcl_msgs::msg::OnDnInfo>(
                    info_topic.name, 1, 
                    std::bind(&MICPRangeSensor::ondnModelCB, this, std::placeholders::_1)
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
        std::string node_ns;
        {
            std::stringstream ss;
            ss << "sensors/" << name;
            node_ns = ss.str();
        }

        if(nh)
        {
            nh_sensor = nh_p->create_sub_node(node_ns);
        } else {
            // ERROR
            std::cerr << "ERROR: MICPRangeSensor has no MICP node." << std::endl;
            throw std::runtime_error("MICPRangeSensor has no MICP node.");
        }
    }

    std::string micp_prefix_local;
    {
        std::stringstream ss;
        ss << "sensors." << name << ".micp";
        micp_prefix_local = ss.str();
    }
    std::string micp_prefix_global = "micp";

    std::map<std::string, rclcpp::Parameter> micp_params_local;
    nh_sensor->get_parameters(micp_prefix_local, micp_params_local);

    std::map<std::string, rclcpp::Parameter> micp_params_global;
    nh->get_parameters(micp_prefix_global, micp_params_global);

    // local settings
    if(micp_params_local.find("max_dist") != micp_params_local.end())
    {
        corr_params_init.max_distance = micp_params_local.at("max_dist").as_double();
    } else if(micp_params_global.find("max_dist") != micp_params_global.end()) {
        corr_params_init.max_distance = micp_params_global.at("max_dist").as_double();
    } else {
        corr_params_init.max_distance = 1.0;
    }

    if(init)
    {
        corr_params = corr_params_init;
    }

    bool adaptive_max_dist;
    
    if(micp_params_local.find("adaptive_max_dist") != micp_params_local.end())
    {
        adaptive_max_dist = micp_params_local.at("adaptive_max_dist").as_bool();
    } else if(micp_params_global.find("adaptive_max_dist") != micp_params_global.end()) {
        adaptive_max_dist = micp_params_global.at("adaptive_max_dist").as_bool();
    } else {
        adaptive_max_dist = false;
    }

    if(adaptive_max_dist)
    {
        enableValidRangesCounting(true);
    }
    

    std::string backend_str;
    if(micp_params_local.find("backend") != micp_params_local.end())
    {
        backend_str = micp_params_local.at("backend").as_string();
    } else {
        backend_str = "embree";
    }

    if(backend_str == "embree")
    {
        backend = 0;
    } else if(backend_str == "optix") {
        backend = 1;
    }

    if(micp_params_local.find("weight") != micp_params_local.end())
    {
        corr_weight = micp_params_local.at("weight").as_double();
    } else {
        corr_weight = 1.0;
    }

    // VIZ
    if(micp_params_local.find("viz_corr") != micp_params_local.end())
    {
        viz_corr = micp_params_local.at("viz_corr").as_bool();
    } else if(micp_params_global.find("viz_corr") != micp_params_global.end()) {
        viz_corr = micp_params_global.at("viz_corr").as_bool();
    } else {
        viz_corr = false;
    }

    enableVizCorrespondences(viz_corr);

    { // viz cor dataset color
        std::vector<double> colors;
        if(micp_params_local.find("viz_corr_data_color") != micp_params_local.end())
        {
            colors = micp_params_local.at("viz_corr_data_color").as_double_array();
        } else if(micp_params_global.find("viz_corr_data_color") != micp_params_global.end()) {
            colors = micp_params_global.at("viz_corr_data_color").as_double_array();
        } else {
            colors = {1.0, 1.0, 1.0, 0.5};
        }

        viz_corr_data_color.r = colors[0];
        viz_corr_data_color.g = colors[1];
        viz_corr_data_color.b = colors[2];
        viz_corr_data_color.a = colors[3];
    }

    { // viz cor model color
        std::vector<double> colors;
        if(micp_params_local.find("viz_corr_model_color") != micp_params_local.end())
        {
            colors = micp_params_local.at("viz_corr_model_color").as_double_array();
        } else if(micp_params_global.find("viz_corr_model_color") != micp_params_global.end()) {
            colors = micp_params_global.at("viz_corr_model_color").as_double_array();
        } else {
            colors = {0.2, 0.2, 0.2, 1.0};
        }

        viz_corr_model_color.r = colors[0];
        viz_corr_model_color.g = colors[1];
        viz_corr_model_color.b = colors[2];
        viz_corr_model_color.a = colors[3];
    }
    
    if(micp_params_local.find("viz_corr_scale") != micp_params_local.end())
    {
        viz_corr_scale = micp_params_local.at("viz_corr_scale").as_double();
    } else if(micp_params_global.find("viz_corr_scale") != micp_params_global.end())
    {
        viz_corr_scale = micp_params_global.at("viz_corr_scale").as_double();
    } else {
        viz_corr_scale = 0.008;
    }

    if(micp_params_local.find("viz_corr_skip") != micp_params_local.end())
    {
        viz_corr_skip = micp_params_local.at("viz_corr_skip").as_int();
    } else if(micp_params_global.find("viz_corr_skip") != micp_params_global.end())
    {
        viz_corr_skip = micp_params_global.at("viz_corr_skip").as_int();
    } else {
        viz_corr_skip = 0;
    }
}

void MICPRangeSensor::fetchTF()
{
    geometry_msgs::msg::TransformStamped T_sensor_base;

    if(frame != base_frame)
    {
        try
        {
            T_sensor_base = tf_buffer->lookupTransform(base_frame, frame, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(nh_sensor->get_logger(), "%s", ex.what());
            RCLCPP_WARN_STREAM(nh_sensor->get_logger(), "Source: " << frame << ", Target: " << base_frame);
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
                
                marker.header.stamp = nh_sensor->now();
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
                
                marker.header.stamp = nh_sensor->now();
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
                
                marker.header.stamp = nh_sensor->now();
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
                
                marker.header.stamp = nh_sensor->now();
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
                
                marker.header.stamp = nh_sensor->now();
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
                
                marker.header.stamp = nh_sensor->now();
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
                corr_pinhole_optix->computeCovs(Tbms, res);
            }

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
                
                marker.header.stamp = nh_sensor->now();
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
                corr_o1dn_optix->computeCovs(Tbms, res);
            }

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
                
                marker.header.stamp = nh_sensor->now();
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
                corr_ondn_optix->computeCovs(Tbms, res);
            }
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
            nh_sensor = nh_p->create_sub_node(ss.str());
        }

        pub_corr = nh_sensor->create_publisher<visualization_msgs::msg::Marker>("correspondences", 1);
    }
}

void MICPRangeSensor::sphericalCB(
    const rmcl_msgs::msg::ScanStamped::SharedPtr msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
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
}

void MICPRangeSensor::pinholeCB(
    const rmcl_msgs::msg::DepthStamped::SharedPtr msg)
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
    const rmcl_msgs::msg::O1DnStamped::SharedPtr msg)
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
    const rmcl_msgs::msg::OnDnStamped::SharedPtr msg)
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
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
    fetchTF();

    rm::Transform T = rm::Transform::Identity();

    if(frame != msg->header.frame_id)
    {
        try {
            auto Tros = tf_buffer->lookupTransform(frame, msg->header.frame_id,
                               tf2::TimePointZero);
            convert(Tros.transform, T);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(nh_sensor->get_logger(), "%s", ex.what());
        }
    }

    
    auto model_ = std::get<0>(model);

    if(ranges.size() < msg->width * msg->height)
    {
        ranges.resize(msg->width * msg->height);
        // fill with invalid values   
    }

    // fill
    for(size_t i=0; i<ranges.size(); i++)
    {
        ranges[i] = model_.range.max + 1.0;
    }

    sensor_msgs::msg::PointField field_x;
    sensor_msgs::msg::PointField field_y;
    sensor_msgs::msg::PointField field_z;

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

        if(field_x.datatype == sensor_msgs::msg::PointField::FLOAT32)
        {
            // Float
            p.x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
            p.y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
            p.z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
        } else if(field_x.datatype == sensor_msgs::msg::PointField::FLOAT64) {
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
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
    fetchTF();

    rm::Transform T = rm::Transform::Identity();

    if(frame != msg->header.frame_id)
    {
        try {
            auto Tros = tf_buffer->lookupTransform(frame, msg->header.frame_id,
                               tf2::TimePointZero);
            convert(Tros.transform, T);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(nh_sensor->get_logger(), "%s", ex.what());
        }
    }

    auto model_ = std::get<1>(model);

    if(ranges.size() < msg->width * msg->height)
    {
        ranges.resize(msg->width * msg->height);
    }

    sensor_msgs::msg::PointField field_x;
    sensor_msgs::msg::PointField field_y;
    sensor_msgs::msg::PointField field_z;

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

        if(field_x.datatype == sensor_msgs::msg::PointField::FLOAT32)
        {
            // Float
            p.x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
            p.y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
            p.z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
        } else if(field_x.datatype == sensor_msgs::msg::PointField::FLOAT64) {
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
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
    fetchTF();

    rm::Transform T = rm::Transform::Identity();

    if(frame != msg->header.frame_id)
    {
        try {
            auto Tros = tf_buffer->lookupTransform(frame, msg->header.frame_id,
                               tf2::TimePointZero);
            convert(Tros.transform, T);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(nh_sensor->get_logger(), "%s", ex.what());
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

    sensor_msgs::msg::PointField field_x;
    sensor_msgs::msg::PointField field_y;
    sensor_msgs::msg::PointField field_z;

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

        if(field_x.datatype == sensor_msgs::msg::PointField::FLOAT32)
        {
            // Float
            p.x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
            p.y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
            p.z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
        } else if(field_x.datatype == sensor_msgs::msg::PointField::FLOAT64) {
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

// HOW TO IMPLEMENT THIS
// void MICPRangeSensor::pclOnDnCB(
//     const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// {
//     // ROS_INFO_STREAM("sensor: " << name << " received " << data_topic.msg << " message");
//     fetchTF();

//     rm::Transform T = rm::Transform::Identity();

//     if(frame != msg->header.frame_id)
//     {
//         try {
//             auto Tros = tf_buffer->lookupTransform(frame, msg->header.frame_id,
//                                tf2::TimePointZero);
//             convert(Tros.transform, T);
//         } catch (tf2::TransformException &ex) {
//             RCLCPP_WARN(nh_sensor->get_logger(), "%s", ex.what());
//         }
//     }
    
//     auto model_ = std::get<3>(model);

//     if(ranges.size() < msg->width * msg->height)
//     {
//         ranges.resize(msg->width * msg->height);
//         // fill with invalid values
//         for(size_t i=0; i<ranges.size(); i++)
//         {
//             ranges[i] = model_.range.max + 1.0;
            
//         }
//     }

//     if(model_.dirs.size() < msg->width * msg->height)
//     {
//         model_.dirs.resize(msg->width * msg->height);
//         // fill with initial values
//         for(size_t i=0; i<ranges.size(); i++)
//         {
//             model_.dirs[i].x = 1.0;
//             model_.dirs[i].y = 0.0;
//             model_.dirs[i].z = 0.0;
//         }
//     }

//     sensor_msgs::msg::PointField field_x;
//     sensor_msgs::msg::PointField field_y;
//     sensor_msgs::msg::PointField field_z;

//     for(size_t i=0; i<msg->fields.size(); i++)
//     {
//         if(msg->fields[i].name == "x")
//         {
//             field_x = msg->fields[i];
//         }
//         if(msg->fields[i].name == "y")
//         {
//             field_y = msg->fields[i];
//         }
//         if(msg->fields[i].name == "z")
//         {
//             field_z = msg->fields[i];
//         }
//     }

//     for(size_t i=0; i<msg->width * msg->height; i++)
//     {
//         const uint8_t* data_ptr = &msg->data[i * msg->point_step];

//         rm::Point p;

//         if(field_x.datatype == sensor_msgs::msg::PointField::FLOAT32)
//         {
//             // Float
//             p.x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
//             p.y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
//             p.z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
//         } else if(field_x.datatype == sensor_msgs::msg::PointField::FLOAT64) {
//             // Double
//             p.x = *reinterpret_cast<const double*>(data_ptr + field_x.offset);
//             p.y = *reinterpret_cast<const double*>(data_ptr + field_y.offset);
//             p.z = *reinterpret_cast<const double*>(data_ptr + field_z.offset);
//         } else {
//             throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
//         }

//         if(!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
//         {
//             p = T * p;
//             ranges[i] = p.l2norm();
//             model_.dirs[i] = p.normalize();
//         }
//     }

//     // upload
//     #ifdef RMCL_CUDA
//     ranges_gpu = ranges;
//     #endif // RMCL_CUDA

//     // data meta
//     data_last_update = msg->header.stamp;
//     data_received_once = true;

//     updateCorrectors();
//     if(count_valid_ranges)
//     {
//         countValidRanges();
//     }
// }

void MICPRangeSensor::laserCB(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
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
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
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
        RCLCPP_WARN_STREAM(nh_sensor->get_logger(), "Could not convert image of encoding " << msg->encoding);
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
    const rmcl_msgs::msg::ScanInfo::SharedPtr msg)
{
    rm::SphericalModel model_ = std::get<0>(model);
    convert(*msg, model_);
    model = model_;
}

void MICPRangeSensor::pinholeModelCB(
    const rmcl_msgs::msg::DepthInfo::SharedPtr msg)
{
    rm::PinholeModel model_ = std::get<1>(model);
    convert(*msg, model_);
    model = model_;
}

void MICPRangeSensor::o1dnModelCB(
    const rmcl_msgs::msg::O1DnInfo::SharedPtr msg)
{
    rm::O1DnModel model_ = std::get<2>(model);
    convert(*msg, model_);
    model = model_;
}

void MICPRangeSensor::ondnModelCB(
    const rmcl_msgs::msg::OnDnInfo::SharedPtr msg)
{
    rm::OnDnModel model_ = std::get<3>(model);
    convert(*msg, model_);
    model = model_;
}

void MICPRangeSensor::cameraInfoCB(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
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