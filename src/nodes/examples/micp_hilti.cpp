#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/PointCloud2.h>


#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>
#include <rmagine/map/EmbreeMap.hpp>
#include <rmcl/correction/OnDnCorrectorEmbree.hpp>
#include <rmcl/math/math_batched.h>
#include <rmcl/math/math.h>

#include <memory>



namespace rm = rmagine;


std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

ros::Publisher pub_pose;


std::string map_frame = "map";
std::string odom_frame = "odom";
std::string base_frame = "base_link";
std::string lidar_frame = "os_lidar";

int iterations = 10;



rm::EmbreeMapPtr mesh;
rmcl::OnDnCorrectorEmbreePtr corr;
rmcl::CorrectionPtr umeyama;

rm::Transform T_mesh_to_map;


rm::Transform Tbm_init; // current guess of base in map

// estimate this
rm::Transform Tom;

bool first_iteration = true;


rm::OnDnModel ouster_model;


void convert(
    const geometry_msgs::Transform& Tros,
    rm::Transform& Trm)
{
    Trm.R.x = Tros.rotation.x;
    Trm.R.y = Tros.rotation.y;
    Trm.R.z = Tros.rotation.z;
    Trm.R.w = Tros.rotation.w;
    Trm.t.x = Tros.translation.x;
    Trm.t.y = Tros.translation.y;
    Trm.t.z = Tros.translation.z;
}

void convert(
    const rm::Transform& Trm,
    geometry_msgs::Transform& Tros)
{
    Tros.rotation.x    = Trm.R.x; 
    Tros.rotation.y    = Trm.R.y; 
    Tros.rotation.z    = Trm.R.z; 
    Tros.rotation.w    = Trm.R.w; 
    Tros.translation.x = Trm.t.x; 
    Tros.translation.y = Trm.t.y;
    Tros.translation.z = Trm.t.z;
}

void convert(
    const rm::Transform& Trm,
    geometry_msgs::Pose& Pros)
{
    Pros.orientation.x = Trm.R.x;
    Pros.orientation.y = Trm.R.y;
    Pros.orientation.z = Trm.R.z;
    Pros.orientation.w = Trm.R.w;
    Pros.position.x = Trm.t.x;
    Pros.position.y = Trm.t.y;
    Pros.position.z = Trm.t.z;
}

void pclCB(const sensor_msgs::PointCloud2ConstPtr& pcl)
{
    ROS_INFO("Got Cloud");

    rm::Transform Tsb; // Transform sensor -> base (urdf)
    rm::Transform Tbo; // Transform base -> odom (odometry estimation)
    rm::Transform Tsl = rm::Transform::Identity();
    rm::Transform Tls;
    double Tbo_time_error = 0.0;
    
    try {
        geometry_msgs::TransformStamped T;

        // static
        T = tf_buffer->lookupTransform(base_frame, pcl->header.frame_id, pcl->header.stamp);
        convert(T.transform, Tsb);

        // dynamic
        T = tf_buffer->lookupTransform(odom_frame, base_frame, pcl->header.stamp);
        convert(T.transform, Tbo);

        Tbo_time_error = (pcl->header.stamp - T.header.stamp).toSec();

        if(pcl->header.frame_id != lidar_frame)
        {
            T = tf_buffer->lookupTransform(lidar_frame, pcl->header.frame_id,
                                              pcl->header.stamp);
            convert(T.transform, Tsl);
            Tls = ~Tsl;
        }


    } catch (tf2::TransformException &ex) {
        ROS_WARN("micp_hilti - %s", ex.what());
        return;
    }

    if(first_iteration)
    {
        Tbm_init = Tbo;

        std::cout << "Calculate Tom for the first time" << std::endl;
        Tom = Tbm_init * ~Tbo;
        std::cout << "- Tbm_init: " << Tbm_init << std::endl;
        std::cout << "- Tbo: " << Tbo << std::endl;
        std::cout << "- Tob: " << ~Tbo << std::endl;
        std::cout << "-> Tom = " << Tom << std::endl;

        first_iteration = false;
    }

    std::cout << "Odom Time Error: " << Tbo_time_error << std::endl;

    // 1. determine Tom
    // rm::Transform Tom = Tbm * Tbm

    corr->setTsb(Tsb * Tls);

    std::cout << "Building Exact Scanner Model" << std::endl;

    // get model
    rm::Memory<float> ranges(pcl->width * pcl->height);
    ouster_model.dirs.resize(pcl->width * pcl->height);
    ouster_model.origs.resize(pcl->width * pcl->height);
    ouster_model.width = pcl->width;
    ouster_model.height = pcl->height;
    ouster_model.range.min = 0.3;
    ouster_model.range.max = 80.0;

    sensor_msgs::PointField field_x;
    sensor_msgs::PointField field_y;
    sensor_msgs::PointField field_z;

    for (size_t i = 0; i < pcl->fields.size(); i++)
    {
        if (pcl->fields[i].name == "x")
        {
            field_x = pcl->fields[i];
        }
        if (pcl->fields[i].name == "y")
        {
            field_y = pcl->fields[i];
        }
        if (pcl->fields[i].name == "z")
        {
            field_z = pcl->fields[i];
        }
    }

    size_t n_valid = 0;
    for (size_t i = 0; i < pcl->width * pcl->height; i++)
    {
        const uint8_t *data_ptr = &pcl->data[i * pcl->point_step];
        
        rm::Vector ps;

        if (field_x.datatype == sensor_msgs::PointField::FLOAT32)
        {
            // Float
            ps.x = *reinterpret_cast<const float *>(data_ptr + field_x.offset);
            ps.y = *reinterpret_cast<const float *>(data_ptr + field_y.offset);
            ps.z = *reinterpret_cast<const float *>(data_ptr + field_z.offset);
        }
        else if (field_x.datatype == sensor_msgs::PointField::FLOAT64)
        {
            // Double
            ps.x = *reinterpret_cast<const double *>(data_ptr + field_x.offset);
            ps.y = *reinterpret_cast<const double *>(data_ptr + field_y.offset);
            ps.z = *reinterpret_cast<const double *>(data_ptr + field_z.offset);
        }
        else
        {
            throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
        }

        if(!std::isnan(ps.x) && !std::isnan(ps.y) && !std::isnan(ps.z))
        {
            rm::Vector3 pl = Tsl * ps; // transform to sensor coords
            float range_est = pl.l2norm();
            float theta_est = atan2(pl.y, pl.x); // horizontal
            float phi_est = atan2(pl.z, range_est); // vertical

            if(range_est > ouster_model.range.min && range_est < ouster_model.range.max)
            {
                rm::Vector ol = {0.0, 0.0, 0.0};
                rm::Vector3 dl = (pl-ol).normalize();

                ouster_model.origs[i] = ol;
                ouster_model.dirs[i] = dl;
                ranges[i] = range_est;
                n_valid++;
            } else {
                ranges[i] = -0.1;
            }

        } else {
            // TODO: write appropriate things into buffers
            ranges[i] = -0.1;
        }
    }

    corr->setModel(ouster_model);
    corr->setInputData(ranges);

    std::cout << n_valid << " valid measurements" << std::endl;

    std::cout << "Start Registration Iterations (" << iterations << ")" << std::endl;
    std::cout << "- T mesh to map: " << T_mesh_to_map << std::endl;
    std::cout << "- Tom: " << Tom << std::endl;
    std::cout << "- Tbo: " << Tbo << std::endl;

    // we need robot(base) -> mesh
    // T_b_mesh = Tm_mesh * Tbm

    size_t num_registration = iterations;

    for(size_t i=0; i<num_registration; i++)
    {
        rm::Transform Tbm = Tom * Tbo;
        rm::Transform T_base_mesh = ~T_mesh_to_map * Tbm;


        rm::Memory<rm::Transform> Tbms(1);
        Tbms[0] = T_base_mesh;

        rm::Memory<rm::Point> dataset_points;
        rm::Memory<rm::Point> model_points;
        rm::Memory<unsigned int> corr_valid;
        corr->findSPC(Tbms, dataset_points, model_points, corr_valid);

        std::cout << "Found " << dataset_points.size() << " SPCs" << std::endl;

        rmcl::CorrectionPreResults<rm::RAM> res;
        res.ds.resize(Tbms.size());
        res.ms.resize(Tbms.size());
        res.Cs.resize(Tbms.size());
        res.Ncorr.resize(Tbms.size());

        rmcl::means_covs_online_batched(
                dataset_points, model_points, corr_valid, // input
                res.ds, res.ms, // outputs
                res.Cs, res.Ncorr
            );

        auto Tdeltas = umeyama->correction_from_covs(res);

        rm::Transform Tdelta = Tdeltas[0];
        std::cout << "Tdelta: " << Tdelta << std::endl;

        Tom = Tbm * Tdelta * ~Tbo;

        std::cout << "New Tbm: " <<  Tom * Tbo << std::endl;
    }

    { // broadcast transform from odom -> map: Tom
        static tf2_ros::TransformBroadcaster br;
    
        geometry_msgs::TransformStamped Tom_ros;
        Tom_ros.header.stamp = pcl->header.stamp;
        Tom_ros.header.frame_id = map_frame;
        Tom_ros.child_frame_id = odom_frame;
        convert(Tom, Tom_ros.transform);

        br.sendTransform(Tom_ros);
    }

    { // publish pose in map frame
        geometry_msgs::PoseStamped micp_pose;
        micp_pose.header.stamp = pcl->header.stamp;
        micp_pose.header.frame_id = map_frame;
        convert(Tom * Tbo, micp_pose.pose);
        pub_pose.publish(micp_pose);
    }

}

void loadParameters(ros::NodeHandle nh_p)
{
    T_mesh_to_map = rm::Transform::Identity();
    std::vector<double> transform_params;
    if(nh_p.getParam("mesh_to_map_transform", transform_params))
    {
        if(transform_params.size() == 6)
        {
            T_mesh_to_map.t = rm::Vector{
                    (float)transform_params[0],
                    (float)transform_params[1],
                    (float)transform_params[2]};
            T_mesh_to_map.R = rm::EulerAngles{
                    (float)transform_params[3],
                    (float)transform_params[4],
                    (float)transform_params[5]};
        } else if(transform_params.size() == 7) {
            T_mesh_to_map.t = rm::Vector{
                    (float)transform_params[0],
                    (float)transform_params[1],
                    (float)transform_params[2]};
            T_mesh_to_map.R = rm::Quaternion{
                    (float)transform_params[3],
                    (float)transform_params[4],
                    (float)transform_params[5],
                    (float)transform_params[6]
            };
        }
    }

    

    // T_mesh_to_map = T_mesh_to_map.inv();

    Tbm_init = rm::Transform::Identity();
    std::vector<double> initial_guess_params;
    if(nh_p.getParam("initial_guess", initial_guess_params))
    {
        if(initial_guess_params.size() == 6)
        {
            Tbm_init.t = rm::Vector{
                    (float)initial_guess_params[0],
                    (float)initial_guess_params[1], 
                    (float)initial_guess_params[2]};
            Tbm_init.R = rm::EulerAngles{
                    (float)initial_guess_params[3],
                    (float)initial_guess_params[4],
                    (float)initial_guess_params[5]};
        } else if(initial_guess_params.size() == 7) {
            Tbm_init.t = rm::Vector{
                    (float)initial_guess_params[0],
                    (float)initial_guess_params[1],
                    (float)initial_guess_params[2]};
            Tbm_init.R = rm::Quaternion{
                    (float)initial_guess_params[3],
                    (float)initial_guess_params[4],
                    (float)initial_guess_params[5],
                    (float)initial_guess_params[6]
            };
        }
    }

    std::string mesh_file = "/home/amock/hilti_uzh_tracking_area/reduced_mesh_09.ply";
    nh_p.param<std::string>("mesh_file", mesh_file, "/home/amock/hilti_uzh_tracking_area/reduced_mesh_09.ply");

    mesh = rm::import_embree_map(mesh_file);
    corr = std::make_shared<rmcl::OnDnCorrectorEmbree>(mesh);
    umeyama = std::make_shared<rmcl::Correction>();

    nh_p.param<int>("iterations", iterations, 10);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "micp_hilti");


    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");


    ROS_INFO("MICP HILTI STARTED.");

    // throw std::runtime_error("bla");

    loadParameters(nh_p);

    // get all necessary transformations
    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("micp_pose", 1);

    ros::Subscriber sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>("/ouster/points", 1, pclCB);

    ros::spin();

    tf_listener.reset();
    tf_buffer.reset();

    return 0;
}