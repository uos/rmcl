#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/PointCloud2.h>

#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/map/EmbreeMap.hpp>
#include <rmcl/correction/OnDnCorrectorEmbree.hpp>
#include <rmcl/math/math_batched.h>
#include <rmcl/math/math.h>

#include <memory>

#include <fstream>

namespace rm = rmagine;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

ros::Publisher pub_pose;

std::string map_frame = "map";
std::string odom_frame = "odom";
std::string base_frame = "base_link";
std::string lidar_frame = "os_lidar";

int correction_mode = 1;
int iterations = 10;
bool disable_registration = false;
bool generate_evaluation = false;

double max_distance = 0.5;
double min_range = 0.3;
double max_range = 80.0;

rm::EmbreeMapPtr mesh;
rmcl::OnDnCorrectorEmbreePtr corr;
rmcl::CorrectionParams corr_params;
rmcl::CorrectionPtr umeyama;


// GLOBAL BUFFER STORAGE
// store valid scan points as mask
rm::Memory<float> scan_ranges;
rm::Memory<unsigned int, rm::RAM> scan_mask; 

rm::Memory<rm::Point> dataset_points;
rm::Memory<rm::Point> model_points;
rm::Memory<rm::Vector> model_normals;
rm::Memory<unsigned int> corr_valid;

rm::Transform T_mesh_to_map;

rm::Transform Tbm_init; // current guess of base in map

rm::Transform Tbm_start;
rm::Transform Tbg_start;

// estimate this
rm::Transform Tom;

bool first_iteration = true;

rm::OnDnModel ouster_model;

std::ofstream eval_file;

double outlier_dist = 5.0;

void convert(
    const geometry_msgs::Transform &Tros,
    rm::Transform &Trm)
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
    const rm::Transform &Trm,
    geometry_msgs::Transform &Tros)
{
    Tros.rotation.x = Trm.R.x;
    Tros.rotation.y = Trm.R.y;
    Tros.rotation.z = Trm.R.z;
    Tros.rotation.w = Trm.R.w;
    Tros.translation.x = Trm.t.x;
    Tros.translation.y = Trm.t.y;
    Tros.translation.z = Trm.t.z;
}

void convert(
    const rm::Transform &Trm,
    geometry_msgs::Pose &Pros)
{
    Pros.orientation.x = Trm.R.x;
    Pros.orientation.y = Trm.R.y;
    Pros.orientation.z = Trm.R.z;
    Pros.orientation.w = Trm.R.w;
    Pros.position.x = Trm.t.x;
    Pros.position.y = Trm.t.y;
    Pros.position.z = Trm.t.z;
}

void pclCB(const sensor_msgs::PointCloud2ConstPtr &pcl)
{
    ROS_INFO("Got Cloud");

    rm::Transform Tsb; // Transform sensor -> base (urdf)
    rm::Transform Tbo; // Transform base -> odom (odometry estimation)
    rm::Transform Tsl = rm::Transform::Identity();
    rm::Transform Tls = rm::Transform::Identity();
    double Tbo_time_error = 0.0;

    try
    {
        geometry_msgs::TransformStamped T;

        // static
        T = tf_buffer->lookupTransform(base_frame, pcl->header.frame_id, pcl->header.stamp);
        convert(T.transform, Tsb);

        // dynamic
        T = tf_buffer->lookupTransform(
            odom_frame, base_frame,
            pcl->header.stamp, ros::Duration(1.0));
        convert(T.transform, Tbo);

        Tbo_time_error = (pcl->header.stamp - T.header.stamp).toSec();

        if (pcl->header.frame_id != lidar_frame)
        {
            T = tf_buffer->lookupTransform(lidar_frame, pcl->header.frame_id,
                                           pcl->header.stamp);
            convert(T.transform, Tsl);
            Tls = ~Tsl;
        }
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("micp_eval - %s", ex.what());
        return;
    }

    if (first_iteration)
    {
        // For ground truth
        // Tbm_init = Tbo;

        // Tbm: T[v[0.24497,-0.0313249,-0.334714], E[0.014787, -0.0566967, 0.00265724]]
        // rm::Transform Tbm_corr;
        // Tbm_corr.R = rm::EulerAngles{0.014787, -0.0566967, 0.00265724};
        // Tbm_corr.t = rm::Vector3{0.24497,-0.0313249,-0.334714};

        // Tbm_init = Tbm_init * Tbm_corr;

        std::cout << "Calculate Tom for the first time" << std::endl;
        Tom = Tbm_init * ~Tbo;
        std::cout << "- Tbm_init: " << Tbm_init << std::endl;
        std::cout << "- Tbo: " << Tbo << std::endl;
        std::cout << "- Tob: " << ~Tbo << std::endl;
        std::cout << "-> Tom = " << Tom << std::endl;

        // save for evaluation
        if(generate_evaluation)
        {
            Tbm_start = Tom * Tbo;

            std::cout << "Evaluation stores absolut poses and poses relative to:" << std::endl;
            std::cout << "- Tbm: " << Tbm_start << std::endl;
        }

        first_iteration = false;
    }
    //
    // std::cout << "Odom Time Error: " << Tbo_time_error << std::endl;

    // 1. determine Tom
    // rm::Transform Tom = Tbm * Tbm

    std::cout << "Set Tsb: " << Tsb * Tls << std::endl;
    rm::Transform Tlb = Tsb * Tls;
    corr->setTsb(Tlb);

    // std::cout << "Building Exact Scanner Model" << std::endl;

    size_t n_scan_points = pcl->width * pcl->height;
    size_t buffer_overestimate = 100;

    if(scan_ranges.size() < n_scan_points)
    {
        // std::cout << "RESIZE GLOBAL BUFFERS" << std::endl;
        scan_ranges.resize(n_scan_points + buffer_overestimate);
        scan_mask.resize(n_scan_points + buffer_overestimate);
    }

    // get model
    
    ouster_model.dirs.resize(n_scan_points);
    ouster_model.origs.resize(n_scan_points);
    ouster_model.width = pcl->width;
    ouster_model.height = pcl->height;

    sensor_msgs::PointField field_x;
    sensor_msgs::PointField field_y;
    sensor_msgs::PointField field_z;

    for(size_t i = 0; i < pcl->fields.size(); i++)
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
    for (size_t i = 0; i < n_scan_points; i++)
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

        if (!std::isnan(ps.x) && !std::isnan(ps.y) && !std::isnan(ps.z))
        {
            rm::Vector3 pl = Tsl * ps; // transform to sensor coords
            float range_est = pl.l2norm();
            float theta_est = atan2(pl.y, pl.x);    // horizontal
            float phi_est = atan2(pl.z, range_est); // vertical

            if (range_est > ouster_model.range.min 
                && range_est < ouster_model.range.max)
            {
                rm::Vector ol = {0.0, 0.0, 0.0};
                rm::Vector3 dl = (pl - ol);

                if(dl.l2normSquared() > 0.0001)
                {
                    dl = (pl - ol).normalize();
                    ouster_model.origs[i] = ol;
                    ouster_model.dirs[i] = dl;
                    scan_ranges[i] = range_est;
                    scan_mask[i] = 1;

                    n_valid++;
                } else {
                    // dir too short
                    scan_ranges[i] = -0.1;
                    scan_mask[i] = 0;

                }
            }
            else
            {
                scan_ranges[i] = -0.1;
                scan_mask[i] = 0;
            }
        }
        else
        {
            // TODO: write appropriate things into buffers
            scan_ranges[i] = -0.1;
        }
    }

    corr->setModel(ouster_model);
    corr->setInputData(scan_ranges);

    std::cout << n_valid << "/" << n_scan_points << " valid measurements" << std::endl;
    // we need robot(base) -> mesh
    // T_b_mesh = Tm_mesh * Tbm

    size_t num_registration = iterations;

    if(!disable_registration && num_registration > 0)
    {
        if(dataset_points.size() < n_scan_points)
        {   
            // Resize buffers!
            dataset_points.resize(n_scan_points + buffer_overestimate);
            corr_valid.resize(n_scan_points + buffer_overestimate);
            model_points.resize(n_scan_points + buffer_overestimate);
            model_normals.resize(n_scan_points + buffer_overestimate);
        }

        // std::cout << "Start Registration Iterations (" << iterations << ")" << std::endl;

        rm::StopWatchHR sw;
        double el;

        if(correction_mode == 0)
        {
            sw();
            for (size_t i = 0; i < num_registration; i++)
            {
                rm::Transform Tbm = Tom * Tbo;
                rm::Transform T_base_mesh = ~T_mesh_to_map * Tbm;

                rm::Memory<rm::Transform> Tbms(1);
                Tbms[0] = T_base_mesh;
                
                corr->findSPC(Tbms, dataset_points, model_points, corr_valid);

                rmcl::CorrectionPreResults<rm::RAM> res;
                res.ds.resize(Tbms.size());
                res.ms.resize(Tbms.size());
                res.Cs.resize(Tbms.size());
                res.Ncorr.resize(Tbms.size());

                rmcl::means_covs_online_batched(
                    dataset_points, model_points, corr_valid, // input
                    res.ds, res.ms,                           // outputs
                    res.Cs, res.Ncorr);

                auto Tdeltas = umeyama->correction_from_covs(res);
                rm::Transform Tdelta = Tdeltas[0];
                Tom = Tbm * Tdelta * ~Tbo;
            }
            el = sw();
            // std::cout << "- Tbm Registered: " << Tom * Tbo << " in " << el * 1000.0 << " ms" << std::endl;

        } else if(correction_mode == 1) {
            rm::Transform Tbm = Tom * Tbo;
            rm::Transform T_base_mesh = ~T_mesh_to_map * Tbm;

            rm::Memory<rm::Transform> Tbms(1);
            Tbms[0] = T_base_mesh;

            // sw();
            corr->findRCC(Tbms, dataset_points, model_points, model_normals, corr_valid);
            // el = sw();
            // std::cout << "- findRCC: " << el * 1000.0 << " ms" << std::endl;

            rm::Memory<rm::Transform> Tdelta_total(Tbms.size()); // size=1
            Tdelta_total[0] = rm::Transform::Identity();

            rmcl::CorrectionPreResults<rm::RAM> res;
            res.ds.resize(Tbms.size());
            res.ms.resize(Tbms.size());
            res.Cs.resize(Tbms.size());
            res.Ncorr.resize(Tbms.size());

            // sw();
            for(size_t i = 0; i < num_registration; i++)
            {
                rmcl::means_covs_p2l_online_batched(
                    Tdelta_total,
                    dataset_points, scan_mask, // from
                    model_points, model_normals, // to
                    corr_valid,
                    max_distance,
                    res.ds, res.ms,                           // outputs
                    res.Cs, res.Ncorr);

                auto Tdeltas = umeyama->correction_from_covs(res);

                // update total delta
                Tdelta_total[0] = Tdelta_total[0] * Tdeltas[0];
            }



            // Update Tom
            Tom = Tbm * Tdelta_total[0] * ~Tbo;
            // el = sw();

            std::cout << "- Tbm Registered: " << Tom * Tbo << ", valid corr: " << res.Ncorr[0] << std::endl;
        }
    }
    else
    {
        std::cout << "Registration disabled" << std::endl;

        // should not be required
        // rm::Transform Tbm = Tom * Tbo;
        // rm::Transform T_base_mesh = ~T_mesh_to_map * Tbm;
        // Tom = Tbm * ~Tbo;
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

    if (generate_evaluation)
    {
        rm::Transform Tbm = Tom * Tbo;
        // T_b_bold = T_bold_map^-1 * T_b_map
        rm::Transform Tbm_rel = ~Tbm_start * Tbm;

        // get last correspondences and determine correspondence error
        rm::Transform T_base_mesh = ~T_mesh_to_map * Tbm;

        rm::Memory<rm::Transform> Tbms(1);
        Tbms[0] = T_base_mesh;

        rm::Memory<rm::Point> dataset_points;
        rm::Memory<rm::Point> model_points;
        rm::Memory<unsigned int> corr_valid;

        rmcl::CorrectionParams corr_params_eval = corr_params;
        corr_params_eval.max_distance = outlier_dist;

        corr->setParams(corr_params_eval);
        corr->findSPC(Tbms, dataset_points, model_points, corr_valid);
        corr->setParams(corr_params);

        std::vector<double> p2ms;

        double p2m_min = 0.0;
        double p2m_max = 0.0;
        double p2m_mean = 0.0;
        double p2m_median = 0.0;

        for (size_t i = 0; i < corr_valid.size(); i++)
        {
            if (corr_valid[i] > 0)
            {
                rm::Point diff = dataset_points[i] - model_points[i];
                // mean_point_to_mesh_distance += diff.l2norm();
                // n_corr++;
                p2ms.push_back(diff.l2norm());
            }
        }

        size_t n_corr = p2ms.size();

        if (n_corr > 0)
        {
            std::sort(p2ms.begin(), p2ms.end());

            p2m_min = p2ms.front();
            p2m_max = p2ms.back();
            p2m_median = p2ms[p2ms.size() / 2];

            for(auto v : p2ms)
            {
                p2m_mean += v;
            }

            p2m_mean /= static_cast<double>(n_corr);
        }
        else
        {
            // something wrong
        }

        { // print to command line
            std::cout << std::endl;
            std::cout << "---------------------------------" << std::endl;
            std::cout << "Evalutation Statistics:" << std::endl;
            std::cout << "- Tbm: " << Tbm << std::endl;
            std::cout << "- Tbm_rel: " << Tbm_rel << std::endl;
            std::cout << "- Time error: " << Tbo_time_error << " s" << std::endl;
            std::cout << "- P2M:" << std::endl;
            std::cout << "  - N: " << p2ms.size() << std::endl;
            std::cout << "  - Min, max: " << p2m_min << " m, " << p2m_max << " m" << std::endl;
            std::cout << "  - Mean: " << p2m_mean << " m" << std::endl;
            std::cout << "  - Median: " << p2m_median << " m" << std::endl;
            std::cout << "---------------------------------" << std::endl;
            std::cout << std::endl;
        }
        

        // Write everything to a file
        if (!eval_file.is_open())
        {
            eval_file.open("micp_eval.csv", std::ios::out);
            eval_file.precision(std::numeric_limits<double>::max_digits10 + 2);
            eval_file << std::fixed;
            eval_file << "# Tbm.t.x, Tbm.t.y, Tbm.t.z, Tbm.R.x, Tbm.R.y, Tbm.R.z, Tbm.R.w, Tbm.stamp";
            eval_file << ", Tbm_rel.t.x, Tbm_rel.t.y, Tbm_rel.t.z, Tbm_rel.R.x, Tbm_rel.R.y, Tbm_rel.R.z, Tbm_rel.R.w, Tbm_rel.stamp";
            eval_file << ", Estamp, Ncorr, P2M_min, P2M_max, P2M_mean, P2M_median\n";
        }

        eval_file << Tbm.t.x << ", " << Tbm.t.y << ", " << Tbm.t.z << ", " << Tbm.R.x << ", " << Tbm.R.y << ", " << Tbm.R.z << ", " << Tbm.R.w << ", " << pcl->header.stamp.toSec();
        eval_file << ", ";
        eval_file << Tbm_rel.t.x << ", " << Tbm_rel.t.y << ", " << Tbm_rel.t.z << ", " << Tbm_rel.R.x << ", " << Tbm_rel.R.y << ", " << Tbm_rel.R.z << ", " << Tbm_rel.R.w << ", " << pcl->header.stamp.toSec();
        eval_file << ", ";
        eval_file << Tbo_time_error << ", " << n_corr << ", " << p2m_min << ", " << p2m_max << ", " << p2m_mean << ", " << p2m_median;
        eval_file << "\n";
    }
}

void loadParameters(ros::NodeHandle nh_p)
{
    T_mesh_to_map = rm::Transform::Identity();
    std::vector<double> transform_params;
    if (nh_p.getParam("mesh_to_map_transform", transform_params))
    {
        if (transform_params.size() == 6)
        {
            T_mesh_to_map.t = rm::Vector{
                (float)transform_params[0],
                (float)transform_params[1],
                (float)transform_params[2]};
            T_mesh_to_map.R = rm::EulerAngles{
                (float)transform_params[3],
                (float)transform_params[4],
                (float)transform_params[5]};
        }
        else if (transform_params.size() == 7)
        {
            T_mesh_to_map.t = rm::Vector{
                (float)transform_params[0],
                (float)transform_params[1],
                (float)transform_params[2]};
            T_mesh_to_map.R = rm::Quaternion{
                (float)transform_params[3],
                (float)transform_params[4],
                (float)transform_params[5],
                (float)transform_params[6]};
        }
    }

    Tbm_init = rm::Transform::Identity();
    std::vector<double> initial_guess_params;
    if (nh_p.getParam("initial_guess", initial_guess_params))
    {
        if (initial_guess_params.size() == 6)
        {
            Tbm_init.t = rm::Vector{
                (float)initial_guess_params[0],
                (float)initial_guess_params[1],
                (float)initial_guess_params[2]};
            Tbm_init.R = rm::EulerAngles{
                (float)initial_guess_params[3],
                (float)initial_guess_params[4],
                (float)initial_guess_params[5]};
        }
        else if (initial_guess_params.size() == 7)
        {
            Tbm_init.t = rm::Vector{
                (float)initial_guess_params[0],
                (float)initial_guess_params[1],
                (float)initial_guess_params[2]};
            Tbm_init.R = rm::Quaternion{
                (float)initial_guess_params[3],
                (float)initial_guess_params[4],
                (float)initial_guess_params[5],
                (float)initial_guess_params[6]};
        }
    }

    std::string mesh_file = "/home/amock/hilti_uzh_tracking_area/reduced_mesh_09.ply";
    nh_p.param<std::string>("mesh_file", mesh_file, "/home/amock/hilti_uzh_tracking_area/reduced_mesh_09.ply");

    mesh = rm::import_embree_map(mesh_file);
    corr = std::make_shared<rmcl::OnDnCorrectorEmbree>(mesh);
    umeyama = std::make_shared<rmcl::Correction>();

    // because float is not possible
    

    nh_p.param<int>("correction_mode", correction_mode, 1);
    nh_p.param<int>("iterations", iterations, 10);
    nh_p.param<bool>("disable_registration", disable_registration, false);
    nh_p.param<bool>("generate_evaluation", generate_evaluation, false);
    nh_p.param<double>("max_distance", max_distance, 0.5);
    nh_p.param<double>("min_range", min_range, 0.3);
    nh_p.param<double>("max_range", max_range, 80.0);
    nh_p.param<double>("outlier_dist", outlier_dist, 5.0);

    corr_params.max_distance = max_distance;
    corr->setParams(corr_params);

    ouster_model.range.min = min_range;
    ouster_model.range.max = max_range;

    nh_p.param<std::string>("map_frame", map_frame, "map");
    nh_p.param<std::string>("odom_frame", odom_frame, "odom");
    nh_p.param<std::string>("base_frame", base_frame, "base_link");
    nh_p.param<std::string>("lidar_frame", lidar_frame, "os_lidar");

    {
        rm::OnDnSimulatorEmbreePtr test_corr = std::make_shared<rm::OnDnSimulatorEmbree>(mesh);

        rm::OnDnModel model;
        model.dirs.resize(6);
        model.origs.resize(6);
        model.width = 6;
        model.height = 1;
        model.range.min = 0.0;
        model.range.max = 1000.0;

        // front
        model.origs[0] = {0.0, 0.0, 0.0};
        model.dirs[0] = {1.0, 0.0, 0.0};

        // left
        model.origs[1] = {0.0, 0.0, 0.0};
        model.dirs[1] = {0.0, 1.0, 0.0};
        
        // right
        model.origs[2] = {0.0, 0.0, 0.0};
        model.dirs[2] = {0.0, -1.0, 0.0};

        // back
        model.origs[3] = {0.0, 0.0, 0.0};
        model.dirs[3] = {-1.0, 0.0, 0.0};

        // top
        model.origs[4] = {0.0, 0.0, 0.0};
        model.dirs[4] = {0.0, 0.0, 1.0};

        // bottom
        model.origs[5] = {0.0, 0.0, 0.0};
        model.dirs[5] = {0.0, 0.0, -1.0};


        test_corr->setModel(model);

        rm::Transform Tsb = rm::Transform::Identity();
        Tsb.t.z = 0.3;
        test_corr->setTsb(Tsb);

        rm::Mem<rm::Transform> poses(1);
        poses[0] = rm::Transform::Identity();

        using ResT = rm::IntAttrAny<rm::RAM>;
        ResT results = test_corr->simulate<ResT>(poses);

        std::cout << "Test Simulation: " << std::endl;
        
        std::vector<std::string> labels = {
            "FRONT",
            "LEFT",
            "RIGHT",
            "BACK",
            "UP",
            "DOWN"
        };

        for(size_t i=0; i<6; i++)
        {
            std::cout << labels[i] << std::endl;
            std::cout << "- range: " << results.ranges[i] << std::endl;
            std::cout << "- point: " << results.points[i] << std::endl;
            std::cout << "- normal: " << results.normals[i] << std::endl;
            std::cout << "- obj_id: " << results.object_ids[i] << std::endl;
            std::cout << "- geom_id: " << results.geom_ids[i] << std::endl;
        }

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "micp_eval");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("MICP EVALUATION SCRIPT STARTED.");


    loadParameters(nh_p);

    // get all necessary transformations
    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("micp_pose", 1);

    ros::Subscriber sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>("ouster/points", 10, pclCB);

    ros::spin();

    tf_listener.reset();
    tf_buffer.reset();

    if(eval_file.is_open())
    {
        eval_file.close();
    }

    return 0;
}