#include <ros/ros.h>

#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/math.h>
#ifdef RMCL_CUDA
#include <rmagine/math/math.cuh>
#endif // RMCL_CUDA

#include <rmcl/correction/MICP.hpp>
#include <rmcl/util/conversions.h>


#include <thread>
#include <mutex>


#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace rmcl;
using namespace rmagine;

MICPPtr micp;

std::string map_frame;
std::string odom_frame;
bool has_odom_frame = true;
std::string base_frame;

// this should be attached to each sensor instead
bool adaptive_max_dist = false;
float adaptive_max_dist_min = 0.15;


// Estimate this
geometry_msgs::TransformStamped T_odom_map;
Transform Tom;
std::mutex                      T_odom_map_mutex;

// dynamic: ekf
geometry_msgs::TransformStamped T_base_odom;
std::mutex                      T_base_odom_mutex;
Transform Tbo;

bool invert_tf = false;
bool correction_disabled = false;
bool draw_correspondences = false;

Transform initial_pose_offset;
unsigned int combining_unit = 1;


// testing
size_t Nposes = 1;


std::thread correction_thread;
bool stop_correction_thread = false;

TFBufferPtr tf_buffer;
TFListenerPtr tf_listener;

bool pose_received = false;


void fetchTF()
{
    std::lock_guard<std::mutex> guard(T_base_odom_mutex);

    if(has_odom_frame)
    {
        try {
            T_base_odom = tf_buffer->lookupTransform(odom_frame, base_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ROS_WARN_STREAM("Source (Base): " << base_frame << ", Target (Odom): " << odom_frame);
            return;
        }
    } else {
        T_base_odom.header.frame_id = odom_frame;
        T_base_odom.child_frame_id = base_frame;
        T_base_odom.transform.translation.x = 0.0;
        T_base_odom.transform.translation.y = 0.0;
        T_base_odom.transform.translation.z = 0.0;
        T_base_odom.transform.rotation.x = 0.0;
        T_base_odom.transform.rotation.y = 0.0;
        T_base_odom.transform.rotation.z = 0.0;
        T_base_odom.transform.rotation.w = 1.0;
    }

    convert(T_base_odom.transform, Tbo);
}

void updateTF()
{
    static tf2_ros::TransformBroadcaster br;
    
    geometry_msgs::TransformStamped T;

    // What is the source frame?
    if(has_odom_frame)
    {
        // With EKF and base_frame: Send odom to map
        if(!invert_tf)
        {
            convert(Tom, T.transform);
            T.header.frame_id = map_frame;
            T.child_frame_id = odom_frame;
        } else {
            convert(~Tom, T.transform);
            T.header.frame_id = odom_frame;
            T.child_frame_id = map_frame;
        }
    } else {
        // With base but no EKF: send base to map
        auto Tbm = Tom * Tbo;
        if(!invert_tf)
        {
            convert(Tbm, T.transform);
            T.header.frame_id = map_frame;
            T.child_frame_id = base_frame;
        } else {
            convert(~Tbm, T.transform);
            T.header.frame_id = base_frame;
            T.child_frame_id = map_frame;
        }
    }

    T.header.stamp = ros::Time::now();
    br.sendTransform(T);
}

void correctOnce()
{
    std::lock_guard<std::mutex> guard1(T_base_odom_mutex);
    std::lock_guard<std::mutex> guard2(T_odom_map_mutex);

    // 1. Get Base in Map
    Transform Tbm = Tom * Tbo;

    Memory<Transform, RAM> poses(Nposes);
    for(size_t i=0; i<Nposes; i++)
    {
        poses[i] = Tbm;
    }

    Transform dT0;
    unsigned int ncorr0 = 0;

    #ifdef RMCL_CUDA
        // exact copy of poses
        Memory<Transform, VRAM_CUDA> poses_ = poses;

        // 0: use CPU to combine sensor corrections
        // 1: use GPU to combine sensor corrections
        if(combining_unit == 0)
        { // CPU version

            Memory<Transform, RAM> dT(poses.size());
            CorrectionPreResults<RAM> covs;

            micp->correct(poses, poses_, covs, dT);
            poses = multNxN(poses, dT);
            dT0 = dT[0];
        }
        else if(combining_unit == 1)
        { // GPU version

            Memory<Transform, VRAM_CUDA> dT_(poses.size());
            CorrectionPreResults<VRAM_CUDA> covs_;

            micp->correct(poses, poses_, covs_, dT_);
            poses_ = multNxN(poses_, dT_);
            // download
            poses = poses_;
            Memory<Transform, RAM> dT = dT_(0,1);
            Memory<unsigned int, RAM> Ncorr = covs_.Ncorr(0,1);
            dT0 = dT[0];
            ncorr0 = Ncorr[0];
        }
    #else // RMCL_CUDA
        
        // 0: use CPU to combine sensor corrections
        // 1: use GPU to combine sensor corrections
        if(combining_unit == 0)
        { // CPU version

            Memory<Transform, RAM> dT(poses.size());
            CorrectionPreResults<RAM> covs;

            micp->correct(poses, covs, dT);
            poses = multNxN(poses, dT);
            dT0 = dT[0];
        }
        else if(combining_unit == 1)
        { // GPU version

            std::cout << "ERROR: combining unit " << combining_unit << " not available" << std::endl;
        }
    #endif // RMCL_CUDA

    if(adaptive_max_dist)
    {
        float trans_force = dT0.t.l2norm();
        float trans_progress = 1.0 / exp(10.0 * trans_force);

        Quaternion qunit;
        qunit.setIdentity();
        float qscalar = dT0.R.dot(qunit);
        float rot_progress = qscalar * qscalar;

        
        unsigned int n_valid_ranges = 0;
        unsigned int n_total_ranges = 0;
        for(auto elem : micp->sensors())
        {
            n_valid_ranges += elem.second->n_ranges_valid;
            n_total_ranges += elem.second->ranges.size();
        }

        float match_ratio = static_cast<float>(ncorr0) / static_cast<float>(n_valid_ranges);
        float adaption_rate = trans_progress * rot_progress * match_ratio;
        
        
        // std::cout << "match ratio = " << match_ratio << ", adaption rate = " << adaption_rate << std::endl;
        for(auto elem : micp->sensors())
        {
            elem.second->adaptCorrectionParams(match_ratio, adaption_rate);
            // std::cout << "- " << elem.first << " - adapt correction params to max_distance = " << elem.second->corr_params.max_distance << std::endl;
        }
    }

    // Update T_odom_map

    // pose == Tbm
    // Tom = Tbm * Tob
    // 

    // apply actual correction as Tom
    if(!correction_disabled)
    {
        Tom = poses[0] * ~Tbo;
    }
}

// Storing Pose information globally
// Calculate transformation from map to odom from pose in map frame
void poseCB(geometry_msgs::PoseStamped msg)
{
    std::lock_guard<std::mutex> guard(T_odom_map_mutex);

    ROS_INFO_STREAM_NAMED(ros::this_node::getName(), ros::this_node::getName() << " Received new pose guess");

    // rest max distance
    // corr_params.max_distance = max_distance;
    // scan_correct->setParams(corr_params);

    // std::cout << "poseCB" << std::endl;
    map_frame = msg.header.frame_id;
    pose_received = true;

    
    Transform Tpm;
    convert(msg.pose, Tpm);

    // total transform: offsert -> pose -> map
    Transform Tbm = Tpm * initial_pose_offset;

    // set T_base_map
    // geometry_msgs::TransformStamped T_base_map;
    // T_base_map.header.frame_id = map_frame;
    // T_base_map.child_frame_id = base_frame;
    // convert(Tbm, T_base_map.transform);

    Tom = Tbm * ~Tbo;


    // fetchTF();
    // T_odom_map = T_base_map * ~T_base_odom;
}

void poseWcCB(geometry_msgs::PoseWithCovarianceStamped msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;

    poseCB(pose);
}


void correct()
{
    if(pose_received)
    {
        fetchTF();
        correctOnce();
    }
}

void init()
{
    T_odom_map.header.frame_id = map_frame;
    T_odom_map.child_frame_id = odom_frame;
    T_odom_map.transform.translation.x = 0.0;
    T_odom_map.transform.translation.y = 0.0;
    T_odom_map.transform.translation.z = 0.0;
    T_odom_map.transform.rotation.x = 0.0;
    T_odom_map.transform.rotation.y = 0.0;
    T_odom_map.transform.rotation.z = 0.0;
    T_odom_map.transform.rotation.w = 1.0;
    Tom = Transform::Identity();

    
    T_base_odom.header.frame_id = odom_frame;
    T_base_odom.child_frame_id = base_frame;
    T_base_odom.transform.translation.x = 0.0;
    T_base_odom.transform.translation.y = 0.0;
    T_base_odom.transform.translation.z = 0.0;
    T_base_odom.transform.rotation.x = 0.0;
    T_base_odom.transform.rotation.y = 0.0;
    T_base_odom.transform.rotation.z = 0.0;
    T_base_odom.transform.rotation.w = 1.0;
    Tbo = Transform::Identity();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "micp_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    double tf_rate = 50.0;
    double corr_rate_max = 200.0;
    bool print_corr_rate = false;
    base_frame = "base_footprint";
    odom_frame = "odom";
    map_frame = "map";

    adaptive_max_dist = true;



    nh_p.param<std::string>("base_frame", base_frame, "base_link");
    nh_p.param<std::string>("odom_frame", odom_frame, "odom");
    nh_p.param<std::string>("map_frame",  map_frame,  "map");

    nh_p.param<double>("tf_rate", tf_rate, 50.0);
    nh_p.param<bool>("invert_tf", invert_tf, false);

    nh_p.param<double>("micp/corr_rate_max", corr_rate_max, 10000.0);
    nh_p.param<bool>("micp/print_corr_rate", print_corr_rate, false);

    nh_p.param<bool>("micp/adaptive_max_dist", adaptive_max_dist, true);

    nh_p.param<bool>("micp/viz_corr", draw_correspondences, false);
    nh_p.param<bool>("micp/disable_corr", correction_disabled, false);

    initial_pose_offset = Transform::Identity();
    std::vector<double> trans, rot;
    
    if(nh_p.getParam("micp/trans", trans))
    {
        if(trans.size() != 3)
        {
            // error
        }
        initial_pose_offset.t.x = trans[0];
        initial_pose_offset.t.y = trans[1];
        initial_pose_offset.t.z = trans[2];
    }

    if(nh_p.getParam("micp/rot", rot))
    {
        if(rot.size() == 3)
        {
            EulerAngles e;
            e.roll = rot[0];
            e.pitch = rot[1];
            e.yaw = rot[2];
            initial_pose_offset.R.set(e);
        } else if(rot.size() == 4) {
            initial_pose_offset.R.x = rot[0];
            initial_pose_offset.R.y = rot[1];
            initial_pose_offset.R.z = rot[2];
            initial_pose_offset.R.w = rot[3];
        }
    }

    init();

    tf_buffer.reset(new tf2_ros::Buffer);
    tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer));

    micp = std::make_shared<MICP>();
    micp->loadParams();


    std::string combining_unit_str;
    nh_p.param<std::string>("micp/combining_unit", combining_unit_str, "cpu");

    if(combining_unit_str == "cpu")
    {
        // std::cout << "Combining Unit: CPU" << std::endl; 
        combining_unit = 0;
    } else if(combining_unit_str == "gpu") {
        // std::cout << "Combining Unit: GPU" << std::endl;
        combining_unit = 1;
    } else {
        // ERROR
        std::cout << "Combining Unit: " << combining_unit_str << " unknown!" << std::endl;
        return 0;
    }


    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, poseCB);
    ros::Subscriber pose_wc_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_wc", 1, poseWcCB);


    // CORRECTION THREAD
    stop_correction_thread = false;
    correction_thread = std::thread([corr_rate_max, print_corr_rate]()
    {
        StopWatch sw;
        double el;

        // minimum duration for one loop
        double el_min = 1.0 / corr_rate_max;

        // reactivate cuda context if required
        micp->useInThisThread();

        while(!stop_correction_thread)
        {
            sw();
            correct();
            el = sw();
            double el_left = el_min - el;
            if(el_left > 0.0)
            {
                std::this_thread::sleep_for(std::chrono::duration<double>(el_left));
            }

            if(print_corr_rate)
            {
                double total_dur = el;
                if(el_left > 0.0)
                {
                    total_dur += el_left;
                }
                std::cout << "- Current Correction Rate:  " << total_dur << " s" << ", " << 1.0/total_dur << " hz" << std::endl; 
                std::cout << "- Possible Correction Rate: " << el << " s" << ", " << 1.0/el << " hz" << std::endl;
            }
        }

        stop_correction_thread = false;
    });

    // MAIN LOOP TF
    ros::Rate r(tf_rate);
    ros::Time stamp = ros::Time::now();

    std::cout << "TF Rate: " << tf_rate << std::endl;

    std::cout << "Waiting for pose guess..." << std::endl;

    while(ros::ok())
    {
        if(pose_received)
        {
            // updateTF();
            // weird bug. new_stamp sometimes is equal to stamp. results 

            // if(print_corr_rate)
            // {
            //     std::cout << "- update tf" << std::endl;
            // }
            

            ros::Time new_stamp = ros::Time::now();
            if(new_stamp > stamp)
            {
                updateTF();
                stamp = new_stamp;
            }
        }
        
        r.sleep();
        ros::spinOnce();
    }

    stop_correction_thread = true;
    correction_thread.join();

    return 0;
}