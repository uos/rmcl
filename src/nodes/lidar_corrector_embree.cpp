#include <ros/ros.h>
#include <rmagine/map/EmbreeMap.hpp>

#include <rmcl/correction/LiDARCorrectorEmbree.hpp>

using namespace rmagine;
using namespace rmcl;


Memory<LiDARModel, RAM> velodyne_model()
{
    Memory<LiDARModel, RAM> model(1);
    model->theta.min = -M_PI;
    model->theta.inc = 0.4 * M_PI / 180.0;
    model->theta.size = 900;

    model->phi.min = -15.0 * M_PI / 180.0;
    model->phi.inc = 2.0 * M_PI / 180.0;
    model->phi.size = 16;
    
    model->range.min = 0.1;
    model->range.max = 130.0;
    return model;
}

/**
 * @brief use LiDARCorrectorEmbree in ROS
 * 
 * For convenience do this automatically:
 * - setTsb from TF
 * - setInputData from ROS topic
 * 
 */
class LiDARCorrectorEmbreeROS {
public:
    LiDARCorrectorEmbreeROS()
    {

    }

protected:
    // LiDARCorrectorEmbreePtr m_corrector;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_corrector_embree");

    ROS_INFO("LiDAR Corrector Embree started.");

    EmbreeMapPtr map = importEmbreeMap("/home/amock/workspaces/ros/mamcl_ws/src/uos_tools/uos_gazebo_worlds/Media/models/avz_neu.dae");

    ROS_INFO("Map loaded.");


    LiDARCorrectorEmbree corrector(map);

    auto model = velodyne_model();
    corrector.setModel(model);




    

    return 0;
}