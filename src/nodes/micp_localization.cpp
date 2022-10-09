#include <ros/ros.h>
#include <rmcl/correction/MICP.hpp>

using namespace rmcl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "micp_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    

    MICPPtr micp = std::make_shared<MICP>();

    micp->loadParams();


    ros::spin();

    return 0;
}