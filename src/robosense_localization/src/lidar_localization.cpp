#include "lidar_localization.h"


LidarLocalization::LidarLocalization(ros::NodeHandle node, ros::NodeHandle private_nh)
{

}



int main(int argc, char** argv)
{

    ros::init(argc, argv, "lidar_localization");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");


    std::string map_path;
    private_nh.getParam("map_path", map_path);

    extraction poles(nh, private_nh);         //extract features for localization
    localization loc(nh, private_nh, map_path); //provide localization information

    ros::spin();

    return 0;

}
