#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Core>
#include <math.h>
#include <iostream>

nav_msgs::Path gps_path;
ros::Publisher  gps_path_pub;
ros::Publisher GPS_origin_pub;

bool initialization = false;
Eigen::Vector3d XYZ;

double ori_lat;
double ori_lon;
double ori_ati; 

Eigen::Vector3d WGS84toECEF(Eigen::Vector3d gps)
{
  double SEMI_MAJOR_AXIS = 6378137.0;
  double RECIPROCAL_OF_FLATTENING = 298.257223563;
  double SEMI_MINOR_AXIS = 6356752.3142;
  double FIRST_ECCENTRICITY_SQUARED = 6.69437999014e-3;
  double SECOND_ECCENTRICITY_SQUARED = 6.73949674228e-3;

  double lon = gps[0];
  double lat = gps[1];
  double ati = gps[2];

  double rad_lon = lon/180*M_PI;
  double rad_lat = lat/180*M_PI;

  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  double chi = sqrt(1.0-FIRST_ECCENTRICITY_SQUARED*sin_lat*sin_lat);
  double   N = SEMI_MAJOR_AXIS/chi+ati;

  Eigen::Vector3d ret;
  ret<< N*cos_lat*cos_lon,
        N*cos_lat*sin_lon,
        (SEMI_MAJOR_AXIS*(1.0-FIRST_ECCENTRICITY_SQUARED)/chi+ati)*sin_lat;

  return ret;
}


void GPSCallBack(const sensor_msgs::NavSatFix &gps_msg)
{


  if(!initialization)
  {
    ori_lat = gps_msg.latitude;
    ori_lon = gps_msg.longitude;
    ori_ati = gps_msg.altitude;
    initialization = true;


    sensor_msgs::NavSatFix fix_;
    fix_.header.stamp = ros::Time().now();
    fix_.header.frame_id = "rslidar";

    fix_.latitude  = ori_lat;
    fix_.longitude = ori_lon;
    GPS_origin_pub.publish(fix_);
  }

  Eigen::Vector3d gps;
  gps<< gps_msg.longitude,
        gps_msg.latitude,
        gps_msg.altitude;

  Eigen::Vector3d ret;
  ret = WGS84toECEF(gps);

  double rad_lon = ori_lon/180*M_PI;
  double rad_lat = ori_lat/180*M_PI;
  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  Eigen::Matrix3d rot;

  rot<<-sin_lon,                 cos_lon,       0,
       -sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat,
        cos_lat*cos_lon,  cos_lat*sin_lon,       0;

  Eigen::Vector3d z0;
  Eigen::Vector3d wgs0;
  wgs0<<ori_lon,
        ori_lat,
        ori_ati;

  z0 = WGS84toECEF(wgs0);
  Eigen::Vector3d l;
  l = ret-z0;
  XYZ = rot*l;

  std::cout<<"XYZ: "<<XYZ.transpose()<<std::endl;

  //---------------------------------------------
  //visualize the GPS path...

  geometry_msgs::PoseStamped odom2;

  odom2.header.stamp = gps_msg.header.stamp;
  odom2.header.frame_id = "rslidar";

  //set the position
  odom2.pose.position.x =  XYZ[0];
  odom2.pose.position.y =  XYZ[1];
  odom2.pose.position.z = 0.0;

  gps_path.poses.push_back(odom2);
  gps_path_pub.publish(gps_path);
}



int main(int argc, char** argv)
{

  ros::init(argc, argv, "visualize_GPS_path");
  ros::NodeHandle nh;

  ros::Subscriber sub_gps = nh.subscribe("/gps/fix", 10, GPSCallBack);
  gps_path_pub = nh.advertise<nav_msgs::Path>("/gps_path",1,true);
  gps_path.header.frame_id = "rslidar";


  GPS_origin_pub =  nh.advertise<sensor_msgs::NavSatFix>("/rs_GPS_origin", 1, true);

  ros::spin();
  return 0;

}












