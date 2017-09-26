/*************************************************************************
Copyright: 2017 RoboSense(Suteng Innovation Technology Co., Ltd.) Inc
License: RoboSense-proprietary

 .
 All rights not expressly granted by the Licensor remain reserved.
 .
 This unpublished material is proprietary to RoboSense Inc.
 .
 Proprietary software is computer software licensed under exclusive legal
 right of the copyright holder. The receipt or possession of this source
 code and/or related information does not convey or imply any rights to use,
 reproduce, disclose or distribute its contents, or to manufacture, use, or
 sell anything that it may describe, in whole or in part unless prior written
 permission is obtained from RoboSense Inc.
 .
 The methods and techniques described herein are considered trade secrets
 and/or confidential. You shall not disclose such Confidential Information
 and shall use it only in accordance with the terms of the license agreement
 you entered into with RoboSense Inc.
 .
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES.
***************************************************************************/

#ifndef PROJECT_LOCALIZATION_H
#define PROJECT_LOCALIZATION_H



#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <iomanip>
#include <stack>
#include <vector>
#include <math.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "ros/package.h"
#include <dirent.h>
#include <opencv/cv.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/time.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <ros/console.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <robosense_localization/canbus.h>
#include <robosense_localization/encoders.h>

#include "pointmatcher/PointMatcher.h"
#include <cassert>

#include "boost/filesystem.hpp"
#include <boost/interprocess/smart_ptr/unique_ptr.hpp>
#include <boost/assign.hpp>
#include "point_cloud.h"
#include "transform.h"
#include <boost/thread/thread.hpp>
//#include "pointmatcher_ros/get_params_from_server.h"
//#include "ros_logger.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
//#include "pointmatcher_ros/get_params_from_server.h"
//#include "pointmatcher_ros/ros_logger.h"


#define COUT(X)  std::cout<< X << std::endl
#define COUTG(X) std::cout<<"\033[1;32m "<< X <<"\033[0m" << std::endl
#define COUTR(X) std::cout<<"\033[1;31m "<< X <<"\033[0m" << std::endl

namespace robosense
{

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::Imu,robosense_localization::encoders,sensor_msgs::PointCloud2> slamSyncPolicy;

struct pairs
{
  cv::Point2f pole;
  cv::Point2f map_pole;
  float distance;
};

struct double_pairs
{
  cv::Point2f xa_1;
  cv::Point2f XA_1;
  cv::Point2f xa_2;
  cv::Point2f XA_2;
  float theta;
};

struct distance
{
  cv::Point2f a;
  cv::Point2f b;
  float dis;
};

struct match
{
  cv::Point2f A;
  cv::Point2f B;
  cv::Point2f a;
  cv::Point2f b;
  float dist;
  int count;
};

struct loc_map_distance
{
  int lables;
  float distance;
};


#define PI 3.1415926

typedef Eigen::Vector3f pose;

typedef  PointMatcher<float > PM;
typedef  PM::DataPoints DP;


class localization
{
public:

  localization(ros::NodeHandle nh, ros::NodeHandle private_nh,
               std::string map_path);
  ~localization() {}

  void init();
  void callback(const sensor_msgs::PointCloud2ConstPtr &poles_msg,
                const sensor_msgs::ImuConstPtr &pose_msg,
                const robosense_localization::encodersConstPtr &vel_msg,
                const sensor_msgs::PointCloud2ConstPtr& rslidar_msg);
  void measurementUpdate(std::vector<cv::Point2f> map, pose estimate_pose,
                         std::vector<cv::Point2f> poles, pose &measure_pose, bool &success);
  bool loadMap(std::string map_str);
  void showCylinder(pcl::PointCloud<pcl::PointXYZ> centers);
  void DisplayIDs(pcl::PointCloud<pcl::PointXYZ> centers);
  void displayFeatureMap(pcl::PointCloud<pcl::PointXYZ> centers);
  void publishMap(const ros::Publisher *in_publisher, pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud);
  void kdTreeRadiusSearch(Eigen::Vector3f estimate_pose, float search_radius,
                          std::vector<cv::Point2f> &resultant_points);
  void transform(std::vector<cv::Point2f> map, std::vector<cv::Point2f> trans_points,
                 Eigen::Vector3f estimate_pose, Eigen::Vector3f &rotated_pose);
  void gps_callback(const sensor_msgs::NavSatFix& gps_msg);
  Eigen::Vector3d WGS84toECEF(Eigen::Vector3d gps);
  void readTxt(std::string txt_path,std::vector<cv::Point2f>& loc_point_vec);
  void loadLocMap(void);
  void visualizeEstimatePath(const sensor_msgs::ImuConstPtr& pose_msg, pose estimate_pose);
  void visualizeGPSPath(const sensor_msgs::ImuConstPtr& pose_msg, pose estimate_pose);

private:
  message_filters::Subscriber<sensor_msgs::PointCloud2> *poles_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *rslidar_sub;
  message_filters::Subscriber<sensor_msgs::Imu> *pose_sub;
  message_filters::Subscriber<robosense_localization::encoders> *vel_sub;
  message_filters::Synchronizer<slamSyncPolicy> *sync_;


  ros::Publisher path_pub;
  ros::Publisher marker_pub;
  ros::Publisher marker_pub2;
  ros::Publisher map_pub;
  ros::Publisher mapping_odom_pub;
  ros::Publisher markerID_pub;
  ros::Publisher gps_path_pub;
  ros::Publisher map_origin_pub;
  ros::Subscriber sub_gps;//gps接受

  std::vector<pairs> pairs_poles;
  std::vector<cv::Point3f> map_3d;//地图储存变量
  std::vector<cv::Point2f> map;
  std::vector<cv::Point2f> poles_points;
  tf::TransformBroadcaster odom_broadcaster;


  Eigen::Vector3d XYZ;//gps中间转换变量
  Eigen::Vector3d gps_origin;//gps初始点


  nav_msgs::Path path;
  nav_msgs::Path gps_path; //GPS路径消息
  pose predict_pose;
  pose predict_pose2;

  pose estimate_pose;
  pose estimate_pose2;

  pose measure_pose;
  pose last_pose;


  DP ref_loc_map;
  std::vector<DP> loc_maps;

  Eigen::Matrix3f Af;
  Eigen::Matrix3f Af2;

  Eigen::Matrix3f Q;
  Eigen::Matrix3f Q2;//二次卡尔曼预测协方差矩阵

  Eigen::Matrix3f R;//一次卡尔曼测量协方差矩阵
  Eigen::Matrix3f R2;//二次卡尔曼测量协方差矩阵

  Eigen::Matrix3f pred_true_Covariance;//一次卡尔曼预测值与真实值之间的协方差矩阵
  Eigen::Matrix3f pred_true_Covariance2;//二次卡尔曼预测值与真实值之间的协方差矩阵

  Eigen::Matrix3f esti_true_Covariance;//1次卡尔曼测量值与真实值之前的协方差矩阵
  Eigen::Matrix3f esti_true_Covariance2;//2次卡尔曼测量值与真实值之前的协方差矩阵

  Eigen::Matrix3f k_filter;//一次卡尔曼系数矩阵
  Eigen::Matrix3f k_filter2;//二次卡尔曼系数矩阵

  float cur_velocity;
  float last_velocity;
  float new_velocity;
  float cur_theta;
  float last_theta;

  double cur_poses_time;
  double last_poses_time;
  double cur_theta_time;
  double last_theta_time;
  double cur_vel_time;
  bool get_gps;//gps信号标志位


  bool success;//车辆定位函数中的标志位
  bool initial;//初始化标志位
  bool gps_msg;
  bool odom_map_msg;
  bool calibration;//图匹配标志位

  int idx, map_idx;
  std::string map_str;//地图pcd保存路径

  std::string fileName;//车辆轨迹保存为pcd文件路径
  pcl::PointCloud<pcl::PointXYZ> trajectory;//车辆轨迹保存pcd
  std::string fileName2;//车辆轨迹保存为pcd文件路径
  pcl::PointCloud<pcl::PointXYZI> trajectory2;//车辆轨迹保存pcd

  int no_ekf;//卡尔曼空跑次数
  int locmap_lable;


  pcl::PointCloud<pcl::PointXYZ> tree_map;
  int frames;//当前运行帧数

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_save;

  PM::DataPointsFilters current_filter_;
  PM::DataPointsFilters locmap_filter_;
  std::string loc_map_str;
  std::string odom_path_str;

  std::vector<cv::Point2f> loc_points;

  PM::ICP icp_;

  Eigen::Matrix4f lidar_to_map_;

  //  boost::movelib::unique_ptr<PM::Transformation> transformation_;

}; //localization




} //robosense

#endif //PROJECT_LOCALIZATION_H
