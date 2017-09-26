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

#ifndef PROJECT_EXTRACTION_H
#define PROJECT_EXTRACTION_H


#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "ros/package.h"
#include <dirent.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "stdio.h"
#include <vector>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>



namespace robosense
{

struct poleslabel2
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cv::Point2f location;
  double time;
  int label;
};

struct poleslabel
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cv::Point3f center;
  int label;
};


class extraction
{
public:
  extraction(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~extraction(){}
  //入口函数


  void gridCallback(const sensor_msgs::PointCloud2& msg);
  void pushFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  void clipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                 float in_min_height, float in_max_height);

  void genGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,cv::Mat& mat);
  cv::Point2i trans(cv::Point2f pt);

  void icvprCcaBySeedFill(const cv::Mat& _binImg, cv::Mat& _lableImg);
  void extractTree(const cv::Mat label,cv::Mat& mat);
  void genClusters2(const cv::Mat label,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,std::vector<poleslabel>& poles);

  void euclideanDistance(std::vector<poleslabel>& poles,std::vector<pcl::PointCloud<pcl::PointXYZI> >& in_cluster);
  void matormat(const cv::Mat temp1,const cv::Mat temp2,cv::Mat& temp3);

  void publishpoles(const ros::Publisher* in_publisher,const pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud);
  void features(const pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud);
  void clusterstoFrames(const std::vector<pcl::PointCloud<pcl::PointXYZI> >in_cluster,
                        std::vector<poleslabel2>& curPoles, pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position);

  void showcylinder(std::vector<cv::Point2f> centers);
  void publishpoles_tree(const ros::Publisher* in_publisher,const pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud);


private:
  std::vector<pcl::PointCloud<pcl::PointXYZI> >in_cluster;
  std::vector<poleslabel2> curPoles;
  std::vector<poleslabel2> prePoles;

  double timeStam;
  std::map<int,int> asso;
  std::vector<cv::Point2f> centers;
  std::vector<int> indices;

  std_msgs::Header robosense_header;
  ros::Subscriber sub_tracklets;
  ros::Publisher pub_poles;
  ros::Publisher marker_pub;
  ros::Publisher pub_poles_track;
  ros::Publisher pub_poles_position;

  //特征数据
  std::vector<float> feature;//30=1+6+3+20
  std::vector<float> Pointnum;//1
  std::vector<float> Cov_mat;//6
  std::vector<float> Local_pose;//3
  std::vector<float> Slice_mat;//10
  float cov_scalar;

  std::ofstream featureFile;

  std::vector<cv::Scalar> _colors;
  int colorsNum,countPoints;
  float countPoints_top,countPoints_bottom,scalarPoints;

  double gridMiH;
  double gridMaH;
  int dilation_size;
  cv::Mat grid,grid_2,grid_3,temp_1,temp_2;
  double gWidth,gHeight;
  double miniGrid;
  int gridW,gridH;

  int frames;

  float in_clip_min_height,in_clip_max_height;
  float in_clip_min_height_2,in_clip_max_height_2;


  pcl::PointCloud<pcl::PointXYZI> last;   //存储局部百帧子地图
  std::vector<int> last_num;  //存储自地图对应单帧点云的点数
  pcl::PointCloud<pcl::PointXYZI> last_frame;
  int frame_num ;

};

}
#endif //PROJECT_extraction_H
