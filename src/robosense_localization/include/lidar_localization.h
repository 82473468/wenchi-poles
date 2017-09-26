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

#include "extraction.h"
#include "localization.h"



using namespace robosense;

class LidarLocalization
{
public:
    LidarLocalization(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~LidarLocalization() {}
};
