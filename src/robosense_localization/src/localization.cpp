//
// Created by wenchi on 17-7-12.
//

#include "localization.h"


#define COUT(X)  std::cout<< X << std::endl
#define COUTG(X) std::cout<<"\033[1;32m "<< X <<"\033[0m" << std::endl
#define COUTR(X) std::cout<<"\033[1;31m "<< X <<"\033[0m" << std::endl
#define SQR(X)   powf(X, 2))


namespace robosense
{
/***********************************************************************************
 *
 * *********************************************************************************/
localization::localization(ros::NodeHandle nh, ros::NodeHandle private_nh, std::string map_path)
{
  poles_sub = new  message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/rs_features",10);
  pose_sub=new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/nav440/nav440", 10);
  vel_sub=new message_filters::Subscriber<robosense_localization::encoders>(nh, "/encoder/encoders", 10);
  rslidar_sub= new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/rslidar_points_global",10);
  sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(10), *poles_sub, *pose_sub, *vel_sub, *rslidar_sub);
  sync_->registerCallback(boost::bind(&localization::callback, this,  _1,  _2, _3, _4));

  sub_gps = nh.subscribe("/gps/fix", 10, &localization::gps_callback, (localization*) this);

  path_pub = nh.advertise<nav_msgs::Path>("/rs_path", 1, true);
  gps_path_pub = nh.advertise<nav_msgs::Path>("/rs_gps_path",1,true);
  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/rs_map", 1, true);
  map_origin_pub =  nh.advertise<sensor_msgs::NavSatFix>("/rs_map_origin", 1, true);
  mapping_odom_pub = nh.advertise<sensor_msgs::PointCloud2>("/rs_mapping_odom", 1, true);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("Tree_map", 1);
  marker_pub2 = nh.advertise<visualization_msgs::MarkerArray>("Tree_instant", 1);
  markerID_pub = nh.advertise<visualization_msgs::MarkerArray>("Map_IDs", 1, true);
  
  map_str = map_path;

  success = false;
  initial = false;
  get_gps = false;
  calibration = false;

  esti_true_Covariance<<100,   0,   0,
                          0, 100,   0,
                          0,   0,   1;

  esti_true_Covariance2<<100,   0,  0,
                           0, 100,  0,
                           0,   0,  1;

  estimate_pose <<0, 0, 0.24132;
  estimate_pose2<<0, 0, 0.24132;

  path.header.frame_id = "rs_odom";
  gps_path.header.frame_id = "rs_odom";

  idx = 0;
  map_idx = -1;
  no_ekf = 0;
  frames = 0;

  time_t t = time(0);   // get time now
  struct tm * now = localtime( & t );
  char buffer [80];
  strftime (buffer,80,"%Y%m%d-%H%M",now);

  std::string pkg_Path = ros::package::getPath("robosense_localization");
  std::string folder_path = pkg_Path + "/data/path_records/";

  if(!fopen(folder_path.c_str(), "wb"))
  {
    mkdir(folder_path.c_str(), S_IRWXU);
  }

  fileName = folder_path + "industrialZone_" + std::string(buffer)  + "-X1.pcd";
//  fileName2 = folder_path + "mapping_odom_" + std::string(buffer)  + ".pcd";

  init();

}



/***********************************************************************************
 *
 * *********************************************************************************/
void localization::callback(const sensor_msgs::PointCloud2ConstPtr& poles_msg,
                            const sensor_msgs::ImuConstPtr& pose_msg,
                            const robosense_localization::encodersConstPtr &vel_msg,
                            const sensor_msgs::PointCloud2ConstPtr& rslidar_msg)
{
  frames++;

  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZ> poles_tmp;
  pcl::PointCloud<pcl::PointXYZI> tmp_I;
  pcl_conversions::toPCL(*poles_msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, tmp_I);
  pcl::copyPointCloud(tmp_I, poles_tmp);
  poles_points.clear();

  for (int i = 0; i < poles_tmp.size(); ++i)
  {
    if(fabs(poles_tmp.points[i].x)>999)
      continue;  //add this line to make sure the message synchronization, bcs even there is no feature, we still publish header message
    else
      poles_points.push_back(cv::Point2f(poles_tmp.points[i].x, poles_tmp.points[i].y));
  }

  if (get_gps && !initial&&(poles_points.size()>5))
  {
    if(gps_msg)
    {
      estimate_pose << XYZ[0], XYZ[1], XYZ[2];
      gps_msg = false;
    }

    //        COUTG("Intializing....."<< estimate_pose.transpose());

    Eigen::Vector3f rotated_pose;
    Eigen::Vector3f last_rotated_pose;

    last_rotated_pose=estimate_pose;
    transform(map, poles_points, estimate_pose, rotated_pose);

    estimate_pose = rotated_pose;
    estimate_pose2 = estimate_pose;

    if(fabs(estimate_pose[0]-last_rotated_pose[0])+fabs(estimate_pose[1]-last_rotated_pose[1])<5)
    {
      initial = true;
      calibration = false;
      COUTR(" ~~ Congratulations! The initialization was done!\n\n");
    }
  }
  else if(!get_gps && !initial)
  {
    COUTR("Warning! Cannot obtain GPS coordinates for initialization...");
  }

  cur_poses_time = vel_msg->header.stamp.sec + vel_msg->header.stamp.nsec * 1e-9;
  cur_velocity = (vel_msg->lfront+vel_msg->rfront)*0.54*PI/(2.*60.);
  double diff_time = cur_poses_time - last_poses_time;
  geometry_msgs::Quaternion q=pose_msg->orientation;

  cur_theta = -tf::getYaw(q);
  cur_theta_time = pose_msg->header.stamp.toSec();

  if (diff_time < 10)
  {
    last_pose = estimate_pose;
    //*******车前进的距离
    float inc_distance = (cur_velocity + last_velocity) / 2 * diff_time;
    float diff_theta = cur_theta - last_theta;
    diff_theta = atan2f(sinf(diff_theta),cosf(diff_theta));

    if (fabs(diff_theta) > 1)
    {
      diff_theta = 0;
    }

    //×××××××车身坐标系中车前进的距离
    Eigen::Vector3f inc_distance_car;
    inc_distance_car<<inc_distance*cosf(diff_theta/2.),
                      inc_distance*sinf(diff_theta/2.),
                      diff_theta;

    //*******车身坐标系相对大地坐标系的旋转矩阵
    Eigen::Matrix3f car_rotate;
    car_rotate<<cosf(estimate_pose[2]), -sinf(estimate_pose[2]), 0,
                     sinf(estimate_pose[2]),  cosf(estimate_pose[2]), 0,
                     0, 0, 1;

    Eigen::Matrix3f car_rotate2;
    car_rotate2<<cosf(estimate_pose2[2]), -sinf(estimate_pose2[2]), 0,
                      sinf(estimate_pose2[2]),  cosf(estimate_pose2[2]), 0,
                      0, 0, 1;

    //××××××××××车在大地坐标系中前进的距离
    Eigen::Vector3f inc_distance_global;
    inc_distance_global = car_rotate*inc_distance_car;
    estimate_pose += inc_distance_global;
    estimate_pose[2] = atan2f(sinf(estimate_pose[2]), cosf(estimate_pose[2]));

    Eigen::Vector3f inc_distance_global2;
    inc_distance_global2 = car_rotate2*inc_distance_car;
    estimate_pose2 += inc_distance_global2;
    estimate_pose2[2] = atan2f(sinf(estimate_pose2[2]),cosf(estimate_pose2[2]));

    Af << 1, 0, -inc_distance * sinf(estimate_pose[2] + diff_theta/2.0),
          0, 1,  inc_distance * cosf(estimate_pose[2] + diff_theta/2.0),
          0, 0, 1;

    Q << 2 * diff_time, 0, 0,
         0, 2 * diff_time, 0,
         0, 0, 2 * diff_time;

    R << 5, 0, 0,
         0, 5, 0,
         0, 0, 0.5;

    Af2 << 1, 0, -inc_distance * sinf(estimate_pose2[2] + diff_theta / 2.),
           0, 1, inc_distance * cosf(estimate_pose2[2] + diff_theta / 2),
           0, 0, 1;

    Q2 << 10 * diff_time, 0, 0,
          0, 10 * diff_time, 0,
          0, 0, 1 * diff_time;

    R2 << 5, 0, 0,
          0, 5, 0,
          0, 0, 0.5;

    pred_true_Covariance = Af * esti_true_Covariance * Af.transpose() + Q;
    pred_true_Covariance2 = Af2 * esti_true_Covariance2 * Af2.transpose() + Q2;

    std::vector<cv::Point2f> loc_map;

    bool icp_active = false;
    float compare_dist = 99;
    for(int i = 0; i<loc_points.size(); ++i)
    {
      float distance_loc = sqrtf(powf(estimate_pose[0] - loc_points[i].x, 2) + powf(estimate_pose[1] - loc_points[i].y, 2));
      loc_map_distance temp_loc_map;
      temp_loc_map.lables = i;
      temp_loc_map.distance = distance_loc;

      if(distance_loc<compare_dist)
      {
        compare_dist = distance_loc;
        locmap_lable = i;
      }

      if (distance_loc < 10)
      {
        icp_active = true;
      }
    }

    if(icp_active)
    {
      loadLocMap();
    }

//    pcl::console::TicToc tt;
//    tt.tic ();

    if(icp_active&&frames%2==0)
    {
      COUTG("\n\n  ~~ ICP is active!");
      DP current_pointcloud_;
      current_pointcloud_ = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*rslidar_msg);
      current_filter_.apply(current_pointcloud_);

      lidar_to_map_ = Eigen::Matrix4f::Identity();
      lidar_to_map_(0, 0) = cosf(estimate_pose2[2]);
      lidar_to_map_(0, 1) =-sinf(estimate_pose2[2]);
      lidar_to_map_(1, 0) = sinf(estimate_pose2[2]);
      lidar_to_map_(1, 1) = cosf(estimate_pose2[2]);
      lidar_to_map_(0, 3) =       estimate_pose2[0];
      lidar_to_map_(1, 3) =       estimate_pose2[1];

      pcl::console::TicToc tt2;
      tt2.tic ();

      lidar_to_map_ = icp_(current_pointcloud_, ref_loc_map, lidar_to_map_);

      std::cout << "  ~~ [ICP was done in \033[1;32m" << tt2.toc () << " \033[0m ms ]" << std::endl;

      lidar_to_map_(0,2) = 0.0f;
      lidar_to_map_(1,2) = 0.0f;
      lidar_to_map_(2,0) = 0.0f;
      lidar_to_map_(2,1) = 0.0f;
      lidar_to_map_(2,2) = 1.0f;
      //          lidar_to_map_ = transformation_->correctParameters(lidar_to_map_);
      float temp_yaw = atan2f(lidar_to_map_(1, 0),lidar_to_map_(1, 1));
      measure_pose << lidar_to_map_(0, 3), lidar_to_map_(1, 3),temp_yaw;
      success = true;
    }
    else
    {
      kdTreeRadiusSearch(estimate_pose2, 30, loc_map);
      measurementUpdate(loc_map, estimate_pose2, poles_points, measure_pose, success);
    }

//    COUTG("[ICP time consuming...., " << tt.toc () << " ms ]" );

    if (success)
    {
      predict_pose = estimate_pose;
      k_filter = pred_true_Covariance * (pred_true_Covariance + R).inverse();
      estimate_pose = predict_pose + k_filter * (measure_pose - predict_pose);
      float mea_yaw = measure_pose[2];
      float pre_yaw = predict_pose[2];
      float k_yaw = k_filter(2, 2);
      estimate_pose[2] = atan2f(k_yaw * sinf(mea_yaw) + (1 - k_yaw) * sinf(pre_yaw),
                                k_yaw * cosf(mea_yaw) + (1 - k_yaw) * cosf(pre_yaw));
      esti_true_Covariance = (Eigen::Matrix3f::Identity() - k_filter) * pred_true_Covariance;

      success = false;
      no_ekf = 0;
    }
    else
    {
      std::cout<<"EKF is no！"<<","<<no_ekf<<std::endl;
      esti_true_Covariance = pred_true_Covariance;
      if(poles_points.size() > 3)
      {
        no_ekf++;
      }

    }

    if(no_ekf>27)
    {
      calibration = true;
    }

    if (initial && calibration && poles_points.size() > 5)
    {
      Eigen::Vector3f  rotated_pose;
      Eigen::Vector3f last_rotated_pose;
      last_rotated_pose = estimate_pose;
        COUTR(" The graph matched algorithm was triggered!!!");

      if(gps_msg&&sqrt(pow((estimate_pose[0]-XYZ[0]),2) + pow((estimate_pose[1]-XYZ[1]),2)) >20)
      {
        estimate_pose << XYZ[0], XYZ[1], 0;
      }

      transform(map, poles_points, estimate_pose, rotated_pose);
      estimate_pose = rotated_pose;
      estimate_pose2 = estimate_pose;

      if (fabs((estimate_pose[0] - last_rotated_pose[0]) + (estimate_pose[1] - last_rotated_pose[1])) < 5)
      {
        calibration = false;
        no_ekf = 0;
      }
    }

    //*******第二次卡尔曼进行平滑
    predict_pose2 = estimate_pose2;
    k_filter2 = pred_true_Covariance2 * (pred_true_Covariance2 + R2).inverse();
    estimate_pose2 = predict_pose2 + k_filter2 * (estimate_pose - predict_pose2);

    float mea_yaw2 = estimate_pose[2];
    float pre_yaw2 = predict_pose2[2];
    float k_yaw2 = k_filter2(2, 2);
    estimate_pose2[2] = atan2f(k_yaw2 * sinf(mea_yaw2) + (1 - k_yaw2) * sinf(pre_yaw2),
                               k_yaw2 * cosf(mea_yaw2) + (1 - k_yaw2) * cosf(pre_yaw2));

    esti_true_Covariance2 = (Eigen::Matrix3f::Identity() - k_filter2) * pred_true_Covariance2;

    float distance_inc;
    distance_inc = sqrtf( powf((estimate_pose2[0]- predict_pose2[0]), 2) + powf((estimate_pose2[1] - predict_pose2[1]),2));

    esti_true_Covariance2 = (Eigen::Matrix3f::Identity() - k_filter2) * pred_true_Covariance2;

    if(initial && !calibration)
    {
      //-------------------------------------------
      visualizeEstimatePath(pose_msg, estimate_pose2);
      //-------------------------------------
      //visualize the GPS path...
      visualizeGPSPath(pose_msg, estimate_pose2);
    }

  }
  last_theta = cur_theta;
  last_velocity = cur_velocity;
  last_poses_time = cur_poses_time;
  last_theta_time = cur_theta_time;

}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::loadLocMap(void)
{
  if(locmap_lable!=map_idx)
  {
    pcl::console::TicToc tt;
    tt.tic ();
    std::cout << "  ~~ The locmap_lable is: " << locmap_lable << std::endl;
    ref_loc_map = loc_maps[locmap_lable];
    locmap_filter_.apply(ref_loc_map);
    std::cout << "  ~~ [Map loaded in,\033[1;31m " << tt.toc () << "\033[0m ms ]" << std::endl;
    std::cout << "  ~~ Switch loc map succeed!" << std::endl;
    map_idx = locmap_lable;
  }
}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::visualizeEstimatePath(const sensor_msgs::ImuConstPtr& pose_msg, pose estimate_pose)
{
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = pose_msg->header.stamp;
  odom_trans.header.frame_id = "rs_odom";
  odom_trans.child_frame_id = "base_link2";
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(estimate_pose[2]);

  odom_trans.transform.translation.x = estimate_pose[0];
  odom_trans.transform.translation.y = estimate_pose[1];
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster.sendTransform(odom_trans);

  geometry_msgs::PoseStamped odom;

  odom.header.stamp = pose_msg->header.stamp;
  odom.header.frame_id = "rs_odom";

  //set the position
  odom.pose.position.x = estimate_pose[0];
  odom.pose.position.y = estimate_pose[1];
  odom.pose.position.z = 0.0;
  odom.pose.orientation = odom_quat;
  path.poses.push_back(odom);
  path_pub.publish(path);

  pcl::PointXYZ tmp_point;
  tmp_point.x = estimate_pose[0];
  tmp_point.y = estimate_pose[1];
  tmp_point.z = 0;

  trajectory.push_back(tmp_point);
  trajectory.height = 1;
  trajectory.width = trajectory.size();
  pcl::io::savePCDFileBinary(fileName, trajectory);
}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::visualizeGPSPath(const sensor_msgs::ImuConstPtr& pose_msg, pose estimate_pose)
{
  geometry_msgs::PoseStamped odom2;

  odom2.header.stamp = pose_msg->header.stamp;
  odom2.header.frame_id = "rs_odom";

  //set the position
  geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(0);
  odom2.pose.position.x = XYZ[0] + 0.345 * cosf(estimate_pose[2]);
  odom2.pose.position.y = XYZ[1] + 0.345 * sinf(estimate_pose[2]);
  odom2.pose.position.z = 0.0; //XYZ[2];
  odom2.pose.orientation = odom_quat2;
  gps_path.poses.push_back(odom2);
  gps_path_pub.publish(gps_path);

}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::showCylinder(pcl::PointCloud<pcl::PointXYZ> centers)
{
  visualization_msgs::MarkerArray markers;
  markers.markers.clear();
  for(int i = 0;i<centers.size();++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "rs_odom";
    marker.ns = "basic_shapes";
    marker.id  = idx;
    marker.type = visualization_msgs::Marker::CYLINDER;

    marker.pose.position.x = centers[i].x;
    marker.pose.position.y = centers[i].y;
    marker.pose.position.z = 2.5;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 8;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration(0.2);
    idx++;
    markers.markers.push_back(marker);
  }

  marker_pub2.publish(markers);

}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::displayFeatureMap(pcl::PointCloud<pcl::PointXYZ> centers)
{

  visualization_msgs::MarkerArray markers;
  markers.markers.clear();
  for(int i = 0;i<centers.size();++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "rs_odom";
    marker.ns = "basic_shapes";
    marker.id  = i;
    marker.type = visualization_msgs::Marker::CYLINDER;

    marker.pose.position.x = centers[i].x;
    marker.pose.position.y = centers[i].y;
    marker.pose.position.z = 2.5;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 10;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration(0);
    idx++;
    markers.markers.push_back(marker);
  }

  marker_pub.publish(markers);
}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::DisplayIDs(pcl::PointCloud<pcl::PointXYZ> centers)
{
  visualization_msgs::MarkerArray  markers;
  markers.markers.clear();

  for(int i = 0;i<centers.size();++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "rs_odom";
    marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = centers[i].x;
    marker.pose.position.y = centers[i].y;
    marker.pose.position.z = 7.5;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;


    marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 2;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    std::stringstream si;
    std::string str;
    si<<i;
    si>>str;
    marker.text  = str;

    marker.lifetime = ros::Duration(0);
    markers.markers.push_back(marker);
  }

  markerID_pub.publish(markers);
}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::init()
{
  std::string pkg_Path = ros::package::getPath("robosense_localization");
  std::string current_filter_file=pkg_Path+"/cfg/current_filter.yaml";
//  std::string icp_filter_file=pkg_Path+"/cfg/SamplingSurfaceNormalDataPointsFilter3.yaml";
  std::string icp_filter_file=pkg_Path+"/cfg/icp_filter.yaml";
  std::string locmap_filter_file=pkg_Path+"/cfg/localmap_filter.yaml";

  std::ifstream current_ifs(current_filter_file.c_str());
  current_filter_ = PM::DataPointsFilters(current_ifs);

  std::ifstream locmap_ifs(locmap_filter_file.c_str());
  locmap_filter_  = PM::DataPointsFilters(locmap_ifs);

  std::ifstream icp_ifs(icp_filter_file.c_str());
  icp_.loadFromYaml(icp_ifs);

  COUTG(" ~~ The configuration files were loaded successfully!");

  loadMap(map_str);
  COUTR(" ~~ ================================== ~~ \n\n");
}


/***********************************************************************************
 *
 * *********************************************************************************/
bool localization::loadMap(std::string map_str)
{
  map.clear();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr display_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr mapping_odom_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  DP all_map(DP::load(map_str));
  DP background = all_map.createSimilarEmpty();
  int new_points = 0;
  map_cloud_save.reset(new pcl::PointCloud<pcl::PointXYZ>);
//  for(int i=0;i<all_map.features.cols();++i)
//  {
//    if(all_map.features(3,i)==1)
//    {
//      std::cout<<"1"<<std::endl;
//    } else{
//      std::cout<<"get the important points!"<<std::endl;
//      new_points++;
//    }
//  }

//  std::cout<<"all have points is "<<all_map.features.cols()<<","<<new_points<<std::endl;

  for(int i=0;i<all_map.features.cols();++i)
  {
    if(all_map.features(2,i)>-1.01||all_map.features(2,i)<-1.09)
    {
      background.setColFrom(new_points,all_map,i);
      new_points++;
    }
    else if(all_map.features(2,i)<-1.01&&all_map.features(2,i)>-1.021)
    {
      map.push_back(cv::Point2f(all_map.features(0,i),all_map.features(1,i)));

      pcl::PointXYZ tmp_point;
      tmp_point.x = all_map.features(0,i);
      tmp_point.y = all_map.features(1,i);
      tmp_point.z = 0;
      map_cloud_save->points.push_back(tmp_point);
      tree_map.points.push_back(tmp_point);
    }
    else if(all_map.features(2,i)<-1.021&&all_map.features(2,i)>-1.039)
    {
      pcl::PointXYZI tmp_point;
      tmp_point.x = all_map.features(0,i);
      tmp_point.y = all_map.features(1,i);
      tmp_point.z = 0;
      trajectory2.push_back(tmp_point);
      tmp_point.z = (all_map.features(2,i)+1.03)*1000.;
      mapping_odom_cloud->push_back(tmp_point);
    }
    else if(all_map.features(2,i)<-1.039&&all_map.features(2,i)>-1.045)
    {
      gps_origin[0] = all_map.features(0,i);
      gps_origin[1] = all_map.features(1,i);
      gps_origin[2] = 1;

      sensor_msgs::NavSatFix fix_;
      fix_.header.stamp = ros::Time().now();
      fix_.header.frame_id = "rs_odom";

      fix_.latitude  = gps_origin[0];
      fix_.longitude = gps_origin[1];

      map_origin_pub.publish(fix_);

      if(fabs(gps_origin[0])  < 0.1 )
      {
        gps_msg = false;
      }
      else
      {
        gps_msg = true;
//        std::cout<<"get gps success!"<<std::endl;
      }
    }
    else if(all_map.features(2,i)<-1.045&&all_map.features(2,i)>-1.055)
    {
      cv::Point2f temp_point;
      temp_point.x = all_map.features(0,i);
      temp_point.y = all_map.features(1,i);
      loc_points.push_back(temp_point);
    }
    else
    {
      std::cout<<"map have bad points!!!"<<std::endl;
    }
  }
  background.conservativeResize(new_points);

//  std::cout<<"the loc points size is "<<loc_points.size()<<std::endl;
  loc_maps.clear();
  std::vector<int> new_ptcount;
  new_ptcount.resize(loc_points.size(),0);
  loc_maps.resize(loc_points.size());
  for(int i = 0;i<loc_points.size();++i)
  {
    loc_maps[i] = all_map.createSimilarEmpty();
//    std::cout<<loc_points[i]<<std::endl;
    for(int j=0;j<background.features.cols();++j)
    {
      if (sqrtf(powf(background.features(0, j) - loc_points[i].x, 2) + powf(background.features(1, j) - loc_points[i].y, 2)) < 50)
      {
        loc_maps[i].setColFrom(new_ptcount[i], background, j);
        new_ptcount[i]++;
      }
    }
    loc_maps[i].conservativeResize(new_ptcount[i]);
  }

  if(mapping_odom_cloud->empty())
  {
    odom_map_msg = false;
  }
  else
  {
    odom_map_msg = true;
    /*
    trajectory2.height = 1;
    trajectory2.width = trajectory2.size();
    pcl::io::savePCDFileASCII(fileName2, trajectory2);
    */
  }

  kdtree.setInputCloud(map_cloud_save);

  COUTG(" ~~ The size of the map is: "<<map.size() );

  sensor_msgs::PointCloud2 background_show = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(background, "/rs_odom", ros::Time::now());
  clock_t start = clock();
  while(1)
  {
    displayFeatureMap(tree_map);
    DisplayIDs(tree_map);
    map_pub.publish(background_show);
    publishMap(&mapping_odom_pub, mapping_odom_cloud);
    if((clock()-start)/CLOCKS_PER_SEC > 2)
      break;
  }
  COUTG(" ~~ The map was loaded successfully!");
}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::publishMap(const ros::Publisher* in_publisher, pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud)
{
  sensor_msgs::PointCloud2 output_poles;
  pcl::toROSMsg(*map_cloud,output_poles);
  output_poles.header.frame_id = "rs_odom";
  in_publisher->publish(output_poles);
}


/***********************************************************************************
 *
 * *********************************************************************************/
bool sort_theta(const double_pairs& a,const double_pairs& b)
{
  return a.theta<b.theta;
}

bool sort_distance(const pairs& a,const pairs& b)
{
  return a.distance<b.distance;
}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::measurementUpdate(std::vector<cv::Point2f> map,pose estimate_pose, std::vector<cv::Point2f> poles_points,pose& measure_pose,bool& success)
{
  pairs_poles.clear();

  if (poles_points.size() < 2)
  {
    success = false;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> trans_poles;
  std::vector<cv::Point2f>::iterator it_poles = poles_points.begin();
  for (; it_poles != poles_points.end(); ++it_poles)
  {
    float tran_x = atan2f(sinf(estimate_pose[2]), cosf(estimate_pose[2]));
    float trans_xa = (*it_poles).x * cosf(tran_x) -
        (*it_poles).y * sinf(tran_x);
    float trans_ya = (*it_poles).y * cosf(tran_x) +
        (*it_poles).x * sinf(tran_x);
    float global_xa = trans_xa + estimate_pose[0] ;
    float global_ya = trans_ya + estimate_pose[1] ;

    pcl::PointXYZ tmp_trans_poles;
    tmp_trans_poles.x = global_xa;
    tmp_trans_poles.y = global_ya;
    tmp_trans_poles.z = 0;
    trans_poles.points.push_back(tmp_trans_poles);

    std::vector<cv::Point2f>::iterator it_map = map.begin();
    std::vector<cv::Point2f>::iterator it_map_idx;

    float min_distance = 999;
    for (; it_map != map.end(); ++it_map)
    {
      float distance = sqrtf(powf(global_xa - (*it_map).x, 2) + powf(global_ya - (*it_map).y, 2));
      if (distance < min_distance)
      {
        min_distance = distance;
        it_map_idx = it_map;
      }
    }

    if (min_distance < 3)
    {
      pairs tmp_pair;
      tmp_pair.pole = (*it_poles);
      tmp_pair.map_pole = (*it_map_idx);
      tmp_pair.distance = min_distance;
      pairs_poles.push_back(tmp_pair);
    }
  }

  showCylinder(trans_poles);

  if (pairs_poles.size() > 1)
  {
    //**********删掉地图上重复的元素
    std::stable_sort(pairs_poles.begin(), pairs_poles.end(), sort_distance);
    std::vector<float> map_points_idx;
    std::vector<float> pair_iters;
    std::vector<pairs>::iterator it_pairs_distance = pairs_poles.begin();
    for (; it_pairs_distance != pairs_poles.end(); ++it_pairs_distance)
    {
      if (std::find(map_points_idx.begin(), map_points_idx.end(),
                    (*it_pairs_distance).map_pole.x + (*it_pairs_distance).map_pole.y) != map_points_idx.end())
      {
        pair_iters.push_back((*it_pairs_distance).distance);
      }
      else
      {
        map_points_idx.push_back((*it_pairs_distance).map_pole.x + (*it_pairs_distance).map_pole.y);
      }
    }
    if (!pair_iters.empty())
    {
      std::vector<float>::iterator it_pair_iters = pair_iters.end();
      for (; it_pair_iters != pair_iters.begin(); --it_pair_iters)
      {
        std::vector<pairs>::iterator it_pairs_distance_del = pairs_poles.end();
        for (; it_pairs_distance_del != pairs_poles.begin(); --it_pairs_distance_del)
        {
          if ((*it_pairs_distance_del).distance == (*it_pair_iters))
          {
            pairs_poles.erase(it_pairs_distance_del);
            break;
          }
        }
      }
    }

    std::vector<double_pairs> two_pairs;
    float theta_result = 0, x0_result = 0, y0_result = 0;
    //        std::cout << "\033[1;31m the size is \033[0m" << pairs_poles.size() << std::endl;
    std::vector<pairs>::iterator it_pairs = pairs_poles.begin();
    two_pairs.clear();
    for (; it_pairs != pairs_poles.end() - 1; ++it_pairs)
    {
      std::vector<pairs>::iterator it_pairs_2 = it_pairs + 1;
      for (; it_pairs_2 != pairs_poles.end(); ++it_pairs_2)
      {
        float YA_diff = (*it_pairs).map_pole.y - (*it_pairs_2).map_pole.y;
        float XA_diff = (*it_pairs).map_pole.x - (*it_pairs_2).map_pole.x;
        float xa_diff = (*it_pairs).pole.x - (*it_pairs_2).pole.x;
        float ya_diff = (*it_pairs).pole.y - (*it_pairs_2).pole.y;
        float theta_car = atan2f(YA_diff * xa_diff - XA_diff * ya_diff, XA_diff * xa_diff + YA_diff * ya_diff);
        float diff_theta = theta_car - (estimate_pose[2]);
        diff_theta = fabs(atan2f(sin(diff_theta), cos(diff_theta)));
        //                std::cout << "the compute and ori yaw is" << theta_car << "," << estimate_pose[2] << std::endl;
        if (theta_car != 0 && diff_theta < 0.1)
        {
          double_pairs tmp_two_pairs;
          tmp_two_pairs.XA_1 = (*it_pairs).map_pole;
          tmp_two_pairs.XA_2 = (*it_pairs_2).map_pole;
          tmp_two_pairs.xa_1 = (*it_pairs).pole;
          tmp_two_pairs.xa_2 = (*it_pairs_2).pole;
          tmp_two_pairs.theta = theta_car;
          two_pairs.push_back(tmp_two_pairs);
        }
      }
    }
    float sin_a = 0;
    float cos_a = 0;
    float x_0 = 0;
    float y_0 = 0;

    if (!two_pairs.empty())
    {
      std::vector<double_pairs>::iterator it_thetas = two_pairs.begin();
      for (; it_thetas != two_pairs.end(); ++it_thetas)
      {
        sin_a += sin((*it_thetas).theta);
        cos_a += cos((*it_thetas).theta);
      }
      theta_result = atan2f(sin_a, cos_a);
      std::vector<double_pairs>::iterator it_thetas_2 = two_pairs.begin();
      for (; it_thetas_2 != two_pairs.end(); ++it_thetas_2)
      {
        float x_1 = 0, y_1 = 0;
        x_1 = (*it_thetas_2).XA_1.x - (*it_thetas_2).xa_1.x * cosf(theta_result) +
            (*it_thetas_2).xa_1.y * sinf(theta_result);
        y_1 = (*it_thetas_2).XA_1.y - (*it_thetas_2).xa_1.x * sinf(theta_result) -
            (*it_thetas_2).xa_1.y * cosf(theta_result);
        x_0 += x_1;
        y_0 += y_1;
      }
      x0_result = x_0 / two_pairs.size();
      y0_result = y_0 / two_pairs.size();

      measure_pose << x0_result, y0_result, theta_result;
      //        std::cout<<"\033[1;32m the RESULT is NORMAL \033[0m"<<std::endl;
      success = true;
    }
    else
    {
      success = false;
    }
  }
  else
  {
    success = false;
  }
}


/***********************************************************************************
 *
 * *********************************************************************************/
bool sort_distance_dist(const distance& a,const distance& b)
{
  return  a.dis<b.dis;
}


/***********************************************************************************
 *
 * *********************************************************************************/
bool sort_count(const match& a,const match& b)
{
  if(a.count!=b.count)
    return a.count>b.count;
  else
    return a.dist<b.dist;
}


/***********************************************************************************
 *
 * *********************************************************************************/
float dist_between(cv::Point2f a,cv::Point2f b)
{
  float dist = sqrtf(powf(a.x-b.x,2)+powf(a.y-b.y,2));
  return dist;
}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::kdTreeRadiusSearch(Eigen::Vector3f estimate_pose, float search_radius,  std::vector<cv::Point2f> &resultant_points)
{
  pcl::PointXYZ searchPoint;
  searchPoint.x = estimate_pose[0];
  searchPoint.y = estimate_pose[1];
  searchPoint.z = 0;

  float radius = search_radius;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> a;
  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, a) > 0 )
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      resultant_points.push_back(cv::Point2f( map_cloud_save->points[ pointIdxRadiusSearch[i] ].x,  map_cloud_save->points[ pointIdxRadiusSearch[i] ].y));
  }

}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::transform(std::vector<cv::Point2f> map,std::vector<cv::Point2f> poles_points,
                             Eigen::Vector3f estimate_pose,Eigen::Vector3f& rotated_pose)
{

  std::vector<cv::Point2f> ori_points;
  kdTreeRadiusSearch(estimate_pose, 50, ori_points);

  std::vector<distance> ori_distance;
  std::vector<cv::Point2f>::iterator it_ori = ori_points.begin();
  for (; it_ori != ori_points.end() - 1; ++it_ori)
  {
    std::vector<cv::Point2f>::iterator it_ori2 = it_ori + 1;
    for (; it_ori2 != ori_points.end(); ++it_ori2)
    {
      float dis_ori = sqrt(powf((*it_ori).x - (*it_ori2).x, 2) + powf((*it_ori).y - (*it_ori2).y, 2));
      distance temp_distance;
      temp_distance.a = (*it_ori);
      temp_distance.b = (*it_ori2);
      temp_distance.dis = dis_ori;
      ori_distance.push_back(temp_distance);
    }
  }

  std::vector<cv::Point2f> trans_points;
  std::vector<cv::Point2f>::iterator it_poles = poles_points.begin();
  for (; it_poles != poles_points.end(); ++it_poles)
  {
    if ((*it_poles).x == -20000) continue;
    float tran_x = atan2f(sinf(estimate_pose[2]), cosf(estimate_pose[2]));
    float trans_xa = (*it_poles).x * cosf(tran_x) -
        (*it_poles).y * sinf(tran_x);
    float trans_ya = (*it_poles).y * cosf(tran_x) +
        (*it_poles).x * sinf(tran_x);
    float global_xa = trans_xa + estimate_pose[0];
    float global_ya = trans_ya + estimate_pose[1];
    trans_points.push_back(cv::Point2f(global_xa, global_ya));
  }

  std::vector<distance> trans_distance;
  std::vector<cv::Point2f>::iterator it_trans = trans_points.begin();
  for (; it_trans != trans_points.end() - 1; ++it_trans)
  {
    std::vector<cv::Point2f>::iterator it_trans2 = it_trans + 1;
    for (; it_trans2 != trans_points.end(); ++it_trans2)
    {
      float dis_trans = sqrt(powf((*it_trans).x - (*it_trans2).x, 2) + powf((*it_trans).y - (*it_trans2).y, 2));
      distance temp_distance;
      temp_distance.a = (*it_trans);
      temp_distance.b = (*it_trans2);
      temp_distance.dis = dis_trans;
      trans_distance.push_back(temp_distance);
    }
  }

  std::stable_sort(ori_distance.begin(), ori_distance.end(), sort_distance_dist);
  std::stable_sort(trans_distance.begin(), trans_distance.end(), sort_distance_dist);
  std::vector<cv::Point2f> ori_points_2 = ori_points;
  float car_x = estimate_pose[0];
  float car_y = estimate_pose[1];

  std::vector<match> pairs_sample;

  std::vector<distance>::iterator it_ori_distance = ori_distance.begin();
  for (; it_ori_distance != ori_distance.end(); ++it_ori_distance)
  {
    float temp_ori_distance = (*it_ori_distance).dis;

    float theta_it_ori_distance = atan2f((*it_ori_distance).b.y - (*it_ori_distance).a.y,
                                         (*it_ori_distance).b.x - (*it_ori_distance).a.x);
    std::vector<distance>::iterator it_trans_distance = trans_distance.begin();
    for (; it_trans_distance != trans_distance.end(); ++it_trans_distance)
    {
      float temp_trans_distance = (*it_trans_distance).dis;
      float theta_it_trans_distance = atan2f(
            (*it_trans_distance).b.y - (*it_trans_distance).a.y,
            (*it_trans_distance).b.x - (*it_trans_distance).a.x);
      float theta_it_trans_distance_2 = atan2f(
            (*it_trans_distance).a.y - (*it_trans_distance).b.y,
            (*it_trans_distance).a.x - (*it_trans_distance).b.x);
      if (fabs(temp_ori_distance - temp_trans_distance) < 0.2)
      {
        match pairs_a_a, pairs_a_b;
        //a对a，b对b
        pairs_a_a.A = (*it_ori_distance).a;
        pairs_a_a.B = (*it_ori_distance).b;
        pairs_a_a.a = (*it_trans_distance).a;
        pairs_a_a.b = (*it_trans_distance).b;
        pairs_a_a.dist = fabs(temp_trans_distance - temp_ori_distance);
        pairs_a_a.count = 0;
        float diff_trans_to_ori_theta = atan2f(sinf(theta_it_ori_distance - theta_it_trans_distance),
                                               cosf(theta_it_ori_distance - theta_it_trans_distance));
        float a_rotate_x =
            pairs_a_a.a.x * cosf(diff_trans_to_ori_theta) - pairs_a_a.a.y * sinf(diff_trans_to_ori_theta) +
            car_x + car_y * sinf(diff_trans_to_ori_theta) - car_x * cosf(diff_trans_to_ori_theta);
        float a_rotate_y =
            pairs_a_a.a.y * cosf(diff_trans_to_ori_theta) + pairs_a_a.a.x * sinf(diff_trans_to_ori_theta) +
            car_y - car_y * cosf(diff_trans_to_ori_theta) - car_x * sinf(diff_trans_to_ori_theta);

        float x_offset = pairs_a_a.A.x - a_rotate_x;
        float y_offset = pairs_a_a.A.y - a_rotate_y;

        std::vector<cv::Point2f>::iterator it_trans_to = trans_points.begin();
        for (; it_trans_to != trans_points.end(); ++it_trans_to)
        {

          float tmp_trans_to_ori_x = (*it_trans_to).x * cosf(diff_trans_to_ori_theta) -
              (*it_trans_to).y * sinf(diff_trans_to_ori_theta) +
              car_x + car_y * sinf(diff_trans_to_ori_theta) -
              car_x * cosf(diff_trans_to_ori_theta);
          float tmp_trans_to_ori_y = (*it_trans_to).y * cosf(diff_trans_to_ori_theta) +
              (*it_trans_to).x * sinf(diff_trans_to_ori_theta) +
              car_y - car_y * cosf(diff_trans_to_ori_theta) -
              car_x * sinf(diff_trans_to_ori_theta);

          cv::Point2f it_trans_to_global;
          it_trans_to_global.x = tmp_trans_to_ori_x + x_offset;
          it_trans_to_global.y = tmp_trans_to_ori_y + y_offset;
          std::vector<cv::Point2f>::iterator it_ori_1 = ori_points_2.begin();
          for (; it_ori_1 != ori_points_2.end(); ++it_ori_1)
          {
            float dist = dist_between(it_trans_to_global, (*it_ori_1));
            if (dist < 0.3)
            {
              pairs_a_a.count++;
              break;
            }
          }
        }
        pairs_a_b.A = (*it_ori_distance).a;
        pairs_a_b.B = (*it_ori_distance).b;
        pairs_a_b.a = (*it_trans_distance).b;
        pairs_a_b.b = (*it_trans_distance).a;
        pairs_a_b.dist = fabs(temp_trans_distance - temp_ori_distance);
        pairs_a_b.count = 0;
        float diff_trans_to_ori_theta_2 = atan2f(sinf(theta_it_ori_distance - theta_it_trans_distance_2),
                                                 cosf(theta_it_ori_distance - theta_it_trans_distance_2));
        float a_rotate_x_2 =
            pairs_a_b.a.x * cosf(diff_trans_to_ori_theta_2) -
            pairs_a_b.a.y * sinf(diff_trans_to_ori_theta) +
            car_x + car_y * sinf(diff_trans_to_ori_theta_2) - car_x * cosf(diff_trans_to_ori_theta);
        float a_rotate_y_2 =
            pairs_a_b.a.y * cosf(diff_trans_to_ori_theta_2) +
            pairs_a_b.a.x * sinf(diff_trans_to_ori_theta_2) +
            car_y - car_y * cosf(diff_trans_to_ori_theta_2) - car_x * sinf(diff_trans_to_ori_theta_2);
        float x_offset_2 = pairs_a_b.A.x - a_rotate_x_2;
        float y_offset_2 = pairs_a_b.A.y - a_rotate_y_2;
        std::vector<cv::Point2f>::iterator it_trans_to_2 = trans_points.begin();
        for (; it_trans_to_2 != trans_points.end(); ++it_trans_to_2)
        {

          float tmp_trans_to_ori_x_2 = (*it_trans_to_2).x * cosf(diff_trans_to_ori_theta_2) -
              (*it_trans_to_2).y * sinf(diff_trans_to_ori_theta_2) +
              car_x + car_y * sinf(diff_trans_to_ori_theta_2) -
              car_x * cosf(diff_trans_to_ori_theta_2);
          float tmp_trans_to_ori_y_2 = (*it_trans_to_2).y * cosf(diff_trans_to_ori_theta_2) +
              (*it_trans_to_2).x * sinf(diff_trans_to_ori_theta_2) +
              car_y - car_y * cosf(diff_trans_to_ori_theta_2) -
              car_x * sinf(diff_trans_to_ori_theta_2);

          cv::Point2f it_trans_to_global_2;
          it_trans_to_global_2.x = tmp_trans_to_ori_x_2 + x_offset_2;
          it_trans_to_global_2.y = tmp_trans_to_ori_y_2 + y_offset_2;
          std::vector<cv::Point2f>::iterator it_ori_2 = ori_points_2.begin();
          for (; it_ori_2 != ori_points_2.end(); ++it_ori_2)
          {
            float dist = dist_between(it_trans_to_global_2, (*it_ori_2));
            if (dist < 0.3)
            {
              pairs_a_b.count++;
              break;
            }
          }
        }
        pairs_sample.push_back(pairs_a_a);
        pairs_sample.push_back(pairs_a_b);
      }

    }

  }

  std::stable_sort(pairs_sample.begin(), pairs_sample.end(), sort_count);
  //    std::cout << (*pairs_sample.begin()).count << std::endl;
  match the_one = (*pairs_sample.begin());
  float pairs_trans_theta = atan2f(the_one.b.y - the_one.a.y, the_one.b.x - the_one.a.x);
  float pairs_oir_theta = atan2f(the_one.B.y - the_one.A.y, the_one.B.x - the_one.A.x);
  //a-b逆时钟旋转角度到地图A-B,车的姿态也要加上这个旋转角度（逆时针）
  float pairs_diffs = atan2f(sinf(pairs_oir_theta - pairs_trans_theta), cosf(pairs_oir_theta - pairs_trans_theta));
  //    std::cout << std::endl << "the trans theta is " << pairs_diffs / 3.1415 * 180 << std::endl;
  float x0 = the_one.a.x * cosf(pairs_diffs) - the_one.a.y * sinf(pairs_diffs) +
      car_x + car_y * sinf(pairs_diffs) - car_x * cosf(pairs_diffs);
  float y0 = the_one.a.y * cosf(pairs_diffs) + the_one.a.x * sinf(pairs_diffs) +
      car_y - car_y * cosf(pairs_diffs) - car_x * sinf(pairs_diffs);

  float car_offset_x = the_one.A.x - x0;
  float car_offset_y = the_one.A.y - y0;
  //    std::cout << "the car offset is " << car_offset_x << "," << car_offset_y << std::endl;

  rotated_pose[0] = estimate_pose[0] + car_offset_x;
  rotated_pose[1] = estimate_pose[1] + car_offset_y;
  rotated_pose[2] = atan2f(sinf(estimate_pose[2] + pairs_diffs), cosf(estimate_pose[2] + pairs_diffs));
}


/***********************************************************************************
 *
 * *********************************************************************************/
void localization::gps_callback(const sensor_msgs::NavSatFix &gps_msg)
{
  //××××××××处理GPS数据
  double ori_lat = gps_origin[0]; //39.9680502;
  double ori_lon = gps_origin[1]; //116.3045885;
  double ori_ati = gps_origin[2];

  Eigen::Vector3d gps;
  gps<<gps_msg.longitude,gps_msg.latitude,10;
  Eigen::Vector3d ret;
  ret = WGS84toECEF(gps);


  double rad_lon = ori_lon/180*PI;
  double rad_lat = ori_lat/180*PI;
  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  Eigen::Matrix3d rot;

  rot<<-sin_lon, cos_lon, 0,
      -sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat,
      cos_lat*cos_lon,  cos_lat*sin_lon,  0;

  Eigen::Vector3d z0;
  Eigen::Vector3d wgs0;
  wgs0<< ori_lon, ori_lat, ori_ati;
  z0 = WGS84toECEF(wgs0);
  Eigen::Vector3d l;
  l = ret-z0;
  XYZ = rot*l;

  XYZ[2] = gps_msg.altitude;

  //  COUTR("GPS value is: "<< XYZ.transpose());

  get_gps = true;
}


/***********************************************************************************
 *
 * *********************************************************************************/
Eigen::Vector3d localization::WGS84toECEF(Eigen::Vector3d gps)
{
  double SEMI_MAJOR_AXIS = 6378137.0;
  double RECIPROCAL_OF_FLATTENING = 298.257223563;
  double SEMI_MINOR_AXIS = 6356752.3142;
  double FIRST_ECCENTRICITY_SQUARED = 6.69437999014e-3;
  double SECOND_ECCENTRICITY_SQUARED = 6.73949674228e-3;

  double lon = gps[0];
  double lat = gps[1];
  double ati = gps[2];

  double rad_lon = lon/180*PI;
  double rad_lat = lat/180*PI;

  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  double chi = sqrt(1.0-FIRST_ECCENTRICITY_SQUARED*sin_lat*sin_lat);
  double N = SEMI_MAJOR_AXIS/chi+ati;

  Eigen::Vector3d ret;
  ret<<N*cos_lat*cos_lon,N*cos_lat*sin_lon,(SEMI_MAJOR_AXIS*(1.0-FIRST_ECCENTRICITY_SQUARED)/chi+ati)*sin_lat;

  return ret;
}





} //robosense
