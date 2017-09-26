//
// Created by guoleiming on 17-7-7.
// 千寻跬步FindM RTD服务
//

#include<iostream>
#include<sstream>
#include "std_msgs/String.h"
#include <sensors/gpgga.h>
#include <string>
#include<sensor.h>
#include <ros/ros.h>
#include <sensors/qxwz_rtcm.h>
#include <qxwz_rtcm.h>


#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

//
using std::cout;
using std::endl;
using std::string;


serial::Serial mycom_;
qxwz_account_info *p_account_info_ = NULL;
bool g_ntrip_ok_ = false;
int  counter = 0;

void  get_qxwz_sdk_account_info(void)
{
  p_account_info_ = getqxwzAccount();
  if(p_account_info_->appkey != NULL)
  {
    ROS_INFO("appkey=%s\n",p_account_info_->appkey);
  }

  if(p_account_info_->deviceID != NULL)
  {
    ROS_INFO("deviceID=%s\n",p_account_info_->deviceID);
  }

  if(p_account_info_->deviceType != NULL)
  {
    ROS_INFO("deviceType=%s\n",p_account_info_->deviceType);
  }

  if(p_account_info_->NtripUserName != NULL)
  {
    ROS_INFO("NtripUserName=%s\n",p_account_info_->NtripUserName);
  }

  if(p_account_info_->NtripPassword != NULL)
  {
    ROS_INFO("NtripPassword=%s\n",p_account_info_->NtripPassword);
  }

  ROS_INFO("expire_time=%ld\n",p_account_info_->expire_time);
}

void qxwz_rtcm_response_callback(qxwz_rtcm data)
{
  if(mycom_.isOpen())
  {
//    int s =mycom_.write((const uint8_t  *)data.buffer,data.length);
    size_t s =mycom_.write(data.buffer);
    //cout <<"write " << s <<endl;
  }
}
void qxwz_status_response_callback(qxwz_rtcm_status code)
{
  //printf("QXWZ_RTCM_STATUS:%d\n",code);
  ROS_INFO("QXWZ_RTCM_STATUS:%d\n",code);
  struct tm *ptr = NULL;
  g_ntrip_ok_ = false;
  //test account expire
  if(code == QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE)
  {
    cout << "账号即将过期" <<endl;
    //get_qxwz_sdk_account_info();
  }
  else if(QXWZ_STATUS_OPENAPI_ACCOUNT_EXPIRED == code)
  {
    cout << "账号已过期" <<endl;
  }
  else if(QXWZ_STATUS_APPKEY_IDENTIFY_SUCCESS == code)
  {
    cout << "验证成功" <<endl;
    g_ntrip_ok_ = true;
  }
  else if (QXWZ_STATUS_NTRIP_RECEIVING_DATA == code)
  {
    g_ntrip_ok_ = true;
  }
}

ros::Publisher navsat_fix_pub;

namespace robosense
{
namespace sensor
{
class FindM : public Sensor
{
public:
  FindM();
  void analysis();
  void setConfig(string appKey,string appSecret,string deviceId,string deviceType );
  bool checkhead();
  qxwz_config config_;

private:
  sensors::gpgga msg_;
  //std_msgs::String msg1_;
  std::vector<std::string>s_vec_;
  std::stringstream stream_;

  sensor_msgs::NavSatFix fix_;

  string gga_;
  friend void qxwz_rtcm_response_callback(qxwz_rtcm data);

};


FindM::FindM()
{
  head_ = "$GNGGA";
  ros::Time().now();
}


void FindM::setConfig(string appKey,string appSecret,string deviceId,string deviceType )
{
  config_.appkey = (char*)appKey.c_str();
  config_.appSecret = (char*)appSecret.c_str();
  config_.deviceId = (char*)deviceId.c_str();
  config_.deviceType = (char*)deviceType.c_str();
}


bool FindM::checkhead()
{
  std::string line;
  serial_.flushInput();
  serial_.close();
  qxwz_setting(&config_);
  mycom_.setPort(port_);
  qxwz_rtcm_start(qxwz_rtcm_response_callback,qxwz_status_response_callback);
  mycom_.setBaudrate(baud_);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(300);
  mycom_.setTimeout(timeout);
  mycom_.open();
  int error = 0;
  for (int i = 0; i < 100; ++i)  //有的gps 发送多种格式的数据
  {
    mycom_.readline(line);

    if (line.find(" ")!= line.npos)
    {
      line.replace(line.find(" "),1,"");
    }

    std::cout << line ;//<<std::endl;

    if( line.npos!= line.find(head_))
    {
      return true;
    }
    line.clear();
  }
  return false;

}


void FindM::analysis()
{
  while (mycom_.isOpen())
  {
    string gps= mycom_.readline();
    if (gps.find(" ")!= gps.npos)
    {
      gps.replace(gps.find(" "),1,"");
    }
    if (gps.npos!= gps.find(head_))
    {
      s_vec_ = split(gps,",");
//      ROS_INFO("%s",gps.c_str());

      if("$GPGGA"== s_vec_[0] && "" != s_vec_[3]) //$GNGGA
      {

        gga_ = gps;

        if (g_ntrip_ok_ && counter%5 == 0)
        {
          qxwz_rtcm_sendGGAWithGGAString((char*)gga_.c_str());
          counter = 1;
          ROS_INFO("Send GGA done");
        }
        else if (g_ntrip_ok_ && counter%5 != 0)
        {
          counter++;
        }
        else
        {
          counter = 0;
          ROS_INFO("No ntrip service");
        }

//------------------------------------------------------------
        msg_.gga_1 = s_vec_[0];

        msg_.utc_time_2 = s_vec_[1];

        float s_vec_3 = atof(s_vec_[2].c_str());
        s_vec_3 = int(s_vec_3)/100 + (s_vec_3 - 100*(int(s_vec_3)/100))/60.0;
        if(s_vec_[3] == "S")
          s_vec_3 = - 1.0*s_vec_3;
        msg_.lat_3 = s_vec_3;
        stream_.clear();

        msg_.latdir_4  = s_vec_[3];

        float s_vec_5 = atof(s_vec_[4].c_str());
        s_vec_5 = int(s_vec_5)/100 + (s_vec_5 - 100*(int(s_vec_5)/100))/60.0;
        if(s_vec_[5] == "W")
          s_vec_5 = -1.0*s_vec_5;
        msg_.lon_5 = s_vec_5;
        stream_.clear();

        msg_.londir_6  = s_vec_[5];

        msg_.QF_7  = s_vec_[6];

        stream_.str(s_vec_[7]);
        stream_ >> msg_.satNo_8;
        stream_.clear();

        stream_.str(s_vec_[8]);
        stream_ >> msg_.hdop_9;
        stream_.clear();

        stream_.str(s_vec_[9]);
        stream_ >> msg_.alt_10;
        stream_.clear();

        msg_.units_11  = s_vec_[10];
        msg_.age_14 = s_vec_[11];
        msg_.mode_15 = s_vec_[12];

        msg_.header.stamp = ros::Time().now();
        msg_.header.frame_id = "/FindM";
        pub_.publish(msg_);
//------------------------------------------------------------

        if(s_vec_[6] =="0")
        {
          fix_.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }
        else if(s_vec_[6] =="1")
        {
          fix_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        }
        else if(s_vec_[6] =="2")
        {
          fix_.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        }
        else
        {
          fix_.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }

        fix_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        fix_.latitude = s_vec_3;
        fix_.longitude = s_vec_5;
        float s_vec_10 = atof(s_vec_[9].c_str());
        fix_.altitude =  s_vec_10;

        float hdop = atof(s_vec_[8].c_str());
        fix_.position_covariance[0] = hdop * hdop;
        fix_.position_covariance[4] = hdop * hdop;
        fix_.position_covariance[8] = 4.0 * hdop * hdop;
        fix_.position_covariance_type =  sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

        fix_.header.stamp = ros::Time().now();
        fix_.header.frame_id = "/fix";
        navsat_fix_pub.publish(fix_);
//------------------------------------------------------------
      }
    }
    usleep(10000);
  }
  usleep(80000);
}


}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "FindM");

  ros::NodeHandle private_nh("~");

  std::string port;
  int baud;

  //=========================GPS=========================================
  if (private_nh.getParam(std::string("port"), port))
  {
  }
  else
  {
    port = "/dev/ttyUSB0";
    ROS_INFO("The port Was not setup");
  }
  cout << "port = " << port << endl;


  if( private_nh.getParam("baud", baud ))
  {
  }
  else
  {
    baud = 115200;
    ROS_INFO("The baud rate was set to 115200" );
  }

  robosense::sensor::FindM sensor;
  ros::Publisher pub = private_nh.advertise<sensors::gpgga>("FindM",100);
  navsat_fix_pub = private_nh.advertise<sensor_msgs::NavSatFix>("fix", 1);

  sensor.set(port,baud);
  sensor.setHead("GGA");
  sensor.setPublisher(pub);
  sensor.open();

  std::string appKey;
  if( private_nh.getParam("appKey", appKey ))
  {
    sensor.config_.appkey = (char*)appKey.c_str();
  }
  else
  {
    sensor.config_.appkey= (char *)"510328" ;
    ROS_INFO("The appkey was set to default: 510328 " );
  }

  std::string appSecret;
  if( private_nh.getParam("appSecret",appSecret))
  {
    sensor.config_.appSecret = (char*)appSecret.c_str();
  }
  else
  {
    sensor.config_.appSecret= (char *)"a9a55c74fa8797e5ba9ca482d83311b9a6341ab99996050345ad43a480bfded9";  //"请输入AppSecret";
    ROS_INFO("The appSecret was set to default: a9a55c74fa8797e5ba9ca482d83311b9a6341ab99996050345ad43a480bfded9");
  }

  std::string deviceId;
  if( private_nh.getParam("deviceId", deviceId ))
  {
    sensor.config_.deviceId = (char*)deviceId.c_str();
  }
  else
  {
    sensor.config_.deviceId=  (char *)"123343455443";  //"请输入唯一的设备ID";
    ROS_INFO("The deviceId was set to default: 123343455443 " );
  }

  std::string deviceType;
  if( private_nh.getParam("deviceType", deviceType ))
  {
    sensor.config_.deviceType = (char*)deviceType.c_str();
  }
  else
  {
    sensor.config_.deviceType= (char *)"gps"; //"请输入设备的类型";
    ROS_INFO("The deviceType was set to default: GPS " );
  }


  if (sensor.isOpen())
  {
    ROS_INFO("port is open " );
    if(sensor.checkhead())
    {
      ROS_INFO("The port:%s is a FindM port",port.c_str() );
    }
    else
    {
      ROS_INFO("The port:%s is not a FindM port",port.c_str() );
      private_nh.shutdown();
    }

  }
  else
  {
    ROS_INFO("The port is not open " );
    private_nh.shutdown();
  }
  //=====================================================



  // loop until shut  down or end of file
  while(private_nh.ok())
  {
    if (mycom_.isOpen())
    {
      sensor.analysis();
    }
    ros::spinOnce();
    //================================================
  }

  return 0;

}
