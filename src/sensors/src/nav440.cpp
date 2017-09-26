//
// Created by guoleiming on 17-7-1.
//
#include <sensor.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <memsic_type.h>
#include <memsic_type.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#define BUFFSIZE 50
using std::cout;
using std::endl;
namespace robosense
{
//用于大端数据向小端数据转换
namespace sensor
{
union I2NAV
{
    unsigned char data[2];
    short nub_s;
};
union I4NAV
{
    unsigned char data[4];
    int nub_i;
};


class nav440 : public Sensor
{
public:
    nav440();

    void analysis();

    virtual bool checkhead();

private:
    sensor_msgs::Imu imu_;
private:
    QUEUE_TYPE circ_buf_;
    XBOW_PACKET packet_;
    unsigned char buff_[BUFFSIZE];
};

nav440::nav440()
{
    baud_ = 38400;
    head_ = "UU";// 0x55 0x55

}

bool nav440::checkhead()
{
    serial_.flush();
    Initialize(&circ_buf_);
    size_t s;
    for (int i = 0; i < 20; ++i)
    {
        s = serial_.read(buff_, BUFFSIZE);
        for (int j = 0; j < s; j++)
        {
            AddQueue(buff_[j], &circ_buf_);
            if (process_xbow_packet(&circ_buf_, &packet_))   //查找协议头
            {
                return true;
            }
        }
    }
    return false;

}


void nav440::analysis()
{
    size_t s;
    I2NAV i2nav;
    I4NAV i4nav;
    short a;
    double b;
    tf::Quaternion qua;
    s = serial_.read(buff_, BUFFSIZE);
    for (int j = 0; j < s; j++)
    {
        AddQueue(buff_[j], &circ_buf_);
    }
    if (process_xbow_packet(&circ_buf_, &packet_))   //查找协议头
    {
        i2nav.data[0] = packet_.data[1];
        i2nav.data[1] = packet_.data[0];
        a = i2nav.nub_s;
        double rool = (a * M_PI * 2) / pow(2, 16);   //roll
        // cout << b<< ",  ";


        i2nav.data[0] = packet_.data[3];
        i2nav.data[1] = packet_.data[2];
        a = i2nav.nub_s;
        double pitch = (a * M_PI * 2) / pow(2, 16);   //pitch
        //  cout <<b<< ",  ";


        i2nav.data[0] = packet_.data[5];
        i2nav.data[1] = packet_.data[4];
        a = i2nav.nub_s;
        double yaw = (a * M_PI * 2) / pow(2, 16);    //yaw
        //   cout << b<< ",  ";
        qua.setRPY(rool, pitch, yaw);
        imu_.orientation.w = qua.getW();
        imu_.orientation.x = qua.getX();
        imu_.orientation.y = qua.getY();
        imu_.orientation.z = qua.getZ();

        i2nav.data[0] = packet_.data[7];
        i2nav.data[1] = packet_.data[6];   //xRateCorrected
        a = i2nav.nub_s;
        b = (a * 1260) / pow(2, 16);
        //  cout << b<< ",  ";

        i2nav.data[0] = packet_.data[9];
        i2nav.data[1] = packet_.data[8];
        a = i2nav.nub_s;
        b = (a * 1260) / pow(2, 16);   //yRateCorrected
        // cout << b<< ",  ";

        i2nav.data[0] = packet_.data[11];
        i2nav.data[1] = packet_.data[10];
        a = i2nav.nub_s;
        b = (a * 1260) / pow(2, 16);    // zRateCorrected
        //  cout << b<< ",  ";

        i2nav.data[0] = packet_.data[13];
        i2nav.data[1] = packet_.data[12];
        a = i2nav.nub_s;
        b = (a * 20) / pow(2, 16);
        cout << b << ",  ";    //xAccel
        imu_.linear_acceleration.x = b;

        i2nav.data[0] = packet_.data[15];
        i2nav.data[1] = packet_.data[14];
        a = i2nav.nub_s;
        b = (a * 20) / pow(2, 16);
        cout << b << ",  ";;   //yAccel
        imu_.linear_acceleration.y = b;

        i2nav.data[0] = packet_.data[17];
        i2nav.data[1] = packet_.data[16];
        a = i2nav.nub_s;
        b = (a * 20) / pow(2, 16);
        cout << b << ",  ";    //zAccel
        imu_.linear_acceleration.z = b;

        i2nav.data[0] = packet_.data[19];
        i2nav.data[1] = packet_.data[18];
        a = i2nav.nub_s;
        b = (a * 512) / pow(2, 16);
        cout << b << ",  ";

        i2nav.data[0] = packet_.data[21];
        i2nav.data[1] = packet_.data[20];
        a = i2nav.nub_s;
        b = (a * 512) / pow(2, 16);
        cout << b << ",  ";;

        i2nav.data[0] = packet_.data[23];
        i2nav.data[1] = packet_.data[22];
        a = i2nav.nub_s;
        b = (a * 512) / pow(2, 16);
        cout << b << ",  ";


        int temp_i;
        double temp_d;
        i4nav.data[0] = packet_.data[27];
        i4nav.data[1] = packet_.data[26];
        i4nav.data[2] = packet_.data[25];
        i4nav.data[3] = packet_.data[24];
        temp_i = i4nav.nub_i;
        temp_d = (temp_i * 360) / pow(2, 32);
        cout << "lo  " << temp_d << ",  ";

        i4nav.data[0] = packet_.data[31];
        i4nav.data[1] = packet_.data[30];
        i4nav.data[2] = packet_.data[29];
        i4nav.data[3] = packet_.data[28];
        temp_i = i4nav.nub_i;
        temp_d = (temp_i * 360) / pow(2, 32);
        cout << "la " << temp_d << ",  ";


        i2nav.data[0] = packet_.data[33];
        i2nav.data[1] = packet_.data[32];
        a = i2nav.nub_s;
        b = (a * 0.01);
        cout << b << endl;

        i2nav.data[0] = packet_.data[35];
        i2nav.data[1] = packet_.data[34];
        a = i2nav.nub_s;
        b = (a * 200) / pow(2, 16);
        cout << b << endl;
        imu_.header.frame_id = "car";
        imu_.header.stamp = ros::Time::now();
        pub_.publish(imu_);

    }
}
}
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav440");

  ros::NodeHandle private_nh("~");


  // private_nh.param(std::string("port"), port , std::string("/dev/pts/33"));
  //private_nh.param(std::string("baud"), baud,57600);
  std::string port;
  int baud;

  //=========================nav440 惯性导航模块=========================================
  if (private_nh.getParam(std::string("port"), port))
  {
  }
  else
  {
    port = "/dev/ttyUSB0";
    ROS_INFO(" port is not setup");
  }
  cout << "port = " << port << endl;
  if( private_nh.getParam(std::string("baud"), baud ))
  {
  }
  else
  {
    baud = 38400;
    ROS_INFO("baud is not setup " );
  }
  robosense::sensor::nav440 nav;
  //ros::Publisher gps_pub = private_nh.advertise<sensors::gprmc>("gps",100);


  ros::Publisher pub = private_nh.advertise<sensor_msgs::Imu>("nav440",100);
  nav.set(port,baud);
  nav.setHead("UU");//0x5555
  nav.setPublisher(pub);
  nav.open();

  if (nav.isOpen())
  {
    ROS_INFO("port is open " );
    if(nav.checkhead())
    {
      ROS_INFO("port:%s is a nav440 port  ",port.c_str() );
    }
    else
    {
      ROS_INFO("port:%s is not a nav440 port ",port.c_str() );
      private_nh.shutdown();
    }

  }
  else
  {
    ROS_INFO("port is not open " );
    private_nh.shutdown();
  }
  //=====================================================

  // loop until shut  down or end of file
  while(private_nh.ok())
  {
    if (nav.isOpen())
    {
      nav.analysis();
    }
    ros::spinOnce();
    //================================================
  }

  return 0;

}
