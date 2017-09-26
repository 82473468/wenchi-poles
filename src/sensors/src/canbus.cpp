
#include<sensor.h>
#include <iostream>
#include <sstream>
#include <sensors/canbus.h>
#include <nav_msgs/Odometry.h>
using std::cout;
using std::endl;

namespace robosense {
  namespace sensor {


    class Canbus : public Sensor {
    public:
      Canbus();
      void analysis();
      bool checkhead();

    private:
      sensors::canbus msg_;
      nav_msgs::Odometry msg2_;
      unsigned char read_buf_[100];
      std::vector<std::string>s_vec_;
      std:: stringstream stream_;
      unsigned char command_[6];


    };
    Canbus::Canbus()
    {
      head_ = ""; //交互式读数据，需要初始化
      command_[0] = '0';
      command_[1] = '1';
      command_[2] = '0';
      command_[3] = 'D';
      command_[4] = '\r';
      command_[5] = '\n';

    }

    bool Canbus::checkhead()
    {
      int wr;

      unsigned char ws_buf[10] = "ATWS\r\n";
      unsigned char tz_buf[10] = "ATZ\r\n";
      unsigned char e0_buf[10] = "ATE0\r\n";
      unsigned char l0_buf[10] = "ATL0\r\n";
      unsigned char h0_buf[10] = "ATH0\r\n";

      wr = serial_.write(ws_buf,6);    //
      if(wr < 0)
      {
        ROS_INFO("sensors/canbus/init   ATWS fail!");
        return -1;
      }
      for(int i = 0 ; i < 10; i++)
      {
        serial_.read(read_buf_,1);

        if ('E' == read_buf_[0])
        {
          serial_.read(read_buf_+1,10);
          ROS_INFO("read_buf=%s\n",read_buf_);
          break;
        }else if(i == 10)
        {
          return false;
        }
      }

      usleep(500000);
      wr = serial_.write(tz_buf,5);
      if(wr < 0)
      {
        ROS_INFO("sensors/canbus/init   ATZ fail!");
        return -1;
      }
      read_buf_[0] = 0;
      for(int i = 0 ; i < 10; i++)
      {
        serial_.read(read_buf_,1);

        if ('E' == read_buf_[0])
        {
          serial_.read(read_buf_+1,10);
          ROS_INFO("read_buf=%s\n",read_buf_);
          break;
        }else if(i == 10)
        {
          return false;
        }
      }

      usleep(500000);
      wr = serial_.write(e0_buf,6);                 //e0_buf
      if(wr < 0)
      {
        ROS_INFO("sensors/canbus/init   ATE0 fail!");
        return -1;
      }
      read_buf_[0] = 0;
      for(int i = 0 ; i < 10; i++)
      {
        serial_.read(read_buf_,1);

        if ('E' == read_buf_[0])
        {
          serial_.read(read_buf_+1,10);
          ROS_INFO("read_buf=%s\n",read_buf_);
        }else if(i == 10)
        {
          return false;
        }
      }

      usleep(500000);
      wr = serial_.write(l0_buf,6);
      if(wr < 0)
      {
        ROS_INFO("sensors/canbus/init   ATL0 fail!");
        return -1;
      }
      read_buf_[0] = 0;
      for(int i = 0 ; i < 10; i++)
      {
        serial_.read(read_buf_,1);

        if ('E' == read_buf_[0])
        {
          serial_.read(read_buf_+1,10);
          ROS_INFO("read_buf=%s\n",read_buf_);
          break;
        }else if(i == 10)
        {
          return false;
        }
      }


      usleep(500000);
      wr = serial_.write(h0_buf,6);
      if(wr < 0)
      {
        ROS_INFO("sensors/canbus/init   ATH0 fail!");
        return -1;
      }
      read_buf_[0] = 0;
      for(int i = 0 ; i < 10; i++) {
        serial_.read(read_buf_, 1);

        if ('E' == read_buf_[0]) {
          serial_.read(read_buf_ + 1, 10);
          ROS_INFO("read_buf=%s\n", read_buf_);
          break;
        } else if (i == 10) {
          return false;
        }
      }
      return true;
    }


    void Canbus::analysis()
    {
      ///ROS_INFO("====analysis=====");
      usleep(500000);
      int wr = 0;

      // wr = serial_.write(command_,6);
      std::string recv,cmd ="010C\r";
      wr= serial_.write(cmd);
      //ROS_INFO("%d",wr);
      //serial_.flushOutput();
      if(wr < 0)
      {
        ROS_INFO("write fail!\n");
        usleep(500000);
        return ;
      }


      read_buf_[0] = 0;
      read_buf_[1] = 0;
      while (1)
      {
        int s = serial_.read(read_buf_,1);
        if('4' == read_buf_[0])
        {
          s = serial_.read(read_buf_+ 1,1);
          if('1' == read_buf_[1])
          {
            s = serial_.read(read_buf_+ 2,10);
            int car_speed,car_angle;
            ROS_INFO("rev %s",read_buf_);

            sscanf((const char *)(read_buf_+ 6),"%x",&car_speed);
           // msg_.speed = (float)car_speed;
            //msg_.steering = 0;
           // msg_.head.stamp = ros::Time::now();
           // pub_.publish(msg_);
            msg2_.header.stamp =ros::Time::now();
            msg2_.twist.twist.linear.x = car_speed;
            pub_.publish(msg2_);

          }
        }
      }

    }
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "canbus");

  ros::NodeHandle private_nh("~");

  std::string port;
  int baud;
  //==================================================================
  if (private_nh.getParam(std::string("port"), port ))
  {
  }
  else
  {
    port = "/dev/ttyUSB4";
    ROS_INFO(" port is not setup");

  }
  cout << "port = " << port << endl;
  if( private_nh.getParam(std::string("baud"), baud ))
  {

  }else
  {
    ROS_INFO("cannot find baud ");
    baud = 38400;
    ROS_INFO("baud is not setup " );
  }
  robosense::sensor::Canbus sensor;
 // ros::Publisher pub = private_nh.advertise<sensors::canbus>("canbus",100);
  ros::Publisher pub = private_nh.advertise<nav_msgs::Odometry>("canbus",100);

  sensor.set(port,baud);
  sensor.setHead("");
  sensor.setPublisher(pub);
  sensor.open();
  if (sensor.isOpen())
  {
    ROS_INFO("port is open " );
    if(sensor.checkhead())
    {
      ROS_INFO("port:%s is a canbus port  ",port.c_str() );
    } else
    {
      ROS_INFO("port:%s is not a canbus port  ",port.c_str() );
    }

  }else
  {
    ROS_INFO("port is not open " );
  }
//=====================================================



  // loop until shut  down or end of file
  while(private_nh.ok())
  {

    if (sensor.isOpen())
    {
      sensor.analysis();

    } else{
      private_nh.shutdown();
    }
    ros::spinOnce();
  }

  return 0;

}