#include <iostream>
#include <sstream>
#include "std_msgs/String.h"
#include <sensors/gprmc.h>
#include <string>
#include <sensor.h>
#include <ros/ros.h>


#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

//
using std::cout;
using std::endl;
using namespace std;


ros::Publisher navsat_fix_pub;

namespace robosense
{
namespace sensor
{
class GPS : public Sensor
{
public:
    GPS();
    void analysis();

private:
    sensors::gprmc msg_;
    //std_msgs::String msg1_;
    std::vector<std::string>s_vec_;
    std:: stringstream stream_;
    sensor_msgs::NavSatFix fix_;

};


GPS::GPS()
{
    head_ = "$GPRMC";
    ros::Time().now();
}
void GPS::analysis()
{
    std::string  line;
    serial_.readline(line);

    if (line.find(" ")!= line.npos)
    {
        line.replace(line.find(" "),1,"");
    }


    if (0 == line.find(head_))
    {
        //msg1_.data= line;
        ROS_INFO("%s",line.c_str());
        //pub_.publish(msg1_);

        stream_.str(line);
        stream_ >> msg_.initdata_0;
        stream_.clear();
        s_vec_ = split(line,",");
        if("$GPRMC"== s_vec_[0] || "$GNRMC"== s_vec_[0])
        {
            msg_.rmc_1 = s_vec_[0];

            msg_.utc_time_2 = s_vec_[1];
            msg_.status_3  = s_vec_[2];

            float s_vec_3 = atof(s_vec_[3].c_str());
            s_vec_3 = int(s_vec_3)/100 + (s_vec_3 - 100*(int(s_vec_3)/100))/60.0;
            if(s_vec_[4] == "S")
                 s_vec_3 = - 1.0*s_vec_3;
            msg_.latitude_4=s_vec_3;
            stream_.clear();

            msg_.N_5  = s_vec_[4];

            float s_vec_5 = atof(s_vec_[5].c_str());
            s_vec_5 = int(s_vec_5)/100 + (s_vec_5 - 100*(int(s_vec_5)/100))/60.0;
            if(s_vec_[6] == "W")
                 s_vec_5 = -1.0*s_vec_5;
            msg_.longitude_6=s_vec_5;
            stream_.clear();

            msg_.E_7  = s_vec_[6];

            stream_.str(s_vec_[7]);
            stream_ >> msg_.spd_8;
            stream_.clear();


            stream_.str(s_vec_[8]);
            stream_ >> msg_.cog_9;
            stream_.clear();

            msg_.utc_day_10  = s_vec_[9];

            stream_.str(s_vec_[10]);
            stream_ >> msg_.mv_11;
            stream_.clear();

            msg_.mvE_12  = s_vec_[11];
            msg_.mode_13 = s_vec_[12];

            msg_.header.stamp = ros::Time().now();
            msg_.header.frame_id = "/gps";
            pub_.publish(msg_);
            //ROS_INFO("msg %s,%s ",msg_.rmc_1.c_str(), msg_.utc_time_2.c_str());
            // ROS_INFO("msg %s, %s, %s, %f, %s, %f, %s, %f, %f, %s, %f, %s, %s, %s, ",msg_.rmc_1.c_str(), msg_.utc_time_2.c_str(), msg_.status_3.c_str(), msg_.latitude_4, msg_.N_5.c_str() ,msg_.longitude_6, msg_.E_7.c_str(), msg_.spd_8 , msg_.cog_9 ,msg_.utc_day_10.c_str(), msg_.mv_11 , msg_.mvE_12.c_str(), msg_.mode_13.c_str() , msg_.cs_14.c_str());

            //-----------------------------------------------------------------------------

            if(s_vec_[2] == "A")
            {
              fix_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            }
            else
            {
              fix_.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            }

            fix_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

            fix_.latitude = s_vec_3;
            fix_.longitude = s_vec_5;

            typedef std::numeric_limits<double> Info;
            fix_.altitude = Info::quiet_NaN();

            fix_.position_covariance_type =  sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

            fix_.header.stamp = ros::Time().now();
            fix_.header.frame_id = "/fix";
            navsat_fix_pub.publish(fix_);

            //-----------------------------------------------------------------------------

        }
    }
}
}
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps");

    ros::NodeHandle private_nh("~");


    // private_nh.param(std::string("port"), port , std::string("/dev/pts/33"));
    //private_nh.param(std::string("baud"), baud,57600);
    std::string port;
    int baud;

    //=========================GPS=========================================
    if (private_nh.getParam(std::string("port"), port))
    {
    }
    else
    {
        port = "/dev/pts/18";
        ROS_INFO(" port is not setup");

    }
    cout << "port = " << port << endl;
    if( private_nh.getParam(std::string("baud"), baud ))
    {

    }else
    {
        baud = 115200;
        ROS_INFO("baud is not setup " );
    }
    robosense::sensor::GPS gps;
    ros::Publisher pub = private_nh.advertise<sensors::gprmc>("gps",100);

    gps.set(port,baud);
    gps.setHead("$GNRMC");
    gps.setPublisher(pub);
    gps.open();

    if (gps.isOpen())
    {
        ROS_INFO("port is open " );
        if(gps.checkhead())
        {
            ROS_INFO("port:%s is a gps port  ",port.c_str() );
        } else
        {
            ROS_INFO("port:%s is not a gps port ",port.c_str() );
            private_nh.shutdown();

        }

    }else
    {
        ROS_INFO("port is not open " );
        private_nh.shutdown();
    }
    //=====================================================



    // loop until shut  down or end of file
    while(private_nh.ok())
    {

        if (gps.isOpen())
        {
            gps.analysis();
        }
        ros::spinOnce();
        //================================================
    }

    return 0;

}
