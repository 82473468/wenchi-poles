#include <sensor.h>
#include <iostream>
#include <sstream>
#include <sensors/encoders.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
using std::cout;
using std::endl;

namespace robosense {
namespace sensor {


class WhlEncoder : public Sensor {
public:
    WhlEncoder();
    void analysis();

private:
    sensors::encoders msg_;
   // nav_msgs::Odometry msg_;
    std::vector<std::string>speeds_vec_;
    std:: stringstream stream_;


};
WhlEncoder::WhlEncoder()
{
    head_ = "$WHEEL";
}


void WhlEncoder::analysis() {
    std::string wheel_msg;
    serial_.readline(wheel_msg);
    cout << "wheel msg line     " << wheel_msg <<endl;
    speeds_vec_ = split(wheel_msg,",");
    int temp;
    if(speeds_vec_.size() >=5)
    {
        if(head_==speeds_vec_[0] )
        {
            stream_.str(speeds_vec_[1] );
            stream_>>temp;   // 编码器输出 100倍转/分钟
            stream_.clear();
            msg_.lfront = temp*0.01;


            stream_.str(speeds_vec_[2] );
            stream_>>temp;
            stream_.clear();
            msg_.rfront = temp*0.01;

            stream_.str(speeds_vec_[3] );
            stream_>>temp;
            stream_.clear();
            msg_.lrear = temp*0.01;

            stream_.str(speeds_vec_[4] );
            stream_>>temp;
            stream_.clear();
            msg_.rrear = temp*0.01;

            msg_.header.stamp = ros::Time().now();
            msg_.header.frame_id = "/car";
            pub_.publish(msg_);
            ROS_INFO("msg %f, %f, %f, %f ",msg_.lfront, msg_.rfront,msg_.lrear,msg_.rrear);

        }
    }
}

}
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder");

    ros::NodeHandle private_nh("~");


    // private_nh.param(std::string("port"), port , std::string("/dev/pts/33"));
    //private_nh.param(std::string("baud"), baud,57600);
    std::string port;
    int baud;
    //==================================================================
    if (private_nh.getParam(std::string("port"), port ))
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
        ROS_DEBUG("cannot find baud ");
        baud = 115200;
        ROS_INFO("baud is not setup " );
    }
    robosense::sensor::WhlEncoder whl_encoder;
    ros::Publisher pub = private_nh.advertise<sensors::encoders>("encoders",100);
    //ros::Publisher pub = private_nh.advertise<nav_msgs::Odometry>("encoders",100);
    whl_encoder.set(port,baud);
    whl_encoder.setHead("$WHEEL");
    whl_encoder.setPublisher(pub);
    whl_encoder.open();
    if (whl_encoder.isOpen())
    {
        ROS_INFO("encoder port is open " );
        if(whl_encoder.checkhead())
        {
            ROS_INFO("port:%s is a encoder port  ",port.c_str() );
        } else
        {
            ROS_INFO("port:%s is not a encoder port  ",port.c_str() );
        }

    }else
    {
        ROS_INFO("encoder port is not open " );
    }
    //=====================================================



    // loop until shut  down or end of file
    while(private_nh.ok())
    {

        if (whl_encoder.isOpen())
        {
            whl_encoder.analysis();

        } else{
            private_nh.shutdown();
        }
        ros::spinOnce();
    }

    return 0;

}
