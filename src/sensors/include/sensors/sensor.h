
#ifndef __SENSOR_H
#define __SENSOR_H

#include <string>
#include <iostream>
#include <cstdio>
#include <sstream>
// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include "serial/serial.h"
#include <ros/ros.h>
namespace robosense
{
    namespace sensor
    {
        //
        std::vector<std::string> split(const  std::string& s, const std::string& delim);


        class Sensor
        {
        public:
            Sensor()
            {
                isOpen_ = false;
                baud_ = 115200;//默认
            }
            ~Sensor()
            {
                serial_.close();
            }

            void set(const  std::string &port,int baud)
            {
                port_ = port;
                if(baud==2400 || baud == 4800 || baud == 9600 ||baud==38400|| baud ==57600|| baud == 115200)
                {
                    baud_  = baud;
                }else
                {
                    std::cerr << "baud is illegal " << std::endl;
                    return;
                }
            }
            void setHead(std::string head)
            {
                head_ = head;
            }


            void open();



            bool isOpen()
            {
                return serial_.isOpen();
            }

            virtual void analysis()=0; //每个传感器根据不同的数据格式，实现度取和解析函数

            std::string port_; //端口地址
            int baud_;  //波特率
            std::string head_; //传感器数据协议头,确保连接到正确的端口
            void setPublisher(const ros::Publisher & pub)  //外部指定topic名字
            {
                pub_ = pub;
            }

            virtual bool checkhead();  // 防止连接到错误的端口

        protected:
            serial::Serial serial_;
            bool isOpen_;
            ros::Publisher pub_;



        };


    }
}

#endif //__SENSOR_H
