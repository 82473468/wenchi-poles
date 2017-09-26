//
// Created by guoleiming on 17-6-29.
//
#include<sensor.h>
namespace robosense
{
    namespace  sensor{
        std::vector<std::string> split(const  std::string& s, const std::string& delim)
        {
            std::vector<std::string> elems;
            size_t pos = 0;
            size_t len = s.length();
            size_t delim_len = delim.length();
            if (delim_len == 0) return elems;
            while (pos < len)
            {
                int find_pos = s.find(delim, pos);
                if (find_pos < 0)
                {
                    elems.push_back(s.substr(pos, len - pos));
                    break;
                }
                elems.push_back(s.substr(pos, find_pos - pos));
                pos = find_pos + delim_len;
            }
            return elems;
        }

        bool Sensor::checkhead()  // 防止连接到错误的端口
        {   std::string line;
            serial_.flushInput();
            int error = 0;
            for (int i = 0; i < 10; ++i)  //有的gps 发送多种格式的数据
            {
                serial_.readline(line);

                if (line.find(" ")!= line.npos)
                {
                    line.replace(line.find(" "),1,"");
                }

                std::cout << line ;//<<std::endl;
                if( 0 == line.find(head_))
                {
                    return true;
                }
                line.clear();
            }
            return false;

        }

        void Sensor::open()
        {
            serial_.setPort(port_);
            serial_.setBaudrate(baud_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();

        }


    }
}
