##使用说明
星期四, 27. 七月 2017 02:06下午 
### 1 安装第三方库 serial
在RS_Localization/3rdParty/serial
或者直接在[github]( https://github.com/wjwwood/serial.git) 上下载
```
cd youdir/serial
 mkdir build
 cmake ..
 make 
 sudo make install
```
###2 运行node
#### rosrun 
进入catkin_ws根目录
打开终端 运行命令行 
```
roscore
```
- 运行 encoder ros节点
打开一个新的终端，运行

```
source devel/setup.bash
rosrun sensors encoder _baud:=115200 _port:="/dev/ttyUSB2"
```


- 运行 gps ros节点
打开一个新的终端，运行
```
  	source devel/setup.bash
      	rosrun sensors gps _name:=rtk _baud:=115200 _port:="/dev/ttyUSB1"      //
```
- iMU
打开一个新的终端，运行
```
 source devel/setup.bash
rosrun sensors nav440 _name:=nav_IMU _baud:=38400 _port:="/dev/ttyUSB0"    //黄色惯导模块，用来做返回topic: imu
```


- 运行 千寻硅步FindM ros 节点
打开一个新的终端，运行
```



  	source devel/setup.bash
    rosrun sensors findm _name:=rtk _baud:=115200 _port:="/dev/ttyUSB0" _appKey:="510328" _appSecret:="a9a55c74fa8797e5ba9ca482d83311b9a6341ab99996050345ad43a480bfded9" _deviceId:="123343455443" _deviceType:="gps"  //
```
### Unable to open serial port /dev/ttyUSB0 ubuntu 解决办法

   权限不够由于引起的
   解决办法：通过增加udev规则来实现。步骤如下：
   创建文件:
   sudo gedit /etc/udev/rules.d/70-ttyusb.rules
   在文件内增加一行
   KERNEL=="ttyUSB[0-9]*", MODE="0666"
   保存，退出
   重新插入USB转串口设备，普通用户就有权限访问了
   
   
   保存传感器数据
   <node pkg="rosbag" type="record" name="record" output="screen" args="-O 20170704.bag /IMU/nav440 /gps/gps /wheel/encoders"/> 
   
   
   
   
   
   
   
   
   
