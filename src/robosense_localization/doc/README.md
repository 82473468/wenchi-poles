# 速腾定位SDK使用指南

**所依赖的第三方库**
- PCL 1.7
- OpenCV 2.4



**集成到ROS工作区间**
-本SDK可作为ros包（包名：robosense_localization）独立运行，定位消息通过“/path”话题发布；
-本程序使用的地图为全局地图，创建地图时以获取的第一个GPS位置为原点，并按直角坐标系表示，正东为X轴， 正北为Y轴；
-要求激光雷达的Y轴与车身的正前方向平行；
-本版本程序可以实现在地图内从任意位置和角度启动，但要求提供gps信息，但并不要求使用DGPS或RTK，GPS的误差10m以下即可
-地图数据保存在data文件夹下，程序运行时通过在launch文件里面配置加载，例如

    <node pkg="Robosense_localization" name="Robosense_localization" type="Robosense_localization">
        <param name="map_path" value="$(find Robosense_localization)/data/map.pcd"/>
     </node>



**程序程序运行效果（demo）**
- 测试的launch文件保存在launch文件下，demo.launch
- 修改demo.launch的22行关于rosbag的路径
- roslaunch Robosense_localization demo.launch 
- 运行结果，只显示当前帧点云和定位轨迹