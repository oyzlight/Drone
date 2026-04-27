#无人机指南
###相关配置
硬件：
miniPC
Intel Realsense T265
mid-360
pix32 v6 Mini-Base

软件：
PX4 1.13.2
Ubuntu 20.04
Ros Neotic
Mavros

PX4的学习看官网就好，里面的教程很详细：https://docs.px4.io/
注意搭配的硬件版本，查看对应版本的教程。

视频可看浙大开源的视频：
【【完结】从0制作自主空中机器人 | 开源 | 浙江大学Fast-Lab】 
https://www.bilibili.com/video/BV1WZ4y167me/?share_source=copy_web&vd_source=1fbcc370e156b662de22ab684c0635b7


运行
roslaunch t1_offboard_takeoff off.launch

##PX4配置
####QGC地面站
chmod +x ./QGroundControl.AppImage  	//安装地面站
./QGroundControl.AppImage  		//启动地面站

##上位机配置

####安装MAVROS

sudo chmod 777 /dev/ttyACM0    //给PX4与机载电脑的USB口授权
roslaunch mavros px4.launch    //启动MAVROS


####MID360与fast-lio
参考链接：https://blog.csdn.net/m0_46182398/article/details/136855714?spm=1001.2014.3001.5506

#####MID360配置

#####运行fast-lio
roslaunch livox_ros_driver2 msg_MID360.launch 
在另一个终端中执行
roslaunch fast_lio mapping_mid360.launch

####自启动文件
参考链接：
https://blog.csdn.net/JeSuisDavid/article/details/140788795?spm=1001.2101.3001.6650.2&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ECtr-2-140788795-blog-145476361.235%5Ev43%5Epc_blog_bottom_relevance_base1&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ECtr-2-140788795-blog-145476361.235%5Ev43%5Epc_blog_bottom_relevance_base1&utm_relevant_index=2

//修改自启动文件
sudo nano /home/seeker/ros_startup.sh

//重新加载系统服务
sudo systemctl daemon-reload

//启用并启动服务
sudo systemctl enable start_ros.service	
seeker@seeker-Venus-series:~$ sudo systemctl start start_ros.service


####安装VIO
将VIO文件夹复制到工作空间的src目录下并编译

cd ~/catkin_ws
source devel/setup.bash 
//VIO文件夹的包名为px4_realsense_bridge
roslaunch px4_realsense_bridge 启动文件

启动文件：
bridge_mavros.launch: 同时启动桥接和MAVROS。
bridge.launch: 如果其他组件负责启动 MAVROS（仅启动桥接），则使用此launch文件
bridge_mavros_sitl.launch:用于模拟仿真(启动桥接, MAVROS, SITL)验证与飞控的连接。

