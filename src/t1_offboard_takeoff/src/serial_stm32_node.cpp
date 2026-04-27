
/**
 * @brief 25年电赛H题，完整巡航，通信，识别
 * @author 23届seeker战队，小怪，雷总，源神
 * @date 2025.8.24
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include "string.h"
#include <geometry_msgs/PoseStamped.h>  
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <cstdlib>
#include <cstring>
#include <t1_offboard_takeoff/Vedio.h>
#include <t1_offboard_takeoff/Point.h>
#include <t1_offboard_takeoff/Mydata.h>
#include <t1_offboard_takeoff/wild.h>
#include <t1_offboard_takeoff/Fly.h>
#include <std_msgs/UInt8.h>
#include <t1_offboard_takeoff/dxy.h>
#include <t1_offboard_takeoff/Tdata.h>
#include <t1_offboard_takeoff/Detect_animals.h>
uint8_t is_LED=0;
uint8_t is_LED_FLASH=0;
uint8_t count_flash=0;
uint8_t Detect_animals=0;
uint8_t back_fly=0;
serial::Serial ser;  
t1_offboard_takeoff::dxy dxy;
int8_t dx=0;
int8_t dy=0;
int8_t detect_flag=0;

void Write_led_msg()
{
    unsigned char buffer[3];
    buffer[0]=0x66;
    buffer[2]=0xE6;
    memcpy(&buffer[1],&is_LED,sizeof(uint8_t));
   
  
   if(is_LED){ROS_INFO("LED LIGHT");}
   ser.write(buffer,sizeof(buffer));

}
//led flash
void Write_flash_msg()
{
    unsigned char buffer[3];
    buffer[0]=0x77;
    buffer[2]=0xE7;
    memcpy(&buffer[1],&is_LED_FLASH,sizeof(uint8_t));
   
   if(is_LED_FLASH){ROS_INFO("is_LED_FLASH");}
   ser.write(buffer,sizeof(buffer));

}
t1_offboard_takeoff::Fly fly_data;
void current_fly_state_cb(const t1_offboard_takeoff::Fly::ConstPtr& msg)
{
    fly_data=*msg;
    if(fly_data.state==4){is_LED=1;}
    if(fly_data.state==5){back_fly=1;}
    //ROS_INFO("current fly state: %d",current_fly_state);
    
}
// void dxy_cb(const t1_offboard_takeoff::dxy::ConstPtr& msg)
// {
//     dxy=*msg;
//     dx=dxy.dx;
//     dy=dxy.dy;
//     detect_flag=dxy.detect_flag;
    
// }

// void Write_dxy_msg()
// { 
//     unsigned char buffer[5];
//     buffer[0]=0x77;
//     memcpy(&buffer[1],&dx,sizeof(int8_t));
//     memcpy(&buffer[2],&dy,sizeof(int8_t));
//     memcpy(&buffer[3],&detect_flag,sizeof(int8_t));
//     buffer[4]=0xE7;
//     ser.write(buffer,sizeof(buffer));
//     ROS_INFO("write dxy: %d,%d,%d",dx,dy,detect_flag);
// }
t1_offboard_takeoff::Detect_animals detect_animals;
void current_Detect_animals_cb(const t1_offboard_takeoff::Detect_animals::ConstPtr& msg)
{
  detect_animals = *msg;
  Detect_animals=msg->Is_Detected;
  if(Detect_animals>0){is_LED_FLASH=1;}
  ROS_INFO("Detect_animals: %d",Detect_animals);
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"serial_stm32_node");  //初始化节点
    ros::NodeHandle n;      //创建节点句柄
    
    // 定义串口设备路径和波特率变量
    std::string serial_port_;  // 串口设备路径，如"/dev/ttyUSB0"
    int baudrate_;             // 串口通信波特率，如115200

    ros::Subscriber current_fly_state = n.subscribe<t1_offboard_takeoff::Fly>("/Fly_state",10,current_fly_state_cb);
    ros::Subscriber current_Detect_animals=n.subscribe<t1_offboard_takeoff::Detect_animals>("/detection_node/Detect_animals",10,current_Detect_animals_cb);
    //ros::Subscriber detect_dxy = n.subscribe<t1_offboard_takeoff::dxy>("/detection_node/dxy",10,dxy_cb);
    // 从参数服务器获取串口配置参数，如果没有设置则使用默认值
    // 参数1：参数名称
    // 参数2：存储参数的变量
    // 参数3：默认值

    n.param<std::string>("serial_port",serial_port_,"/dev/stm32_com");
    n.param<int>("baudrate",baudrate_,115200);
    
    try
    {
        // 设置串口设备路径
        ser.setPort(serial_port_);
        
        // 设置串口波特率
        ser.setBaudrate(baudrate_);
        
        //数据位为8位
        ser.setBytesize(serial::bytesize_t::eightbits);         
        
        //停止位为1位        
        ser.setStopbits(serial::stopbits_t::stopbits_one);      
        
        //流控制为无
        ser.setFlowcontrol(serial::flowcontrol_t::flowcontrol_none);    

        // 设置串口超时时间：1000毫秒
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        // 应用超时设置
        ser.setTimeout(to);
        
        // 打开串口
        ser.open();
    }
    catch(serial::IOException& e)  // 捕获串口IO异常
    {
        // 打印错误信息并退出程序
        ROS_ERROR_STREAM("Unable to open port!!!");
        return -1;
    }
    if (ser.isOpen())       //检查串口是否打开
    {
        ROS_INFO_STREAM("Serial Port opened");
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open port!!!");
        return -1;
    }
    
    ros::Rate loop_rate(10); //设置循环频率为10hz

    uint8_t buffer[1];
    int count=0;
    //is_LED=1;
    while(ros::ok())
    {
       // Write_dxy_msg();
        // dx=0;
        // dy=0;
        // detect_flag=0;
        ros::spinOnce(); 
        //is_LED=1; 
        if (ser.available())
        {
           

        }
        if(is_LED&&count_flash==0)
        {
            for(int i=0;i<10;i++)
            {
                 Write_led_msg();
                 ros::Duration(0.2).sleep(); 
            }
          
            //count_flash=1;
        }
       //is_LED_FLASH=1;
        if(is_LED_FLASH==1&&back_fly!=1)
        {
             for(int i=0;i<3;i++)
            {
                 Write_flash_msg(); 
            }
            //Detect_animals=0;
            is_LED_FLASH=0;
            ROS_INFO("FLASH");
            //back_fly=0;
            //count=1;
           
        }
        loop_rate.sleep();
    }
    
}