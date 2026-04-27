
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
#include <t1_offboard_takeoff/Tdata.h>
#include <cstdlib>
#include <cstring>
#include <t1_offboard_takeoff/Vedio.h>
#include <t1_offboard_takeoff/Point.h>
#include <t1_offboard_takeoff/Mydata.h>
#include <t1_offboard_takeoff/wild.h>
#include <t1_offboard_takeoff/Fly.h>
#include <t1_offboard_takeoff/clean_wild.h>
#include <std_msgs/UInt8.h>
int8_t pose_x=0;
int8_t pose_y=0;
serial::Serial ser;   //声明一个串口变量
int8_t number=0;

int8_t mission=0;
int8_t mian=0;
int8_t id=0;
int8_t num=0;

unsigned char p2[70];
uint8_t success=0;
uint8_t is_takeoff=0;
uint8_t is_three_point=0;
t1_offboard_takeoff::Tdata tdata;
t1_offboard_takeoff::Tdata tdata_plan_points;
bool trigger_write = false;
bool trigger_read = false;
uint8_t prev_received = 0;  // 添加这一行
uint8_t allpoint_size=0;
uint8_t xy0,xy1,xy2;
uint8_t Map_receive=0;
uint8_t wild[5]={0};
uint8_t wild_is_need=0;
uint8_t current_xy=0;
uint8_t current_fly_state=0;
uint8_t trigger_wild=0;
uint8_t is_LED=0;
uint8_t is_send=0;

void Write_Allpoints()
{
    if(allpoint_size==0){
        return;
    }
    unsigned char buffer[80];
    buffer[0]=0xD3;
    uint8_t point_count = allpoint_size;
    ROS_INFO("point_count: %d tdata_points.points.size()",allpoint_size);
    buffer[1]= static_cast<uint8_t>(allpoint_size);
    for(size_t i = 0; i < point_count; ++i)
    {
        int row = tdata_plan_points.points[i].row;
        int col = tdata_plan_points.points[i].col;
        buffer[i + 2] = static_cast<unsigned char>(row * 10 + col);
        ROS_INFO("point: %d",static_cast<unsigned char>(row * 10 + col));
    }
    buffer[point_count + 2]=0xE3;
    // for(int i=0;i<60;i++){buffer[i+2]=i;ROS_INFO("point: %d",buffer[i+2]);}
    // buffer[62]=0xE3;
    ser.write(buffer, sizeof(buffer));
    ROS_INFO("T:%d,W:%d",buffer[0],buffer[1]);

}

void Write_is_3point_msg()
{
    unsigned char buffer[3];
    buffer[0]=0xD2;buffer[2]=0xE2;//发送点已经处理提示可以起飞
    memcpy(&buffer[1],&is_three_point,sizeof(uint8_t));
   
    if(is_three_point)
    {
         ROS_INFO("Send: %d ",is_three_point);
    }
   
    ser.write(buffer,sizeof(buffer));
    
}
void Write_wild_msgs()
{
   unsigned char buffer[9];
   buffer[0]=0xFD;
   buffer[8]=0xEC;
   memcpy(&buffer[1],&current_fly_state,sizeof(uint8_t));
   memcpy(&buffer[2],&current_xy,sizeof(uint8_t));
   for(int i=0;i<5;i++)
   {
    memcpy(&buffer[i+3],&wild[i],sizeof(uint8_t));
    ROS_INFO("Send: %d ",wild[i]);
   }
//    memcpy(&buffer[8],&current_xy,sizeof(uint8_t));
 
   ser.write(buffer,sizeof(buffer));
   ROS_INFO("T:%d FLY:%d XY:%d W:%d ",buffer[0],buffer[1],buffer[2],buffer[9]);

}
void Write_Map_Init()
{
    unsigned char buffer[3];
    buffer[0]=0xDF;
    buffer[2]=0xEF;
    memcpy(&buffer[1],&Map_receive,sizeof(uint8_t));
    if(Map_receive==1){ROS_INFO("Map init success: %d ",Map_receive);}
    ser.write(buffer,sizeof(buffer));

}
void Read_Three_msg(uint8_t xy0,uint8_t xy1,uint8_t xy2)
{
    unsigned char buffer[6];
    ser.read(buffer, sizeof(buffer));
    if(buffer[0]==0xD1&&buffer[4]==0xE1){
          memcpy(&xy0,&buffer[1],sizeof(uint8_t));
          memcpy(&xy1,&buffer[2],sizeof(uint8_t));
          memcpy(&xy2,&buffer[3],sizeof(uint8_t));
          tdata.xy0=xy0;
          tdata.xy1=xy1;
          tdata.xy2=xy2;
         
          ROS_INFO("zt: %d xy0: %d xy1: %d xy2: %d zw: %d",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
    }
    
}
void Read_takeoff_msg()
{
    unsigned char buffer[3];
    ser.read(buffer,sizeof(buffer));
    if(buffer[0]==0xD0&&buffer[2]==0xE0)
    {
        memcpy(&is_takeoff,&buffer[1],sizeof(uint8_t));
        tdata.is_takeoff=is_takeoff;
        ROS_INFO("is_takeoff: %d",buffer[1]);
    }
}
void Read_EEFE_msg()
{
    unsigned char buffer[2];
    ser.read(buffer,sizeof(buffer));
    if(buffer[0]==0xFE&&buffer[1]==0xEE)
    {
        ROS_INFO("read two EEFE");
        trigger_write=1;
    }
}
// void Read_EEFE_msg()
// {
//     unsigned char buffer[2];
//     ser.read(buffer,sizeof(buffer));
//     if(buffer[0]==0xEE&&buffer[1]==0xFE)
//      {
//         ROS_INFO("read two EEFE");
//         trigger_read=1;
//      }

// }
void Read_EEDF_msg()
{
    unsigned char buffer[2];
    ser.read(buffer,sizeof(buffer));
    if(buffer[0]==0xEE&&buffer[1]==0xDF)
     {
        ROS_INFO("read two EEDF");
        trigger_wild=1;
     }
}


void Read60_msg()
{
    unsigned char buffer[70];
    ser.read(buffer, sizeof(buffer));
   
        for(int i=0;i<70;i++)
        {
           memcpy(&p2[i],&buffer[i],sizeof(uint8_t));
           ROS_INFO("p2[%d]: %d",i,p2[i]);
           tdata.points[i].row=p2[i]/10;
           tdata.points[i].col=p2[i]%10;
        }
   

}
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    local_pos = *msg;
    pose_x=local_pos.pose.position.x*10;
    pose_y=local_pos.pose.position.y*10;

    //Write_msg(pose_x,pose_y,number);
}



t1_offboard_takeoff::Vedio vedio0_data;
void vedio0_cb(const t1_offboard_takeoff::Vedio::ConstPtr& msg)
{
    vedio0_data = *msg;
    mian=vedio0_data.position;
    id=vedio0_data.ID;
    num=vedio0_data.data;
   
    ROS_INFO("ID: %d NUM:%d mian:%d",id,num,mian+1);
}
t1_offboard_takeoff::Mydata mydata;
void mydata_cb(const t1_offboard_takeoff::Mydata::ConstPtr& msg)
{
    mydata = *msg;
    is_three_point=mydata.is_three_points;
    //ROS_INFO("number: %d",is_three_point);
}



void tdata_plan_cb(const t1_offboard_takeoff::Tdata::ConstPtr& msg)
{
    tdata_plan_points = *msg;
    allpoint_size=(uint8_t)msg->points.size();
    //ROS_INFO("Received /tdata/plan: points size = %ld", msg->points.size());  // 新增调试
}
std_msgs::UInt8 Map_init;
void Map_cb(const std_msgs::UInt8::ConstPtr& msg)
{
   Map_init=*msg;
   Map_receive=Map_init.data;
//    if(Map_receive==1){ROS_INFO("Map_init=1");}
   
}
t1_offboard_takeoff::wild wild_data;
void wild_cb(const t1_offboard_takeoff::wild::ConstPtr& msg)
{
   wild_data=*msg;
   wild[0]=wild_data.E;
   wild[1]=wild_data.K;
   wild[2]=wild_data.M;
   wild[3]=wild_data.T;
   wild[4]=wild_data.W;
}
t1_offboard_takeoff::Point current_path_xy;
void current_path_cb(const t1_offboard_takeoff::Point::ConstPtr& msg)
{
   current_path_xy=*msg;
   current_xy=msg->row*10+msg->col;
   wild_is_need=msg->is_need_wild;

  
}
t1_offboard_takeoff::Fly fly_data;
void current_fly_state_cb(const t1_offboard_takeoff::Fly::ConstPtr& msg)
{
    fly_data=*msg;
    current_fly_state=fly_data.state;
    is_send=fly_data.to_posion;
    
    if(fly_data.state==3){is_LED=1;}
    //ROS_INFO("current fly state: %d",current_fly_state);
    
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"serial_node");  //初始化节点
    ros::NodeHandle n;      //创建节点句柄
    
    ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped >("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber sub_three_point=n.subscribe<t1_offboard_takeoff::Mydata>("Mydata/is_right",10,mydata_cb);
    ros::Subscriber vedio0= n.subscribe<t1_offboard_takeoff::Vedio>("/qr_code/vedio0", 10, vedio0_cb);
    //ros::Subscriber sub_allpoints=n.subscribe<t1_offboard_takeoff::Tdata>("/allpoints",10,point_cb);
    
    ros::Publisher  tdata_pub = n.advertise<t1_offboard_takeoff::Tdata>("/tdata", 10);
    ros::Publisher Is_clean_pub=n.advertise<t1_offboard_takeoff::clean_wild>("/should_clean",10);//clean wild
    ros::Subscriber tdata_plan_sub = n.subscribe<t1_offboard_takeoff::Tdata>("/tdata/plan", 10, tdata_plan_cb);
    ros::Subscriber Map_init_sub = n.subscribe<std_msgs::UInt8>("/Map_Init", 10,Map_cb);
    ros::Subscriber wild_sub = n.subscribe<t1_offboard_takeoff::wild>("/detection_node/wild_msg", 10, wild_cb);
    ros::Subscriber current_path = n.subscribe<t1_offboard_takeoff::Point>("/current_path_position",10,current_path_cb);
    ros::Subscriber current_fly_state = n.subscribe<t1_offboard_takeoff::Fly>("/Fly_state",10,current_fly_state_cb);

    // 定义串口设备路径和波特率变量
    std::string serial_port_;  // 串口设备路径，如"/dev/ttyUSB0"
    int baudrate_;             // 串口通信波特率，如115200

    // 从参数服务器获取串口配置参数，如果没有设置则使用默认值
    // 参数1：参数名称
    // 参数2：存储参数的变量
    // 参数3：默认值

    n.param<std::string>("serial_port",serial_port_,"/dev/serial0");
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
    while(ros::ok())
    {
        //Write_Allpoints();
        //Write_msg(pose_x,pose_y,number);
        // Write_wild_msgs();
        // if(count==1)
        // {
        //     ros::Duration delay(2.0);
        //     delay.sleep();
        //      for(int i=0;i<20;i++)
        //     {
        //           Write_wild_msgs();
        //     }
        // }
      
        ros::spinOnce();  
        if (ser.available())
        {
            Read_Three_msg(xy0,xy1,xy2); 
           
            if(tdata.xy0!=0&&tdata.xy1!=0&&tdata.xy2!=0)
            {
                   tdata_pub.publish(tdata);
                   ROS_INFO("%d %d %d",tdata.xy0,tdata.xy1,tdata.xy2);
            }
         
            Read_takeoff_msg();
            Read_EEFE_msg();
            Read_EEDF_msg();
           




        }
        if(is_send==1)
        {
            for(int i=0;i<5;i++)
            {
                 Write_wild_msgs();
            }
            // t1_offboard_takeoff::clean_wild clean_msg;
            // clean_msg.Is_Clean = 1;  // 触发清空
            // Is_clean_pub.publish(clean_msg);
            // clean_msg.Is_Clean = 0;

            memset(wild, 0, sizeof(wild));  // 清空数组
            is_send=0;
        }
        if(trigger_write==1)
        {
            for(int i=0;i<20;i++)
            {
                Write_Allpoints();
            }
            trigger_write=0;
           
           // count=1;
        }
        // Write_led_msg();
        
        
        // Write_Allpoints();
        // Write_is_3point_msg();//发送点已经处理提示可以起飞
        loop_rate.sleep();
    }
    
}