// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
// #include <std_msgs/String.h>
// #include <queue>
// #include <cmath>
// #include <numeric>
// #include <vector>  // 新增vector头文件
// #include <t1_offboard_takeoff/Tdata.h>

// bool is_takeoff = false;
// uint8_t mission=0;

// mavros_msgs::State current_state;
// void state_cb(const mavros_msgs::State::ConstPtr& msg) {
//     current_state = *msg;
// }

// geometry_msgs::PoseStamped local_pos;
// void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//     local_pos = *msg;
// }

// std_msgs::String qr_current_num;
// void qr_number_cb(const std_msgs::String::ConstPtr& msg)
// {
//     qr_current_num=*msg;
//     ROS_INFO("QR: %s",qr_current_num.data.c_str());
// }
// t1_offboard_takeoff::Tdata tdata_current;
// void tdata_cb(const t1_offboard_takeoff::TdataConstPtr& msg)
// {
//     tdata_current=*msg;
//     is_takeoff=tdata_current.is_takeoff;
//     mission=tdata_current.mission_mode;
//     ROS_INFO("mission: %d",mission);
//     ROS_INFO("istakeoff: %d",is_takeoff);
// }

// // 检查是否达到目标位置
// bool isAtTargetPosition(const geometry_msgs::PoseStamped& pose, 
//                         const geometry_msgs::PoseStamped& target,
//                         double tolerance = 0.1) {
//     double dx = pose.pose.position.x - target.pose.position.x;
//     double dy = pose.pose.position.y - target.pose.position.y;
//     double dz = pose.pose.position.z - target.pose.position.z;
//     return (fabs(dx) < tolerance && fabs(dy) < tolerance && fabs(dz) < tolerance);
// }

// int main(int argc, char **argv) {   
//     ros::init(argc, argv, "offb_cfx");
//     ros::NodeHandle nh;
    
//     // 订阅者和发布者
//     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
//     ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
//     ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
//     ros::Subscriber qr_number = nh.subscribe<std_msgs::String>("/qr_code/data", 10, qr_number_cb);
//     ros::Subscriber tdata_sub =nh.subscribe<t1_offboard_takeoff::Tdata>("/tdata", 10, tdata_cb);

//     // 服务客户端
//     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
//     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

//     ros::Rate rate(20.0);
    
//     // 等待飞控连接
//     while (ros::ok() && !current_state.connected) {
//         ros::spinOnce();
//         rate.sleep();
//     }
//     ROS_INFO("Connected to FCU");

//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = 0;
//     pose.pose.position.y = 0;
//     pose.pose.position.z = 0.1;

//     // 发送初始设置点
//     for (int i = 100; ros::ok() && i > 0; --i) {
//         local_pos_pub.publish(pose);
//         ros::spinOnce();
//         rate.sleep();
//     }
//     ROS_INFO("Original pose Send");
//     // 设置offboard模式
//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD";
    
//     // 设置解锁命令
//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;
    
//     ros::Time last_request = ros::Time::now();

//     // 航点状态变量 - 使用数组索引作为状态
//     int current_waypoint = 0;  // 初始化为第一个航点
//     int sametimes = 0;
//     int mission =0; //任务状态
    
//     // 降落相关变量
//     const int HEIGHT_WINDOW_SIZE = 50;      // 高度数据窗口大小 (2.5秒@20Hz)
//     const double HEIGHT_THRESHOLD = 0.25;   // 高度阈值 (米)
//     bool landing_confirmed = false;         // 降落确认标志
//     std::queue<double> height_history;      // 高度历史数据队列
    
//     // 定义所有目标位置 - 使用vector数组
//     std::vector<geometry_msgs::PoseStamped> waypoints;
    
//     // 初始化航点数组
//     geometry_msgs::PoseStamped takeoff_target; // 起飞点
//     takeoff_target.pose.position.x = 0;
//     takeoff_target.pose.position.y = 0;
//     takeoff_target.pose.position.z = 0.5;
//     waypoints.push_back(takeoff_target);
    
//     // geometry_msgs::PoseStamped waypoint1;
//     // waypoint1.pose.position.x = 0;
//     // waypoint1.pose.position.y = 0.75;
//     // waypoint1.pose.position.z = 0.9;
//     // waypoints.push_back(waypoint1);
    
//     // geometry_msgs::PoseStamped waypoint2;
//     // waypoint2.pose.position.x = 0;
//     // waypoint2.pose.position.y = 1.25;
//     // waypoint2.pose.position.z = 0.9;
//     // waypoints.push_back(waypoint2);
    
//     // geometry_msgs::PoseStamped waypoint3;
//     // waypoint3.pose.position.x = 0;
//     // waypoint3.pose.position.y = 1.75;
//     // waypoint3.pose.position.z = 0.9;
//     // waypoints.push_back(waypoint3);
    
//     // geometry_msgs::PoseStamped waypoint4;
//     // waypoint4.pose.position.x = 0;
//     // waypoint4.pose.position.y = 1.75;
//     // waypoint4.pose.position.z = 1.3;
//     // waypoints.push_back(waypoint4);
    
//     // geometry_msgs::PoseStamped waypoint5;
//     // waypoint5.pose.position.x = 0;
//     // waypoint5.pose.position.y = 1.25;
//     // waypoint5.pose.position.z = 1.3;
//     // waypoints.push_back(waypoint5);
    
//     // geometry_msgs::PoseStamped waypoint6;
//     // waypoint6.pose.position.x = 0;
//     // waypoint6.pose.position.y = 0.75;
//     // waypoint6.pose.position.z = 1.3;
//     // waypoints.push_back(waypoint6);

//     // //进入B面
//     // geometry_msgs::PoseStamped waypoint7;
//     // waypoint7.pose.position.x = 0;
//     // waypoint7.pose.position.y = -0.25;
//     // waypoint7.pose.position.z = 1.3;
//     // waypoints.push_back(waypoint7);

//     // geometry_msgs::PoseStamped waypoint8;
//     // waypoint8.pose.position.x = 1.75;
//     // waypoint8.pose.position.y = -0.25;
//     // waypoint8.pose.position.z = 1.4;
//     // waypoints.push_back(waypoint8);
//     // //进入六个点位
//     // //上三个点位
//     // geometry_msgs::PoseStamped waypoint9;
//     // waypoint9.pose.position.x = 1.75;
//     // waypoint9.pose.position.y = 0.75;
//     // waypoint9.pose.position.z = 1.4;
//     // waypoints.push_back(waypoint9);

//     // geometry_msgs::PoseStamped waypoint10;
//     // waypoint10.pose.position.x = 1.75;
//     // waypoint10.pose.position.y = 1.25;
//     // waypoint10.pose.position.z = 1.4;
//     // waypoints.push_back(waypoint10);

//     // geometry_msgs::PoseStamped waypoint11;
//     // waypoint11.pose.position.x = 1.75;
//     // waypoint11.pose.position.y = 1.75;
//     // waypoint11.pose.position.z = 1.4;
//     // waypoints.push_back(waypoint11);
//     // //下三个
//     // geometry_msgs::PoseStamped waypoint12;
//     // waypoint12.pose.position.x = 1.75;
//     // waypoint12.pose.position.y = 1.75;
//     // waypoint12.pose.position.z = 1.0;
//     // waypoints.push_back(waypoint12);

//     // geometry_msgs::PoseStamped waypoint13;
//     // waypoint13.pose.position.x = 1.75;
//     // waypoint13.pose.position.y = 1.25;
//     // waypoint13.pose.position.z = 1.0;
//     // waypoints.push_back(waypoint13);

//     // geometry_msgs::PoseStamped waypoint14;
//     // waypoint14.pose.position.x = 1.75;
//     // waypoint14.pose.position.y = 0.75;
//     // waypoint14.pose.position.z = 1.0;
//     // waypoints.push_back(waypoint14);

//     // //c面特殊点
//     // geometry_msgs::PoseStamped waypoint15;
//     // waypoint15.pose.position.x = 1.75;
//     // waypoint15.pose.position.y = -0.25;
//     // waypoint15.pose.position.z = 1.0;
//     // waypoints.push_back(waypoint15);

//     // geometry_msgs::PoseStamped waypoint16;
//     // waypoint16.pose.position.x = 3.5;
//     // waypoint16.pose.position.y = -0.25;
//     // waypoint16.pose.position.z = 1.0;
//     // waypoints.push_back(waypoint16);

//     // //3s
//     // geometry_msgs::PoseStamped waypoint17;
//     // waypoint17.pose.position.x = 3.5;
//     // waypoint17.pose.position.y = 0.75;
//     // waypoint17.pose.position.z = 1.0;
//     // waypoints.push_back(waypoint17);

//     // geometry_msgs::PoseStamped waypoint18;
//     // waypoint18.pose.position.x = 3.5;
//     // waypoint18.pose.position.y = 1.25;
//     // waypoint18.pose.position.z = 1.0;
//     // waypoints.push_back(waypoint18);

//     // geometry_msgs::PoseStamped waypoint19;
//     // waypoint19.pose.position.x = 3.5;
//     // waypoint19.pose.position.y = 1.75;
//     // waypoint19.pose.position.z = 1.0;
//     // waypoints.push_back(waypoint19);

//     // //x3
//     // geometry_msgs::PoseStamped waypoint20;
//     // waypoint20.pose.position.x = 3.5;
//     // waypoint20.pose.position.y = 1.75;
//     // waypoint20.pose.position.z = 1.4;
//     // waypoints.push_back(waypoint20);


//     // geometry_msgs::PoseStamped waypoint21;
//     // waypoint21.pose.position.x = 3.5;
//     // waypoint21.pose.position.y = 1.25;
//     // waypoint21.pose.position.z = 1.4;
//     // waypoints.push_back(waypoint21);

//     // geometry_msgs::PoseStamped waypoint22;
//     // waypoint22.pose.position.x=3.5;
//     // waypoint22.pose.position.y=0.75;
//     // waypoint22.pose.position.z=1.4;
//     // waypoints.push_back(waypoint22);

//     // //zd
//     // geometry_msgs::PoseStamped waypoint23;
//     // waypoint23.pose.position.x=3.5;
//     // waypoint23.pose.position.y=2.5;
//     // waypoint23.pose.position.z=1.4;
//     // waypoints.push_back(waypoint23);


//     while (ros::ok())
//     {
//         // 状态切换逻辑
//         if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
//         {
//             if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent ) {
//                 ROS_INFO("Offboard enabled");//&&is_takeoff==1
//             }
//             last_request = ros::Time::now();
//         } 
//         else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
//         {
//             if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
//                 ROS_INFO("Vehicle armed");
//             }
//             last_request = ros::Time::now();
//         }
//         else if(mission==1)
//         {

//         }
//         else 
//         {
//             // 当所有航点执行完毕，进入降落阶段
//             if (current_waypoint >= waypoints.size()) {
//                 // 进入降落阶段
//                 // 重置降落状态变量
//                 landing_confirmed = false;
//                 while (!height_history.empty()) height_history.pop();
//                 ROS_INFO("Starting AUTO.LAND sequence...");
                
//                 // 切换到AUTO.LAND模式
//                 mavros_msgs::SetMode land_set_mode;
//                 land_set_mode.request.custom_mode = "AUTO.LAND";
                
//                 if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) {
//                     ROS_INFO("AUTO.LAND mode enabled");
//                     last_request = ros::Time::now();
//                 } else {
//                     ROS_WARN("Failed to switch to AUTO.LAND mode, retrying...");
//                 }
//             } 
//             // 执行航点任务
//             else {
//                 // 获取当前目标航点
//                 geometry_msgs::PoseStamped current_target = waypoints[current_waypoint];
//                 pose = current_target;
//                 local_pos_pub.publish(pose);
                
//                 // 检查是否到达目标航点
//                 if (isAtTargetPosition(local_pos, current_target)){
//                     if (sametimes++ > 40) {
//                         sametimes = 0;
//                         current_waypoint++; // 切换到下一个航点
//                         if (current_waypoint < waypoints.size()) {
//                             ROS_INFO("Reached waypoint %d, moving to waypoint %d", 
//                                      current_waypoint-1, current_waypoint);
//                         } else {
//                             ROS_INFO("All waypoints completed, preparing to land");
//                         }
//                     }
//                 }
//             }
//         }



//         if (current_state.mode == "OFFBOARD") 
//         {
//             local_pos_pub.publish(pose);
//         }
//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }   
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include<geometry_msgs/Twist.h>

// 全局变量
mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pos; // 用于存储无人机当前位置

// 回调函数：更新无人机状态
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 回调函数：更新无人机当前位置
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    local_pos = *msg;
}


// 检查是否达到目标位置
bool isAtTargetPosition(const geometry_msgs::PoseStamped& pose, 
                        const geometry_msgs::PoseStamped& target,
                        double tolerance = 0.2) {
    double dx = pose.pose.position.x - target.pose.position.x;
    double dy = pose.pose.position.y - target.pose.position.y;
    double dz = pose.pose.position.z - target.pose.position.z;
    return (fabs(dx) < tolerance && fabs(dy) < tolerance && fabs(dz) < tolerance);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // 订阅者：无人机状态、当前位置
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);

    // 发布者：目标位置
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // 服务客户端：解锁、模式切换
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 循环频率（必须高于2Hz，满足OFFBOARD模式要求）
    ros::Rate rate(20.0);

    // 等待飞控连接
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to FCU");

    // 目标位置（可根据需求修改）
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = 0.0;  // 目标X坐标
    target_pose.pose.position.y = 0.0;  // 目标Y坐标
    target_pose.pose.position.z = 0.0;  // 目标Z坐标（1米高度）

    // 发送初始目标位置（确保进入OFFBOARD模式前有足够的设定点）
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Initial setpoints sent");

    // 模式切换请求（OFFBOARD和AUTO.LAND）
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    // 解锁请求
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 时间戳：用于控制模式切换和解锁的重试频率
    ros::Time last_request = ros::Time::now();
    // 标志位：是否已触发降落
    bool land_triggered = false;
    uint8_t is_takeoff=0;
    int current_index=0;
    std::vector<geometry_msgs::PoseStamped> waypoints;
    geometry_msgs::PoseStamped takeoff_pose;
    takeoff_pose.pose.position.x = 0.0;
    takeoff_pose.pose.position.y = 0.0;
    takeoff_pose.pose.position.z = 1.0;
    waypoints.push_back(takeoff_pose);

    while (ros::ok()) 
    {
        // 未触发降落时：执行OFFBOARD模式控制
        if (!land_triggered)
        {
            // 切换到OFFBOARD模式（每5秒重试一次）
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("OFFBOARD mode enabled");
                }
                last_request = ros::Time::now();
            }
            // 解锁（每5秒重试一次）
            else if (!current_state.armed &&
                     (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            // 未到达目标：持续发布目标位置
            else
            {
                geometry_msgs::PoseStamped current_pose=waypoints[current_index];
                target_pose=current_pose;
                local_pos_pub.publish(target_pose);

                if(isAtTargetPosition(local_pos,current_pose)&&(ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    current_index++;
                    if(current_index>=waypoints.size())
                    {
                        ROS_INFO("Reached target position, triggering AUTO.LAND");
                        land_triggered = true;  // 标记为已触发降落
                       
                    }
                    last_request = ros::Time::now();  // 重置时间戳
                }
            }
            
        }
        else 
        {
            // 切换到AUTO.LAND模式（每5秒重试一次）
            if (current_state.mode != "AUTO.LAND" &&(ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
                {
                    ROS_INFO("AUTO.LAND mode enabled");
                }
                last_request = ros::Time::now();
            }
            // 进入AUTO.LAND后：无需发布位置（飞控自动控制降落）
        }



        // 仅在OFFBOARD模式下发布目标位置   
        if(current_state.mode == "OFFBOARD")
        {
            local_pos_pub.publish(target_pose);             
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
