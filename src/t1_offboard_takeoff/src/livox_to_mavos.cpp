/**
 * @brief 25年电赛H题，完整巡航，通信，识别
 * @author 23届seeker战队，小怪，雷总，源神
 * @date 2025.8.24
 */
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <Eigen/Eigen>
// #include <cmath>
// #include <queue>
// #include <mutex>

// // 全局变量 (添加互斥锁保护)
// Eigen::Vector3d p_lidar_body;
// Eigen::Quaterniond q_mav;
// std::mutex data_mutex;

// // 滑动窗口平均类 (用于计算初始yaw)
// class SlidingWindowAverage {
// public:
//     SlidingWindowAverage(int windowSize) 
//         : windowSize(windowSize), windowSum(0.0), windowAvg(0.0) {}

//     double addData(double newData) {
//         std::lock_guard<std::mutex> lock(queue_mutex);
        
//         // 处理数据突变
//         if (!dataQueue.empty() && std::abs(newData - dataQueue.back()) > 0.1) {
//             dataQueue = std::queue<double>();
//             windowSum = 0.0;
//         }
        
//         dataQueue.push(newData);
//         windowSum += newData;
        
//         // 维护窗口大小
//         if (dataQueue.size() > windowSize) {
//             windowSum -= dataQueue.front();
//             dataQueue.pop();
//         }
        
//         // 计算平均值
//         windowAvg = dataQueue.empty() ? 0.0 : windowSum / dataQueue.size();
//         return windowAvg;
//     }

//     int get_size() const {
//         std::lock_guard<std::mutex> lock(queue_mutex);
//         return dataQueue.size();
//     }

//     double get_avg() const {
//         std::lock_guard<std::mutex> lock(queue_mutex);
//         return windowAvg;
//     }

//     void reset() {
//         std::lock_guard<std::mutex> lock(queue_mutex);
//         dataQueue = std::queue<double>();
//         windowSum = 0.0;
//         windowAvg = 0.0;
//     }

// private:
//     int windowSize;
//     double windowSum;
//     double windowAvg;
//     mutable std::mutex queue_mutex;
//     std::queue<double> dataQueue;
// };

// // 从四元数计算yaw角 (更稳健的版本)
// double fromQuaternion2yaw(const Eigen::Quaterniond& q) {
//     // 使用标准公式计算偏航角
//     const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
//     const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
//     return std::atan2(siny_cosp, cosy_cosp);
// }

// void vins_callback(const nav_msgs::Odometry::ConstPtr &msg) {
//     std::lock_guard<std::mutex> lock(data_mutex);
    
//     // 更新位置
//     p_lidar_body = Eigen::Vector3d(
//         msg->pose.pose.position.x,
//         msg->pose.pose.position.y,
//         msg->pose.pose.position.z
//     );
    
//     // 更新姿态并归一化
//     q_mav = Eigen::Quaterniond(
//         msg->pose.pose.orientation.w,
//         msg->pose.pose.orientation.x,
//         msg->pose.pose.orientation.y,
//         msg->pose.pose.orientation.z
//     ).normalized();
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "livox_to_mavros");
//     ros::NodeHandle nh("~");
    
//     // 参数配置
//     double publish_rate;
//     int yaw_window_size;
//     double yaw_threshold;
//     double init_delay;
    
//     nh.param("publish_rate", publish_rate, 20.0);
//     nh.param("yaw_window_size", yaw_window_size, 15);
//     nh.param("yaw_threshold", yaw_threshold, 0.05); // 弧度
//     nh.param("init_delay", init_delay, 2.0); // 秒
    
//     ROS_INFO("Initialization parameters:");
//     ROS_INFO("  publish_rate: %.1f Hz", publish_rate);
//     ROS_INFO("  yaw_window_size: %d", yaw_window_size);
//     ROS_INFO("  yaw_threshold: %.4f rad", yaw_threshold);
//     ROS_INFO("  init_delay: %.1f sec", init_delay);
    
//     // 初始化滑动窗口
//     SlidingWindowAverage yaw_sw(yaw_window_size);
    
//     // 订阅Mid360的VINS里程计
//     ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>(
//         "/Odometry", 100, vins_callback);
        
//     ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>(
//         "/mavros/vision_pose/pose", 10);
    
//     ros::Rate rate(publish_rate);
    
//     // 状态变量
//     enum State { INITIALIZING, WAITING_STABILITY, RUNNING };
//     State state = INITIALIZING;
//     ros::Time start_time = ros::Time::now();
//     ros::Time last_valid_time;
//     Eigen::Quaterniond init_q = Eigen::Quaterniond::Identity();
//     double initial_yaw = 0.0;
//     int init_count = 0;
    
//     ROS_INFO("Starting initialization process...");
    
//     while (ros::ok()) {
//         ros::Time current_time = ros::Time::now();
        
//         // 获取当前数据
//         Eigen::Vector3d current_p_lidar;
//         Eigen::Quaterniond current_q_mav;
        
//         {
//             std::lock_guard<std::mutex> lock(data_mutex);
//             current_p_lidar = p_lidar_body;
//             current_q_mav = q_mav;
//         }
        
//         // 状态机处理
//         switch (state) {
//         case INITIALIZING:
//             // 等待足够的消息
//             if (init_count > 5) {
//                 state = WAITING_STABILITY;
//                 ROS_INFO("Initial data received, waiting for yaw stability...");
//             } else {
//                 init_count++;
//             }
//             break;
            
//         case WAITING_STABILITY:
//             // 计算当前yaw并加入滑动窗口
//             double current_yaw = fromQuaternion2yaw(current_q_mav);
//             double avg_yaw = yaw_sw.addData(current_yaw);
            
//             // 检查yaw稳定性
//             if (yaw_sw.get_size() == yaw_window_size) {
//                 double yaw_range = std::abs(current_yaw - avg_yaw);
                
//                 if (yaw_range < yaw_threshold) {
//                     initial_yaw = avg_yaw;
//                     init_q = Eigen::AngleAxisd(initial_yaw, Eigen::Vector3d::UnitZ());
//                     state = RUNNING;
                    
//                     ROS_INFO("Yaw stabilized! Initial yaw: %.4f rad (%.2f deg)", 
//                              initial_yaw, initial_yaw * 180.0 / M_PI);
//                     ROS_INFO("Starting data publishing...");
//                 } else {
//                     ROS_DEBUG_THROTTLE(1.0, "Yaw not stable: range=%.4f, threshold=%.4f", 
//                                       yaw_range, yaw_threshold);
//                 }
//             }
            
//             // 检查超时
//             if ((current_time - start_time).toSec() > init_delay * 2.0) {
//                 ROS_WARN("Yaw stabilization timeout, using current average yaw");
//                 initial_yaw = yaw_sw.get_avg();
//                 init_q = Eigen::AngleAxisd(initial_yaw, Eigen::Vector3d::UnitZ());
//                 state = RUNNING;
//             }
//             break;
            
//         case RUNNING:
//             // 坐标系转换
//             Eigen::Vector3d p_enu = init_q * current_p_lidar;
//             Eigen::Quaterniond q_enu = init_q * current_q_mav;
            
//             // 准备发布消息
//             geometry_msgs::PoseStamped vision;
//             vision.header.stamp = ros::Time::now();
//             vision.header.frame_id = "map";
            
//             vision.pose.position.x = p_enu.x();
//             vision.pose.position.y = p_enu.y();
//             vision.pose.position.z = p_enu.z();
            
//             vision.pose.orientation.x = q_enu.x();
//             vision.pose.orientation.y = q_enu.y();
//             vision.pose.orientation.z = q_enu.z();
//             vision.pose.orientation.w = q_enu.w();
            
//             vision_pub.publish(vision);
            
//             // 节流日志输出
//             ROS_DEBUG_THROTTLE(2.0, "Published vision pose: [%.3f, %.3f, %.3f]", 
//                               p_enu.x(), p_enu.y(), p_enu.z());
//             break;
//         }
        
//         ros::spinOnce();
//         rate.sleep();
//     }
    
//     return 0;
// }
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include<cmath>
 #include <queue>
 
Eigen::Vector3d p_lidar_body, p_enu;
Eigen::Quaterniond q_mav;
Eigen::Quaterniond q_px4_odom;

 class SlidingWindowAverage {
public:
    SlidingWindowAverage(int windowSize) : windowSize(windowSize), windowSum(0.0) {}

    double addData(double newData) {
        if(!dataQueue.empty()&&fabs(newData-dataQueue.back())>0.01){
            dataQueue = std::queue<double>();
            windowSum = 0.0;
            dataQueue.push(newData);
            windowSum += newData;
        }
        else{            
            dataQueue.push(newData);
            windowSum += newData;
        }

        // 如果队列大小超过窗口大小，弹出队列头部元素并更新窗口和队列和
        if (dataQueue.size() > windowSize) {
            windowSum -= dataQueue.front();
            dataQueue.pop();
        }
        windowAvg = windowSum / dataQueue.size();
        // 返回当前窗口内的平均值
        return windowAvg;
    }

    int get_size(){
        return dataQueue.size();
    }

    double get_avg(){
        return windowAvg;
    }

private:
    int windowSize;
    double windowSum;
    double windowAvg;
    std::queue<double> dataQueue;
};

int windowSize = 8;
SlidingWindowAverage swa=SlidingWindowAverage(windowSize);

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    q_mav = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}
 
void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    q_px4_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    swa.addData(fromQuaternion2yaw(q_px4_odom));
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_to_mavros");
    ros::NodeHandle nh("~");
 
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, vins_callback);
    ros::Subscriber px4_odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 5, px4_odom_callback);
 
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
 
 
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    ros::Time last_request = ros::Time::now();
    float init_yaw = 0.0;
    bool init_flag = 1;
    Eigen::Quaterniond init_q;
    while(ros::ok()){
        if(swa.get_size()==windowSize&&!init_flag){
            init_yaw = swa.get_avg();
            init_flag = 1;
            init_q = Eigen::AngleAxisd(init_yaw,Eigen::Vector3d::UnitZ())//des.yaw
    * Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX());
        // delete swa;
        }

        if(init_flag){
            geometry_msgs::PoseStamped vision;
            p_enu = init_q*p_lidar_body;
    
            vision.pose.position.x = p_enu[0];
            vision.pose.position.y = p_enu[1];
            vision.pose.position.z = p_enu[2];
    
            vision.pose.orientation.x = q_mav.x();
            vision.pose.orientation.y = q_mav.y();
            vision.pose.orientation.z = q_mav.z();
            vision.pose.orientation.w = q_mav.w();
    
            vision.header.stamp = ros::Time::now();
            vision_pub.publish(vision);
    
            ROS_INFO("\nposition in enu:\n   x: %.18f\n   y: %.18f\n   z: %.18f\norientation of lidar:\n   x: %.18f\n   y: %.18f\n   z: %.18f\n   w: %.18f", \
            p_enu[0],p_enu[1],p_enu[2],q_mav.x(),q_mav.y(),q_mav.z(),q_mav.w());

        }

 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}



// #include <ros/ros.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/Odometry.h>
// ros::Publisher position_pub;
// using namespace std;
// int main(int argc, char** argv) {
//     ros::init(argc, argv, "position_to_mavros");
 
//     ros::NodeHandle node("~");
 
//     geometry_msgs::PoseStamped cur_position;//创建一个名为 cur_position 的变量，用于存储当前位置信息。
 
//     position_pub = node.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
 
//     tf2_ros::Buffer tfBuffer;//创建一个坐标变换的缓存对象。
//     tf2_ros::TransformListener tfListener(tfBuffer);//创建一个监听坐标变换的对象，并将缓存对象传递给它。
 
//     //view path in rviz
//     nav_msgs::Path body_path;
//     std::string target_frame_id;
//     std::string source_frame_id;
//     //std::string target_frame_id = "carto_odom";
//     //std::string source_frame_id = "base_link";
 
//     double output_rate = 50, roll_obj = 0, pitch_obj = 0, yaw_obj = 0;
 
//     node.getParam("target_frame_id", target_frame_id);
//     node.getParam("source_frame_id", source_frame_id);
//     node.getParam("output_rate", output_rate);
//     node.getParam("roll_obj", roll_obj);
//     node.getParam("pitch_obj", pitch_obj);
//     node.getParam("yaw_obj", yaw_obj);
 
//     ROS_INFO( "target_frame_id: %s,source_frame_id: %s,output_rate: %f,roll_obj: %f,pitch_obj: %f,yaw_obj: %f",
//         target_frame_id.c_str(),source_frame_id.c_str(),output_rate,roll_obj,pitch_obj,yaw_obj);
 
//     ros::Rate rate(output_rate);
//     while (node.ok()) {
//         geometry_msgs::TransformStamped transformStamped;
//         try {
//             transformStamped = tfBuffer.lookupTransform(target_frame_id, source_frame_id,
//                 ros::Time(0), ros::Duration(3.0));//创建一个存储坐标变换信息的对象。
 
//             static tf2::Quaternion quat_obj, quat_body;//创建两个静态变量，用于存储对象的四元数。
//             quat_obj = tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);//根据获取的坐标变换信息，将旋转部分的四元数存储到 quat_obj 中。
 
//             quat_body.setRPY(roll_obj, pitch_obj, yaw_obj);//设置 quat_body 的欧拉角（滚转、俯仰和偏航）。
//             //ROS_INFO_STREAM(quat_body);
//             quat_body = quat_obj * quat_body;//将 quat_body 和 quat_obj 进行四元数乘法。
//             quat_body.normalize();//将四元数归一化，确保其长度为1。
//             //
//             cur_position.pose.position.x = transformStamped.transform.translation.x;
//             cur_position.pose.position.y = transformStamped.transform.translation.y;
//             cur_position.pose.position.z = transformStamped.transform.translation.z;
//             ROS_INFO("cur_position.pose.position.x:%f cur_position.pose.position.y:%f cur_position.pose.position.z:%f", cur_position.pose.position.x, cur_position.pose.position.y, cur_position.pose.position.z);
 
//             cur_position.pose.orientation.x = quat_body.x();
//             cur_position.pose.orientation.y = quat_body.y();
//             cur_position.pose.orientation.z = quat_body.z();
//             cur_position.pose.orientation.w = quat_body.w();
//             ROS_INFO("cur_position.pose.orientation.x:%f cur_position.pose.orientation.y:%f cur_position.pose.orientation.z:%f cur_position.pose.orientation.w:%f", cur_position.pose.orientation.x, cur_position.pose.orientation.y, cur_position.pose.orientation.z, cur_position.pose.orientation.w);
//             cur_position.header.stamp = ros::Time::now();
//             cur_position.header.frame_id = transformStamped.header.frame_id;
//             position_pub.publish(cur_position);
 
//         }
//         catch (tf2::TransformException& ex) {
//             ROS_WARN("%s", ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//         }
//         rate.sleep();
//     }
//     return 0;
// };