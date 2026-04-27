/**
 * @brief 25年电赛H题，完整巡航，通信，识别
 * @author 23届seeker战队，小怪，雷总，源神
 * @date 2025.8.24
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <queue>
#include <cmath>
#include <numeric>
#include <vector>  // 新增vector头文件

#include <t1_offboard_takeoff/Mydata.h>
#include <t1_offboard_takeoff/Point.h>
#include <t1_offboard_takeoff/Fly.h>
#include <t1_offboard_takeoff/Tdata.h>
#include <t1_offboard_takeoff/clean_wild.h>
#include "/home/seeker/seeker_ws/src/t1_offboard_takeoff/include/shared_flags.hpp"

 t1_offboard_takeoff::clean_wild clean_msg;
uint8_t is_takeoff = 0;//起飞标志位
mavros_msgs::State current_state;//状态
geometry_msgs::PoseStamped waypoints[7][9];//存储每个位置
int8_t point_array[61][2]= {{-1, -1}};      // 初始化无效值(-1,-1)，避免随机值;//转存缓存
std::vector<geometry_msgs::PoseStamped> All_waypoints;//航点
t1_offboard_takeoff::Mydata mydata;
//t1_offboard_takeoff::clean_wild clean;
int current_waypoint = 0;  // 初始化为第一个航点
//验证安全
uint8_t PX4_status=0;
uint8_t bag_sec=0;
uint8_t map_init=0;
uint8_t lidat_init=0;
uint8_t receive_three_point=0;
uint8_t pos_x1,pos_y1,pos_x2,pos_y2,pos_x3,pos_y3;
uint8_t clean_count=0;
uint8_t Is_clean=0;
struct Pos {
    int x, y;
    bool operator<(const Pos& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }
    bool operator==(const Pos& other) const {
        return x == other.x && y == other.y;
    }
};

std::vector<Pos> path;
std::set<Pos> blocked;
std::map<Pos, int> visited;
Pos cur = {6, 8};  // 起点
uint8_t count_info1=0;
t1_offboard_takeoff::Point point;
t1_offboard_takeoff::Tdata tdata_current;
t1_offboard_takeoff::Tdata plane_points;
t1_offboard_takeoff::Fly Fly_state;
uint8_t only_planone=0;
uint8_t Map_Init=0;

// 新增：返回路径队列（存储从覆盖路径终点到起点的路径）
std::queue<Pos> return_path_queue;

// 新增：临时变量，用于计算返回路径时不影响原路径
std::vector<Pos> temp_return_path; // 临时存储返回路径的点
// 在原有变量定义处添加
std::vector<Pos> simplified_return_path; // 简化后的返回路径（仅保留转折点和首尾）

// 在原有结构体/变量定义处添加
const std::vector<Pos> ADJACENT_POINTS = {{5,8}, {6,7}}; // (6,8)的有效相邻点

// 新增函数：获取有效相邻点
Pos getValidReturnPoint() {
    for (const auto& p : ADJACENT_POINTS) {
        if (!blocked.count(p)) { // 检查是否未被禁飞
            return p;
        }
    }
    return {-1, -1}; // 所有相邻点均被禁飞时返回无效点
}

// 新增：计算从终点到起点的返回路径（绕开禁飞区）
// 参数：start（覆盖路径终点）、target（起点，固定为{6,8}）
// 返回：是否成功规划路径
bool isValid(int x, int y) {
    return x >= 0 && x < 7 && y >= 0 && y < 9 && !blocked.count({x, y});
}

// 路径简化：同行/同列连续点只保留首尾，仅保留转折点
std::vector<Pos> simplifyPath(const std::vector<Pos>& original_path) {
    if (original_path.size() <= 2) return original_path; // 路径过短无需简化

    std::vector<Pos> simplified;
    simplified.push_back(original_path[0]); // 保留起点

    Pos last = original_path[0];
    // 遍历原始路径，寻找方向改变的转折点
    for (size_t i = 1; i < original_path.size() - 1; ++i) {
        Pos current = original_path[i];
        Pos next = original_path[i+1];

        // 判断当前点是否为转折点：与上一点和下一点既不同行也不同列
        bool sameRowWithLast = (current.x == last.x);
        bool sameColWithLast = (current.y == last.y);
        bool sameRowWithNext = (current.x == next.x);
        bool sameColWithNext = (current.y == next.y);

        // 仅保留转折点（方向改变）
        if (!((sameRowWithLast && sameRowWithNext) || (sameColWithLast && sameColWithNext))) {
            simplified.push_back(current);
            last = current;
        }
    }

    simplified.push_back(original_path.back()); // 保留终点
    return simplified;
}

bool computeReturnPath(Pos start, Pos target) {
    temp_return_path.clear(); // 清空临时路径
     simplified_return_path.clear(); // 清空简化路径
    if (start == target) {
        temp_return_path.push_back(start);
        return true;
    }

    // 复用BFS逻辑，计算从start到target的路径
    std::queue<std::vector<Pos>> q;
    std::set<Pos> seen;
    q.push({start});
    seen.insert(start);

    while (!q.empty()) {
        auto route = q.front(); q.pop();
        Pos last = route.back();
        if (last == target) {
            // 将路径存入临时向量（跳过起点，避免重复）
            for (size_t i = 0; i < route.size(); ++i) {
                temp_return_path.push_back(route[i]);
            }
             // 新增：对原始路径进行简化
            simplified_return_path = simplifyPath(temp_return_path);
            return true;
        }

        // 四向移动（上下左右）
        int dx[4] = {-1, 1, 0, 0};
        int dy[4] = {0, 0, -1, 1};
        for (int d = 0; d < 4; ++d) {
            int nx = last.x + dx[d];
            int ny = last.y + dy[d];
            Pos np{nx, ny};
            if (isValid(nx, ny) && !seen.count(np)) { // 验证是否合法（非禁飞区且在边界内）
                auto new_route = route;
                new_route.push_back(np);
                q.push(new_route);
                seen.insert(np);
            }
        }
    }
    return false; // 无法规划路径
}



void stepTo(Pos p) {
    path.push_back(p);
    visited[p]++;
    cur = p;
}

// 使用 BFS 规划从当前位置到目标位置的最短合法路径
bool moveTo(Pos target) {
    if (cur == target) return true;
    std::queue<std::vector<Pos>> q;
    std::set<Pos> seen;
    q.push({cur});
    seen.insert(cur);

    while (!q.empty()) {
        auto route = q.front(); q.pop();
        Pos last = route.back();
        if (last == target) {
            // 走这条路径
            for (int i = 1; i < route.size(); ++i) {
                stepTo(route[i]);
            }
            return true;
        }

        int dx[4] = {-1, 1, 0, 0};
        int dy[4] = {0, 0, -1, 1};
        for (int d = 0; d < 4; ++d) {
            int nx = last.x + dx[d];
            int ny = last.y + dy[d];
            Pos np{nx, ny};
            if (isValid(nx, ny) && !seen.count(np)) {
                auto new_route = route;
                new_route.push_back(np);
                q.push(new_route);
                seen.insert(np);
            }
        }
    }
    return false;
}

void fly_row_strategy() {
    bool right_to_left = true;
    for (int i = 6; i >= 0; --i) {
        if (right_to_left) {
            for (int j = 8; j >= 0; --j) {
                Pos p{i, j};
                if (!blocked.count(p)) {
                    moveTo(p);
                }
            }
        } else {
            for (int j = 0; j < 9; ++j) {
                Pos p{i, j};
                if (!blocked.count(p)) {
                    moveTo(p);
                }
            }
        }
        right_to_left = !right_to_left;
    }
}

void fly_col_strategy() {
    bool bottom_to_top = true;
    for (int j = 8; j >= 0; --j) {
        if (bottom_to_top) {
            for (int i = 6; i >= 0; --i) {
                Pos p{i, j};
                if (!blocked.count(p)) {
                    moveTo(p);
                }
            }
        } else {
            for (int i = 0; i < 7; ++i) {
                Pos p{i, j};
                if (!blocked.count(p)) {
                    moveTo(p);
                }
            }
        }
        bottom_to_top = !bottom_to_top;
    }
}
// 新增：将返回路径转换为航点并追加到All_waypoints
void append_return_waypoints() {
    if (simplified_return_path.empty()) return;

    // 遍历简化后的路径生成航点（替代原temp_return_path）
     for (size_t i = 1; i < simplified_return_path.size() ; ++i)
     {
        const auto& p = simplified_return_path[i];
        if (p.x >= 0 && p.x < 7 && p.y >= 0 && p.y < 9) { 
            All_waypoints.push_back(waypoints[p.x][p.y]);
        }
     }
     geometry_msgs::PoseStamped last = waypoints[6][8]; // 使用起点位置作为降落点
     last.pose.position.z =0.1;
     All_waypoints.push_back(last);
    // 打印简化前后的数量对比
    // 返程，
  
    ROS_INFO("adding extended points %d,simply %d,all points %d",
             (int)temp_return_path.size(), 
             (int)simplified_return_path.size(), 
             (int)All_waypoints.size());
}


void printVisualPath() {
    std::vector<std::vector<std::vector<int>>> index_map(7, std::vector<std::vector<int>>(9));
    for (int i = 0; i < path.size(); ++i) {
        index_map[path[i].x][path[i].y].push_back(i);
    }

    std::cout << "\n🛰️ 飞行路径可视化（重复走过加括号）：" << std::endl;
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 9; ++j) {
            if (blocked.count({i, j})) {
                std::cout << std::setw(9) << "##";
                continue;
            }

            auto& visits = index_map[i][j];
            if (visits.empty()) {
                std::cout << std::setw(9) << "..";
            } else {
                std::string label;
                // 主显示编号
                if (i == 6 && j == 8) {
                    label = "S";
                } else {
                    label = (visits[0] < 10 ? "0" : "") + std::to_string(visits[0]);
                }

                // 附加括号编号
                for (int k = 1; k < visits.size(); ++k) {
                    label += "(" + std::to_string(visits[k]) + ")";
                }

                std::cout << std::setw(9) << label;
            }
        }
        std::cout << std::endl;
    }
}

// 可视化打印返回路径
void printReturnPath(const std::vector<Pos>& return_path, Pos cover_end) {
   if (simplified_return_path.empty()) {
        ROS_WARN("返回路径为空，无法可视化");
        return;
    }

    // 遍历简化路径生成索引（替代原temp_return_path）
    std::map<Pos, int> return_index;
    for (size_t i = 0; i < simplified_return_path.size(); ++i) {
        return_index[simplified_return_path[i]] = i;
    }

    std::cout << "\n🚀 返回路径可视化（从覆盖终点E到起点S，禁飞区为##）：" << std::endl;
    for (int i = 0; i < 7; ++i) {  // 行：0（上）~6（下）
        for (int j = 0; j < 9; ++j) {  // 列：0（左）~8（右）
            Pos current_pos = {i, j};

            // 禁飞区标记
            if (blocked.count(current_pos)) {
                std::cout << std::setw(6) << "##";
                continue;
            }

            // 起点（6,8）标记为S
            if (i == 6 && j == 8) {
                std::cout << std::setw(6) << "S";
                continue;
            }

            // 覆盖路径终点（返回路径起点）标记为E
            if (current_pos == cover_end) {
                std::cout << std::setw(6) << "E";
                continue;
            }

            // 返回路径中的点，显示其索引（补0保持两位）
            if (return_index.count(current_pos)) {
                int idx = return_index[current_pos];
                std::string label = (idx < 10 ? "0" : "") + std::to_string(idx);
                std::cout << std::setw(6) << label;
            } else {
                // 非返回路径的点
                std::cout << std::setw(6) << "..";
            }
        }
        std::cout << std::endl;  // 每行结束换行
    }
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    local_pos = *msg;
}



// 将路径规划的path（Pos类型）转换为实际航点All_waypoints（PoseStamped类型）
void path_to_waypoints() {
    All_waypoints.clear();  // 清空原有航点，避免重复
    
    for (const auto& pos : path)
    {  // 遍历path中的每个点
        // 检查网格坐标是否有效（防止越界）
        
        if (pos.x < 0 || pos.x >= 7 || pos.y < 0 || pos.y >= 9)
        {
            ROS_WARN("Invalid grid position (%d, %d), skipped", pos.x, pos.y);
            continue;
        }
        
        // 从waypoints中获取该网格点对应的实际坐标，添加到All_waypoints
        geometry_msgs::PoseStamped waypoint = waypoints[pos.x][pos.y];
        point.row=pos.x;
        point.col=pos.y;
        plane_points.points.push_back(point);
        All_waypoints.push_back(waypoint);
        ROS_INFO("Added waypoint: grid(%d,%d) -> (x:%.1f, y:%.1f, z:%.1f) size:%ld",
                 pos.x, pos.y,
                 waypoint.pose.position.x,
                 waypoint.pose.position.y,
                 waypoint.pose.position.z, plane_points.points.size());
    }

    // geometry_msgs::PoseStamped point1=waypoints[0][8];
    // geometry_msgs::PoseStamped point2=waypoints[4][8];
    // geometry_msgs::PoseStamped point3=waypoints[6][8];
    // point3.pose.position.z=0.3;
    // All_waypoints.push_back(point1);
    // All_waypoints.push_back(point2);
    // All_waypoints.push_back(point3);
    // 标记航点生成完成
    if (!All_waypoints.empty())
    {
        bag_sec = 1;
        ROS_INFO("Successfully converted %zu path points to waypoints", All_waypoints.size());
    } else {
        ROS_WARN("No valid waypoints generated from path");
    }
}

void tdata_cb(const t1_offboard_takeoff::TdataConstPtr& msg)
{
    tdata_current=*msg;
    pos_x1=(uint8_t)msg->xy0/10;
    pos_y1=(uint8_t)msg->xy0%10;
    pos_x2=(uint8_t)msg->xy1/10;
    pos_y2=(uint8_t)msg->xy1%10;

    pos_x3=(uint8_t)tdata_current.xy2/10;
    pos_y3=(uint8_t)tdata_current.xy2%10;
    is_takeoff=msg->is_takeoff;
   
    if(is_takeoff==1)
    {
        if(count_info1<5){ ROS_INFO("receive takeoff");count_info1++;}  
    } 
  
    if(tdata_current.xy0>0&&tdata_current.xy1>0&&tdata_current.xy2>0&&only_planone==0)
    {
        ROS_INFO("JRGH");
        only_planone=1;

        receive_three_point=1;
        mydata.is_three_points=receive_three_point;
        blocked.insert({pos_x1, pos_y1});
        blocked.insert({pos_x2, pos_y2});
        blocked.insert({pos_x3, pos_y3});
        path.clear();
        stepTo(cur); // 起点加入路径

        if (pos_x1 == pos_x2 && pos_x2 == pos_x3)
        {
            fly_row_strategy();
        } 
        else if (pos_y1 == pos_y2 && pos_y2 == pos_y3)
        {
            fly_col_strategy();
        } 
        else
        {
            std::cout << "禁飞区必须是水平或竖直连续的三格！" << std::endl;
            return ;
        }
        if (!path.empty())
        { // 确保覆盖路径非空
            Pos cover_end = path.back(); // 覆盖路径的最后一个点
            Pos takeoff_pos = {6, 8}; // 起点

            const Pos RETURN_TARGET_X = {4, 8}; // x方向2格（6-2=4），实际x距离1米（2*0.5）
            const Pos RETURN_TARGET_Y = {6, 6}; // y方向2格（8-2=6），实际y距离1米（2*0.5）
            uint8_t compute_success = 0;
            // 计算从覆盖终点到起点的返回路径（绕开禁飞区）
            if (computeReturnPath(cover_end, RETURN_TARGET_X)) {
                // 将临时向量中的返回路径转入队列
                return_path_queue = std::queue<Pos>(); // 清空队列
                for (const auto& p : temp_return_path) {
                    return_path_queue.push(p);
                }
                ROS_INFO("成功规划返回路径，共%d个点", (int)return_path_queue.size());
                compute_success=1;

                // 打印返回路径（可选，用于调试）
                std::cout << "\n返回路径(从覆盖终点到起点）：";
                std::queue<Pos> temp_q = return_path_queue; // 临时队列用于打印
                while (!temp_q.empty()) {
                    auto p = temp_q.front();
                    temp_q.pop();
                    std::cout << "(" << p.x << "," << p.y << ") ";
                }
                printReturnPath(simplified_return_path, cover_end); // 传入简化路径
                std::cout << std::endl;
            }
            else if(compute_success==0 && computeReturnPath(cover_end, RETURN_TARGET_Y))//x轴失败，尝试y轴
            {
               
                 // 将临时向量中的返回路径转入队列
                return_path_queue = std::queue<Pos>(); // 清空队列
                for (const auto& p : temp_return_path) {
                    return_path_queue.push(p);
                }
                ROS_INFO("成功规划返回路径，共%d个点", (int)return_path_queue.size());
                compute_success=1;
                // 打印返回路径（可选，用于调试）
                std::cout << "\n返回路径(从覆盖终点到起点）：";
                std::queue<Pos> temp_q = return_path_queue; // 临时队列用于打印
                while (!temp_q.empty()) {
                    auto p = temp_q.front();
                    temp_q.pop();
                    std::cout << "(" << p.x << "," << p.y << ") ";
                }
                printReturnPath(simplified_return_path, cover_end); // 传入简化路径
                std::cout << std::endl;
            }
            else//应急情况
            {
                computeReturnPath(cover_end, takeoff_pos);
                 // 将临时向量中的返回路径转入队列
                return_path_queue = std::queue<Pos>(); // 清空队列
                for (const auto& p : temp_return_path) {
                    return_path_queue.push(p);
                }
                ROS_INFO("成功规划返回路径，共%d个点", (int)return_path_queue.size());
                compute_success=1;
                // 打印返回路径（可选，用于调试）
                std::cout << "\n返回路径(从覆盖终点到起点）：";
                std::queue<Pos> temp_q = return_path_queue; // 临时队列用于打印
                while (!temp_q.empty()) {
                    auto p = temp_q.front();
                    temp_q.pop();
                    std::cout << "(" << p.x << "," << p.y << ") ";
                }
                printReturnPath(simplified_return_path, cover_end); // 传入简化路径
                std::cout << std::endl;
                ROS_WARN("无法规划返回路径（可能被禁飞区阻挡）");
            }
        }
        path_to_waypoints();//压入规划好的航点
        append_return_waypoints();
        ROS_WARN("plan complete");
        Map_Init=1;
        printVisualPath();
     
       
      
    }//接收到三个点
   
  


}

// 检查是否达到目标位置
bool isAtTargetPosition(const geometry_msgs::PoseStamped& pose, 
                        const geometry_msgs::PoseStamped& target,
                        double tolerance = 0.1) {
    double dx = pose.pose.position.x - target.pose.position.x;
    double dy = pose.pose.position.y - target.pose.position.y;
    double dz = pose.pose.position.z - target.pose.position.z;
    return (fabs(dx) < tolerance && fabs(dy) < tolerance && fabs(dz) < 0.2);
}

int main(int argc, char **argv) {   
    ros::init(argc, argv, "offb_cfx");
    ros::NodeHandle nh;
    
    // 订阅者和发布者
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher pro_status_pub=nh.advertise<t1_offboard_takeoff::Mydata>("Mydata/is_right",10);
    ros::Publisher Is_clean_pub=nh.advertise<t1_offboard_takeoff::clean_wild>("/should_clean",10);//clean wild
    ros::Subscriber tdata_sub =nh.subscribe<t1_offboard_takeoff::Tdata>("/tdata", 10, tdata_cb);

    ros::Publisher tdata_plan_pub = nh.advertise<t1_offboard_takeoff::Tdata>("/tdata/plan", 10);
    ros::Publisher Map_Init_pub = nh.advertise<std_msgs::UInt8>("/Map_Init", 10);
    // 在main函数的订阅者/发布者定义部分添加
    ros::Publisher current_path_pos_pub = nh.advertise<t1_offboard_takeoff::Point>("/current_path_position", 10);
    
    ros::Publisher Fly_state_pub = nh.advertise<t1_offboard_takeoff::Fly>("/Fly_state", 10);
    // 服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


    ros::Rate rate(20.0);
    
    // 等待飞控连接
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to FCU");
    PX4_status=1;
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    //初始化地图位置7*9,,0-6 0-9
    for (int row = 0; row < 7; ++row)
    {       // 行索引：0（上）~6（下）
        for (int col = 0; col < 9; ++col)
        {   // 列索引：0（左）~8（右）
            // 计算x坐标：从原点(6,8)向上（行减小）每格增加0.5
            waypoints[row][col].pose.position.x = (6 - row) * 0.5;
            
            // 计算y坐标：从原点(6,8)向左（列减小）每格增加0.5
            waypoints[row][col].pose.position.y = (8 - col) * 0.5;
            
            // 固定高度z=1.2
            waypoints[row][col].pose.position.z = 0.9;
        }
    }
    
    // 发送初始设置点
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Original pose Send && map init");
    map_init=1;
    // 设置offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    // 设置解锁命令
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();

    // 航点状态变量 - 使用数组索引作为状态
    
    int sametimes = 0;
   
    
    // 降落相关变量
    const int HEIGHT_WINDOW_SIZE = 50;      // 高度数据窗口大小 (2.5秒@20Hz)
    const double HEIGHT_THRESHOLD = 0.25;   // 高度阈值 (米)
    bool landing_confirmed = false;         // 降落确认标志
    std::queue<double> height_history;      // 高度历史数据队列

    
    mydata.bag_succuss=bag_sec;
    mydata.PX4_status=PX4_status;
    mydata.map_init=map_init;
    pro_status_pub.publish(mydata);
    uint8_t flag=0;
    // std::cout << "\n 飞行路径坐标如下：" << std::endl;
        // for (auto& p : path)
        // {
        //     printf("(%d,%d) ", p.x, p.y);
        // }
        // std::cout << std::endl;
   
   
    while (!is_takeoff&&ros::ok())
    {
        if (only_planone == 1) 
        {
            pro_status_pub.publish(mydata);
            tdata_plan_pub.publish(plane_points);
            unsigned char data = 0x01;
            std_msgs::UInt8 msg;
            msg.data = data;  // 将unsigned char存入消息的data字段
            Map_Init_pub.publish(msg);  // 发布ROS消息（正确）
            Fly_state.state=1;//准备飞行
            Fly_state_pub.publish(Fly_state);
            //ROS_INFO("Publish number: %ld", plane_points.points.size());
        }
        ros::spinOnce(); // 及时处理回调
        rate.sleep(); // 按20Hz频率循环，避免阻塞
    }
    
    
    while (ros::ok())
    {
         
        
            // 状态切换逻辑
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent ) {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } 
            else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            else 
            {
                if(All_waypoints.empty())
                {
                    // 发布当前位置悬停，避免无人机失控
                    local_pos_pub.publish(local_pos);
                    ROS_WARN("empty...");
                    continue;
                } 
                // 当所有航点执行完毕，进入降落阶段
                if (current_waypoint >= All_waypoints.size())
                {
                    Fly_state.state=3;//降落
                    Fly_state_pub.publish(Fly_state);
                    // 进入降落阶段
                    // 重置降落状态变量
                    landing_confirmed = false;
                    while (!height_history.empty()) height_history.pop();
                    ROS_INFO("Starting AUTO.LAND sequence...");

                    // geometry_msgs::PoseStamped land_pose;
                    // land_pose.pose.position.z = -0.06;
                    // pose.pose.position.z= land_pose.pose.position.z;
                    // local_pos_pub.publish(pose);
                    // ROS_INFO("height %f",local_pos.pose.position.z);
                    // if(local_pos.pose.position.z<0.35)
                    // {
                    //     arm_cmd.request.value = false;
                    //     if(arming_client.call(arm_cmd) && arm_cmd.response.success)
                    //     {

                    //       ROS_INFO("Disarmed successfully");
                    //     }
                    //     else{ROS_INFO("Disarming Failed");}
                    //     last_request = ros::Time::now();

                    // }

                    // 切换到AUTO.LAND模式
                    mavros_msgs::SetMode land_set_mode;
                    land_set_mode.request.custom_mode = "AUTO.LAND";
                    
                    if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) {
                        ROS_INFO("AUTO.LAND mode enabled");
                        arm_cmd.request.value = false;
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {ROS_INFO("Disarming Successful");}
                        else{ROS_INFO("Disarming Failed");}
                        last_request = ros::Time::now(); 
                    } 
                    else 
                    {
                        ROS_WARN("Failed to switch to AUTO.LAND mode, retrying...");
                    }
                } 
                // 执行航点任务
                else
                {
                    
                    // 获取当前目标航点
                    geometry_msgs::PoseStamped current_target = All_waypoints[current_waypoint];
                    pose = current_target;
                    local_pos_pub.publish(pose);
                    Fly_state.state=2;
                    Fly_state_pub.publish(Fly_state);

                    //  t1_offboard_takeoff::clean_wild clean_msg;
                    // clean_msg.Is_Clean = 1;  // 触发清空
                    // Is_clean_pub.publish(clean_msg);
                    // clean_msg.Is_Clean = 0;
                    // 检查是否到达目标航点  
                    if (isAtTargetPosition(local_pos, current_target)) 
                    {
                        Fly_state.to_posion=1;//到达点位
                        Fly_state_pub.publish(Fly_state);
                        Fly_state.to_posion=0;//清空
                        bool is_return_waypoint = (current_waypoint >= path.size());

                         // 1. 先记录当前航点索引（未递增前）
                        int current_idx = current_waypoint;
                          // 4. 发布“当前停留点”的坐标（用递增前的索引）
                        if (current_idx < path.size())  // 注意这里用 current_idx 而非 current_waypoint
                        {
                                t1_offboard_takeoff::Point current_pos_msg;
                              
                                current_pos_msg.row = path[current_idx].x;  // 当前点的网格x
                                current_pos_msg.col = path[current_idx].y;  // 当前点的网格y
                                
                               
                              
                                if(current_idx > 0)
                                {
                                    current_pos_msg.is_need_wild=1;
                                }
                                current_path_pos_pub.publish(current_pos_msg);
                                ROS_INFO("Current path position: grid(%d, %d)", current_pos_msg.row, current_pos_msg.col);
                        }
                        if(current_idx > path.size()-1)
                        {
                                Fly_state.state=5;//返航
                                Fly_state_pub.publish(Fly_state);
                        }
                       
                        if(is_return_waypoint)
                        {
                             if(current_waypoint==All_waypoints.size()-2)
                            {
                                Fly_state.state=4;
                                 Fly_state_pub.publish(Fly_state);
                            }
                            // 返回路径：到达后直接切换下一个航点，不停留
                            current_waypoint++; 
                           
                            if (current_waypoint < All_waypoints.size()) 
                            {
                                ROS_INFO("\033[34mReached return waypoint %d, moving to waypoint %d\033[0m", 
                                        current_waypoint - 1, current_waypoint);
                            } 
                            else 
                            {
                                ROS_INFO("All return waypoints completed, preparing to land");
                            }
                        }
                        else if (sametimes++ > 1) 
                        {
                            sametimes = 0;
                            //  // 1. 先记录当前航点索引（未递增前）
                            // int current_idx = current_waypoint;

                           // 2. 发送清空指令（保持不变）
                            // clean_msg.Is_Clean=1;
                            // for(int i=0;i<10;i++){ Is_clean_pub.publish(clean_msg); }
                            // clean_msg.Is_Clean=0;

                            // 3. 切换到下一个航点
                            current_waypoint++; 

                            clean_msg.Is_Clean=1;
                            for(int i=0;i<10;i++){ Is_clean_pub.publish(clean_msg); }
                            clean_msg.Is_Clean=0;

                            // // 4. 发布“当前停留点”的坐标（用递增前的索引）
                            // if (current_idx < path.size())  // 注意这里用 current_idx 而非 current_waypoint
                            // {
                            //     t1_offboard_takeoff::Point current_pos_msg;
                            //     current_pos_msg.row = path[current_idx].x;  // 当前点的网格x
                            //     current_pos_msg.col = path[current_idx].y;  // 当前点的网格y
                              
                            //     if(current_idx > 0)
                            //     {
                            //         current_pos_msg.is_need_wild=1;
                            //     }
                            //     current_path_pos_pub.publish(current_pos_msg);
                            //     ROS_INFO("Current path position: grid(%d, %d)", current_pos_msg.row, current_pos_msg.col);
                            // }
                            // if(current_idx > path.size()-1)
                            // {
                            //     Fly_state.state=5;//返航
                            //     Fly_state_pub.publish(Fly_state);
                            // }
                                                    
                           
                            
                            if (current_waypoint < All_waypoints.size())
                            {
                                ROS_INFO("\033[32mReached waypoint %d, moving to waypoint %d\033[0m", 
                                            current_waypoint - 1, current_waypoint);

                            } 
                            else 
                            {
                                ROS_INFO("All waypoints completed, preparing to land");
                            }
                        }
                    }
                   

                }
            }



                if (current_state.mode == "OFFBOARD") 
                {
                    local_pos_pub.publish(pose);
                }
                ros::spinOnce();
                rate.sleep();
        
    }
    return 0;

}   
