/**
 * @brief 25年电赛H题，BFS路径规划
 * @author 23届seeker战队，小怪，雷总，源神
 * @date 2025.8.24
 */
#include <iostream>
#include <vector>
#include <set>
#include <tuple>
#include <iomanip>
#include <queue>
#include <map>
#include <string>  // 显式包含string头文件

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

bool isValid(int x, int y) {
    return x >= 0 && x < 7 && y >= 0 && y < 9 && !blocked.count({x, y});
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

// (3,2),(3,3),(3,4)
// (0,0),(0,1),(0,2)
// (0,0),(1,0),(2,0)
// (0,5),(1,5),(2,5)
// (5,0),(5,1),(5,2)

int main() {
    system("chcp 65001");
    std::cout << "请输入禁飞区（格式：(x1,y1),(x2,y2),(x3,y3)）：" << std::endl;
    int x1, y1, x2, y2, x3, y3;
    scanf("(%d,%d),(%d,%d),(%d,%d)", &x1, &y1, &x2, &y2, &x3, &y3);
    blocked.insert({x1, y1});
    blocked.insert({x2, y2});
    blocked.insert({x3, y3});

    stepTo(cur); // 起点加入路径

    if (x1 == x2 && x2 == x3) {
        fly_row_strategy();
    } else if (y1 == y2 && y2 == y3) {
        fly_col_strategy();
    } else {
        std::cout << "禁飞区必须是水平或竖直连续的三格！" << std::endl;
        return 1;
    }

    std::cout << "\n 飞行路径坐标如下：" << std::endl;
    for (auto& p : path) {
        printf("(%d,%d) ", p.x, p.y);
    }
    std::cout << std::endl;

    printVisualPath();
    return 0;
}
