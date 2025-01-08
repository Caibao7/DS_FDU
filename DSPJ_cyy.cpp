#include <iostream> 
#include <vector>
#include <algorithm>
#include <cstring>
#include <map>
#include <unordered_map>
#include <queue>
#include <chrono>
#include <bitset>
using namespace std;
#define MAX_N 20
#define MAX_PATHS 1000000

// 车辆结构体，每个车包含起点和终点的信息
struct Car{ 
    int start;
    int end;
};

// 存储路径
struct Path{
    int nodes[MAX_N];
    int length;
    int cost;
};

// 存储每辆车每次选什么路的组合结构体，还包含该组合的总花费
struct Combination {
    int combs[MAX_N];
    int total_cost;

    bool operator==(const Combination &other) const {
        return memcmp(combs, other.combs, sizeof(combs)) == 0;
    }
};

// 自定义组合的哈希函数
struct CombinationHash {
    size_t operator()(const Combination& c) const {
        size_t res = 0;
        for(int i = 0; i < MAX_N; ++i){
            res = res * 31 + std::hash<int>()(c.combs[i]);
        }
        return res;
    }
};

// 自定义组合的判路径是否相同的函数
struct CombinationEqual {
    bool operator()(const Combination& a, const Combination& b) const {
        return memcmp(a.combs, b.combs, sizeof(a.combs)) == 0;
    }
};

// 自定义组合的优先级比较函数，用于最小堆
struct CompareCombination {
    bool operator()(const Combination& a, const Combination& b) const {
        return a.total_cost > b.total_cost; // 最小堆
    }
};

// Traffic 类封装了所有相关的变量和函数
class Traffic{
public:
    // 成员变量
    int road_length[MAX_N][MAX_N]; // 道路长度矩阵
    int traffic_limit[MAX_N][MAX_N]; // 交通流量限制矩阵
    int N, M; // 地点数量和车辆数量
    Car cars[MAX_N]; // 车辆数组
    Path all_paths[MAX_N][MAX_PATHS]; // 行表示第i辆车，列表示第i辆车的第j条路径
    Path ans_paths[MAX_N]; // 储存每辆车的答案路径
    int path_count[MAX_N]; // 每辆车可以走的路径数量

    // 构造函数，初始化成员变量
    Traffic() : N(0), M(0) {}

    // 成员函数
    // 深度优先搜索构建所有路径
    void FindPaths(int src, int dest, bitset<MAX_N> &visited, int current_path[], int path_length, int Car_index, int current_cost) {
        // 标记当前节点为已访问
        visited.set(src);
        current_path[path_length] = src;
        path_length++;

        if (src == dest) {
            // 如果已经达到最大路径数量，停止搜索
            if (path_count[Car_index] >= MAX_PATHS) {
                visited.reset(src);
                return;
            }

            // 创建新路径并赋值
            Path new_path;
            new_path.length = path_length;
            new_path.cost = current_cost;

            for(int i = 0; i < path_length; i++) {
                new_path.nodes[i] = current_path[i];
            }

            // 存储新路径到 all_paths
            all_paths[Car_index][path_count[Car_index]++] = new_path;
        } else {
            // 遍历所有相邻节点
            for(int v = 0; v < N; v++) {
                if(!visited.test(v) && road_length[src][v] > 0){
                    // 递归调用，累加路径花费
                    FindPaths(v, dest, visited, current_path, path_length, Car_index, current_cost + road_length[src][v]);
                }
            }
        }

        // 回溯，取消访问标记
        visited.reset(src);
    }


    // 检查路径组合是否符合流量限制（基于时间步模拟）
    bool CheckComb(Path *paths, int num_paths) {
        int time_usage[MAX_N][MAX_N] = {0};
        int progress[MAX_N][3];
        int book[MAX_N] = {0};
        int is_leave[MAX_N] = {0};
        int last_progress[MAX_N][2];

        for (int i = 0; i < num_paths; i++) {
            progress[i][0] = paths[i].nodes[0];
            progress[i][1] = paths[i].nodes[1];
            progress[i][2] = road_length[progress[i][0]][progress[i][1]];
        }

        while (1) {
            bool all_done = true;
            for (int i = 0; i < num_paths; i++) {
                if (is_leave[i] == 1) {
                    time_usage[last_progress[i][0]][last_progress[i][1]]--;
                    time_usage[last_progress[i][1]][last_progress[i][0]]--;
                    is_leave[i] = 0;
                    book[i] = 0;
                }
            }

            for (int i = 0; i < num_paths; i++) {
                if (progress[i][2] > 0) {
                    all_done = false;
                    if (book[i] == 0) {
                        time_usage[progress[i][0]][progress[i][1]]++;
                        time_usage[progress[i][1]][progress[i][0]]++;
                        book[i] = 1;
                    }

                    if (time_usage[progress[i][0]][progress[i][1]] > traffic_limit[progress[i][0]][progress[i][1]]) {
                        return false;
                    }

                    progress[i][2]--;

                    if (progress[i][2] == 0) {
                        is_leave[i] = 1;
                        int current_node = progress[i][1];
                        int next_index = -1;
                        last_progress[i][0] = progress[i][0];
                        last_progress[i][1] = progress[i][1];

                        for (int j = 0; j < paths[i].length - 1; j++) {
                            if (paths[i].nodes[j] == current_node) {
                                next_index = j + 1;
                                break;
                            }
                        }

                        if (next_index != -1) {
                            progress[i][0] = current_node;
                            progress[i][1] = paths[i].nodes[next_index];
                            progress[i][2] = road_length[progress[i][0]][progress[i][1]];
                        }
                    }
                }
            }

            if (all_done) break;
        }

        return true;
    }

    // 基于贪心和BFS，从所有合法路径组合中寻找最优组合
    int FindBestComb(int M, Path all_paths[MAX_N][MAX_PATHS], int path_count[MAX_N], Path ans_paths[MAX_N]) {
        priority_queue<Combination, vector<Combination>, CompareCombination> pq; // 优先队列，用于存储所有合法路径组合，按照总成本从小到大排序
        unordered_map<Combination, bool, CombinationHash, CombinationEqual> map; // 用于记录已经访问过的组合
        Combination initial_comb;
        memset(initial_comb.combs, 0, sizeof(initial_comb.combs));
        initial_comb.total_cost = 0;
        for (int i = 0; i < M; i++) {
            initial_comb.total_cost += all_paths[i][0].cost;
        }
        pq.push(initial_comb);
        map[initial_comb] = true;

        while (!pq.empty()) {
            Combination current = pq.top();
            pq.pop();

            Path selected_paths[MAX_N];
            for (int i = 0; i < M; i++) {
                selected_paths[i] = all_paths[i][current.combs[i]];
            }

            for (int i = 0; i < M; i++) {
                selected_paths[i] = all_paths[i][current.combs[i]];
            }

            if (CheckComb(selected_paths, M)) {
                for (int i = 0; i < M; i++) {
                    ans_paths[i] = selected_paths[i];
                }
                return current.total_cost;
            }
            
            for (int i = 0; i < M; i++) {
                if (current.combs[i] + 1 < path_count[i]) {
                    Combination next = current;
                    next.combs[i]++;
                    next.total_cost += all_paths[i][next.combs[i]].cost - all_paths[i][current.combs[i]].cost;
                    if(!map[next]){
                        pq.push(next);
                        map[next] = true;
                    }
                }
            }
        }

        return -1;
    }

    // 读取输入数据
    void read_input() {
        cin >> N >> M;

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                cin >> road_length[i][j];
            }
        }

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                cin >> traffic_limit[i][j];
            }
        }

        for (int i = 0; i < M; i++) {
            cin >> cars[i].start >> cars[i].end;
            cars[i].start--;
            cars[i].end--;
        }
    }

    // 生成所有路径
    void GeneratePaths() {
        for (int i = 0; i < M; i++) {
            bitset<MAX_N> visited;
            int current_path[MAX_N];
            int path_length = 0;
            int current_cost = 0;
            FindPaths(cars[i].start, cars[i].end, visited, current_path, path_length, i, current_cost);
            sort(all_paths[i], all_paths[i] + path_count[i], [&](const Path &a, const Path &b) -> bool {
                return a.cost < b.cost;
            });
        }
    }

    // 输出结果
    void output_result(int best_cost) {
        if(best_cost == -1){
            cout << "No valid combination found.\n";
            return;
        }
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < ans_paths[i].length; j++) {
                cout << ans_paths[i].nodes[j] + 1 << " ";
            }
            cout << "\n";
        }
        cout << best_cost << "\n";
    }
};

// 实例声明为全局变量，储存在数据段中而不是栈，避免栈溢出
Traffic traffic;

// 主函数
int main() {
    // 读取输入
    traffic.read_input();

    // 获取当前时间点（开始时间）
    // auto start = std::chrono::high_resolution_clock::now();

    // 生成所有路径
    traffic.GeneratePaths();

    // 找到最佳组合
    int best_cost = traffic.FindBestComb(traffic.M, traffic.all_paths, traffic.path_count, traffic.ans_paths);

    // 输出结果
    traffic.output_result(best_cost);

    // 获取结束时间
    /*auto end = std::chrono::high_resolution_clock::now();
    // 计算程序运行时间
    std::chrono::duration<double> duration = end - start;
    // 输出程序运行时间（秒）
    std::cout << "Program run time: " << duration.count() << " seconds" << std::endl;
    */

    return 0;
}
