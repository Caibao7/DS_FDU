# FDU24秋数据结构PJ：城市交通路径规划系统  
**姓名**：Caibao7  
## 项目简介  
实现了一个在每条路在某个时段有流量限制的条件下为多个车辆提供最优道路的算法，此处的“最优”指所有车辆从起点到终点的时间之和最短。用C++实现。详细PJ介绍与要求可见PJ文档。  
测试：通过了全部自己构造的样例以及PJ文档给出的样例，此PJ可以找到要求条件下的最复杂情况的最优解，并满足时间要求（20s）。
## 具体实现  
### 数据结构的设计  
1. **Car结构体**，每个车包含起点和终点的信息  
2. **Path结构体**，包含这个路径上的所有点以及这条路的花费  
3. **Combination结构体**，存储每辆车每次选什么路的组合结构体，还包含该组合的总花费。这里的combs[i]的值表示的是索引为i的车的索引值为combs[i]的路。  
4. **三个自定义函数**，目的是让priority_queue和unordered_map可以使用我自己定义的结构。  
### 最短时间规划的实现  
**Traffic**类：封装了所有相关的变量和函数。调试中遇到栈溢出的情况，因此考虑将实例声明为全局变量，储存在数据段中而不是栈。  
#### 成员变量  
代码中每个成员变量均有注释解释其含义。  
#### 成员函数  
1. **FindPaths**：采用DFS去构建每辆车（即每个起点->终点）的可能路径。它通过递归遍历图中的节点，记录当前路径、路径长度和累计成本。当达到终点时，将当前路径存储到 all_paths 数组中，并确保每辆车的路径数量不超过预设的 MAX_PATHS。此外，函数使用 bitset 来标记已访问的节点，防止路径中出现环路。  
2. **CheckComb**：用于验证给定的路径组合是否符合道路的流量限制。它通过模拟每辆车在各个时间步的行驶过程，记录每条道路在任意时刻的使用次数。如果在任何时间步发现某条道路的使用次数超过了其限制，则返回 false，表示该组合不合法；否则，返回 true，表示该组合符合。  
3. **FindBestComb**：利用贪心策略和优先队列（最小堆）从所有可能的路径组合中寻找总成本最低的最佳组合。它首先将初始组合（所有车辆选择第一条路径）加入优先队列，并使用一个unordered_map记录已访问的组合以避免重复检查。然后，函数不断从队列中取出当前总成本最小的组合（即队首），检查其合法性。如果合法，则更新答案路径并返回总成本；如果不合法，则尝试生成新的组合（通过增加某辆车的路径索引），若此组合映射的布尔值不是true就将其加入队列。这样可以确保找到最优解，并且有效降低了时间复杂度。  
4. **read_input**：负责从标准输入读取所有必要的数据，包括地点数量 N、车辆数量 M，道路长度矩阵、交通流量限制矩阵，以及每辆车的起点和终点信息。读取的数据被存储到相应的成员变量中。  
5. **GeneratePaths**：为每辆车生成所有可能的路径。它遍历每辆车，调用 find_all_paths 函数来构建从起点到终点的所有路径。生成路径后，函数对每辆车的所有路径按成本进行排序，以便在后续选择最佳路径组合时，能够优先考虑低成本的路径，优化了后续组合选择的效率。  
6. **output_result**：负责输出找到的最佳路径组合及其总成本。如果没有找到任何合法的路径组合，函数会输出相应的提示信息。对于每辆车，函数会输出其选择的路径节点序列，最后输出所有车辆路径的总成本。
