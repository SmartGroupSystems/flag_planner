# flag planner
轻量化的无人机规划仿真器 
## Table of Contents

* [介绍](#介绍)  
  * [1.算法框架](#jump1)
  * [2.Astar](#jump2)
  * [3.Bspline](#jump3)
  * [4.RHP](#jump4)
  * [5.nlopt](#jump5)
  
* [用法](#用法)
* [可调整的参数](#可调整的参数)
* [补充](#补充)
* [Changelog-更新日志](#Changelog-更新日志)
* [已找出的问题](#已找出的问题)

## 介绍
### <span id="jump1"> 1. 算法框架  </span>
在视觉-IMU松耦合的融合定位算法得到的位姿信息以及环境感知算法得到的栅格地图的基础上，设计基于 A*的路径生成方法，对得到的栅格地图进行膨胀以保证初始可行解的安全性；根据设计基于 B样条的轨迹优化方法，结合定时触发与碰撞触发两种逻辑实现无人机自主生成连续、光滑、安全且满足无人机动力学可行性的轨迹；针对具有有限感知范围的无人机在未知环境中自主导航的问题，构建了
Receding Horizon Planning (RHP)框架，在可信的地图范围内进行路径规划，提高了规划模块生成轨迹的鲁棒性。
<p id="struct1" align="center">
  <img src="pics/pic1.png" width = "480" height = "270"/>
</p>  

### <span id="jump2"> 2. Astar </span>
使用A*算法实现无人机当前位置到目标位置的实时路径规划。该算法维护了一个启发式估价函数：  

![Astar](https://latex.codecogs.com/svg.image?f(n)&space;=&space;g(n)&space;&plus;&space;h(n))   

该函数以最短路径为优化目标，g(n)为起始节点到当前节点的代价，h(n)为启发式函数，表示当前节点到终点的代价。考虑到计算速度与路径长度，结合所使用的栅格模型，将启发函数设置为对角线距离。

### <span id="jump3"> 3. Bspline </span>
由于四旋翼无人机具有微分平坦特性，因此本设计在四旋翼平坦空间![bsbs](https://latex.codecogs.com/svg.image?\{{x,&space;y,&space;z,&space;yaw}\})进行规划，最终解算的控制指令映射到全状态空间实现无人机实时的姿态控制。本设计采用 B 样条曲线作为运动基元生成参考轨迹，并将样条曲线控制点作为优化变量。为保证边界状态不改变，忽略起点与终点各 3 个控制点，只对![Bspline1](https://latex.codecogs.com/svg.image?\{C_2,&space;C_3,...,C_{N-3}\})进行优化。构建优化函数的表达式如下：

![Bspline2](https://latex.codecogs.com/svg.image?min&space;J_{total}&space;=&space;\lambda_sJ_s&space;&plus;&space;\lambda_cJ_c&space;&plus;&space;\lambda_fJ_f)

其中![jsjs](https://latex.codecogs.com/svg.image?J_s,J_e,J_f)分别为平滑度、碰撞度和可行性成本。![lsls](https://latex.codecogs.com/svg.image?{\lambda_s,&space;\lambda_c,&space;\lambda_f})是每个成本项的权重。平滑度成本定义为最小化 jerk 项，碰撞度成本定义为当前控制点集在欧式距离场地图（ESDF）上的和，可行性成本定义为轨迹对速度和加速度上下界的违背程度。各项成本的表达式与梯度函数均为解析函数，初始可行解由 A*算法给出。在此基础上，使用梯度下降法 `L-BFGS` 可实现快速求解。

### <span id="jump4"> 4. RHP </span>
为了在未知环境中通过局部信息实现无人机的自主导航，本方案使用 Receding Horizon Planning (RHP)不断生成轨迹，直至无人机到达最终目标。这里将滑动窗划定在从无人机当前位置到可信感知范围的最长距离dr处。当 A*规划了一条从起点到目标的路径时，RHP 框架判断当前只使用该路径的一部
分，即以无人机当前位置为圆心的半径dr范围内的路径Pr。无人机遵循定时触发与碰撞触发逻辑，在两次触发之间的一个不定的时间范围称之为执行范围$e。触发重新规划后，为了保证所生成的轨迹的平滑性，轨迹生成的起始状态由当前无人机状态![phi](https://latex.codecogs.com/svg.image?\phi(Te))确定。由于触发间隔时间Te远大于生成轨迹所需的时间，因此机器人在执行完当前轨迹后总是能够过渡以跟踪新轨迹。
<p id="struct2" align="center">
  <img src="pics/pic2.png" width = "480" height = "270"/>
</p>  



### <span id="jump5"> 5. nlopt </span>
NLOPT中有多种可以选择的算法，在头文件里面算法名称的枚举类型为
```c++
enum algorithm {  
     GN_DIRECT = 0,  
     GN_DIRECT_L,  
     GN_DIRECT_L_RAND,  
     GN_DIRECT_NOSCAL,    
     GN_DIRECT_L_NOSCAL,  
     GN_DIRECT_L_RAND_NOSCAL,  
     GN_ORIG_DIRECT,  
     GN_ORIG_DIRECT_L,  
     GD_STOGO,  
     GD_STOGO_RAND,  
     LD_LBFGS_NOCEDAL,  
     LD_LBFGS,  
     LN_PRAXIS,
     LD_VAR1,   
     LD_VAR2,  
     LD_TNEWTON,  
     LD_TNEWTON_RESTART,  
     LD_TNEWTON_PRECOND,  
     LD_TNEWTON_PRECOND_RESTART,  
     GN_CRS2_LM,    
     GN_MLSL,  
     GD_MLSL,  
     GN_MLSL_LDS,  
     GD_MLSL_LDS,  
     LD_MMA,  
     LN_COBYLA,  
     LN_NEWUOA,  
     LN_NEWUOA_BOUND,  
     LN_NELDERMEAD,  
     LN_SBPLX,  
     LN_AUGLAG,  
     LD_AUGLAG,  
     LN_AUGLAG_EQ,  
     LD_AUGLAG_EQ,  
     LN_BOBYQA,    
     GN_ISRES,  
     AUGLAG,  
     AUGLAG_EQ,  
     G_MLSL,  
     G_MLSL_LDS,  
     LD_SLSQP,   
     LD_CCSAQ,   
     GN_ESCH,  
     NUM_ALGORITHMS /*不是一种算法 只是算法的数量*/   
  };  
```
  
- 命名规律：
**G/L**代表的就是 **全局（global）** 或者 **局部（local）** 优化，N/D代表的就是 不需导数 或者 需要导数 的优化.  
例如 LN_COBYLA 就是用的 COBYLA 算法 ，然后该算法用于局部（L）无需导数（N）的优化. 
  

更多使用可以参考这个博客：  
https://www.guyuehome.com/35169


## 用法
该项目已经在Ubuntu 18.04(ROS Melodic)上进行了测试。运行以下命令进行配置:  
```linux-kernel-module
sudo apt-get install ros-melodic-nlopt
cd ${YOUR_WORKSPACE_PATH}
git clone https://github.com/FLAGDroneracing/flag_planner.git
catkin_make
```

编译完成后，需要在`.bashrc`文件中最后一行添加命令：
```
source ${YOUR_WORKSPACE_PATH}/devel/setup.bash 
eval "$RUN_AFTER_BASHRC"
```

重启终端或运行`source ~/.bashrc`以应用改动。通过以下方式启动仿真器:    
```linux-kernel-module
./static_planner.sh
```
 ```rviz``` 中会生成随机地图与无人机，使用```2D Nav Goal```为无人机选择目标。这里展示了一个模拟示例：

<!-- add some gif here -->
 <p id="gif1" align="center">
  <img src="pics/gif1.gif" width = "480" height = "270"/>
 </p>

## 可调整的参数
### Astar
文件 `src/grid_path_searcher/launch/astar_node.launch`：  
```xml
...
<arg name="resolution" default="0.2"/> <!-- 栅格边长 -->

<arg name="map_size_x" default="70.0"/>
<arg name="map_size_y" default="70.0"/>
<arg name="map_size_z" default=" 5.0"/> <!-- 地图尺寸 -->

<arg name="start_x" default=" 0.0"/>
<arg name="start_y" default=" 0.0"/> <!-- 起点 -->
<!-- <arg name="start_z" default=" 1.0"/> -->

<arg name="sight_radius" default=" 5.0"/> <!-- 视野半径 -->
<arg name="interval" default=" 0.5"/>    <!-- 重规划半径 -->
...
```
### Bspline
文件 `src/bspline_race/launch/traj_testing.launch`：  
```xml
...
  <arg name="traj_order"     value="3" />       <!-- 轨迹阶数 -->
  <arg name="dimension"      value="2" />       <!-- 2维 -->
  <arg name="TrajSampleRate" value="50" />      <!-- 每段点数量 10, 20, 30 , ... ,-->
  <arg name="max_vel"        value="3.0" />
  <arg name="max_acc"        value="5.0" />     <!-- 最大速度与加速度 -->

  <arg name="goal_x" 			   value="9.0" />
  <arg name="goal_y" 			   value="-10.0" />
  <arg name="lambda1" 			 value="5.0" />
  <arg name="lambda2" 			 value="1.0" />
  <arg name="lambda3" 			 value="10.0" />    <!-- 光滑 可行 避障权重 -->
  <arg name="esdf_collision" value="1.8" />     <!-- 小于此值则认为轨迹有碰撞 -->
  <arg name="frame" 			   value="world" />   <!-- 参考系 -->
  <arg name="map_resolution" value="0.1" />
  <arg name="start_x" 			 value="-39.95" />
  <arg name="start_y" 			 value="39.95" />   <!-- 地图起始点 -->
  <arg name="safe_distance"  value="6.0" />     <!-- 安全距离 -->
  <arg name="dist_p" 			   value="0.5" />     <!-- 均匀b样条每段长度：0.5 -->
  <arg name="TrajSampleRate" value="50" />
  <arg name="max_vel" 			 value="3.0" />
  <arg name="max_acc" 			 value="5.0" />
...
```
## 补充 
安装plotjuggler  
```
sudo apt-get install ros-melodic-plotjuggler 
```
点击`layout`,在界面中选中`smk.xml`文件，添加对应窗口：
<!-- add some gif here -->
 <p id="gif2" align="center">
  <img src="pics/gif2.gif" width = "480" height = "270"/>
 </p>

参考文章：https://blog.csdn.net/qq_39779233/article/details/106478608  
## 已找出的问题
~~1. 到达终点时可能会有小幅波动~~

~~2. `control_bspline` 有时会卡死~~

3. 斜向曲线有误差
## Changelog-更新日志
待补充

