# GeoSafe

**GeoSafe** 是一个为应对多无人机协同吊运在狭窄复杂环境中（如仓库货架间、灾后管道内）的挑战而设计的 ROS 功能包。传统基于平移和缩放的四自由度（4-DOF）规划方法在面对此类场景时往往能力不足。

本项目提出了一种基于优化的统一框架 **GeoSafe**，通过引入额外的**旋转自由度**，将解空间扩展到五维。我们利用 MINCO 变换，通过平滑映射和惩罚函数将复杂的约束优化问题重构为无约束优化问题，从而实现了避障与队形控制的同步规划。该框架在仿真和物理实验中均被证明优于传统的采样法和 IF-based 方法。

## 主要特性

  - **扩展的五维解空间**：在传统的平移和缩放（4-DOF）基础上，创新性地引入了旋转自由度（Rotational DOF），创建了一个五维解空间，显著提高了在狭窄通道中的通过成功率。
  - **统一的无约束优化框架**：借助 MINCO 变换，将包含几何安全、动力学平衡等复杂约束的队形调整问题，转化为一个统一的无约束优化问题，实现了几何路径规划与轨迹优化的同步求解，保证了全局解的一致性并提升了计算效率。
  - **连续参数优化**：与传统分步决策的策略（如 IF 策略）不同，本方法采用连续参数优化，能够生成更平滑、动态可行的轨迹，避免了因重新计算队形而导致的停顿和负载摆动。

## 功能包结构

  - `geosafe_ros_msgs`: 项目所使用的自定义 ROS 消息。
  - `GeoSafe_Flight_node`: 实现 GeoSafe 算法的核心飞行节点，集成了完整的无约束优化流程。
  - `GeoSafeUtil`: 实现 GeoSafe 核心几何安全策略的 C++ 功能库。
  - `cooperative_controller`: 用于多无人机吊运系统的协同控制器。
  - `waypoint_generator`: 处理航路点的工具。
  - `iris-distro`: 用于通过半定规划进行迭代凸区域放大的 IRIS 算法。

## 视频演示

关于本项目的实际运行效果，我们推荐您观看相关的演示视频：https://www.youtube.com/watch?v=eZcdj34zb84

## 1\. 安装

### 1.1 环境依赖 (Software)

  - `ROS (melodic)`
  - [`CoppeliaSim (V4.3.0 for Ubuntu 18.04)`](https://www.coppeliarobotics.com/previousVersions)
  - [`catkin_simple`](https://github.com/catkin/catkin_simple)
  - [`LBFGS-Lite`](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) (本项目的优化求解器)
  - `Mosek` (IRIS 依赖)
  - `CasADi`

请参考 [CoppeliaSim 的 ROS 教程](https://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm) 来配置 CoppeliaSim 的 ROS 环境。

如果 git 的子模块没有被初始化，请先运行以下命令：

```bash
cd /你的路径/GeoSafe
git submodule update --init --recursive
```

### 1.2 硬件参考 (Hardware)

本项目在物理实验中使用了以下硬件平台，可供复现时参考：

  - **运动捕捉系统**: OptiTrack (12 摄像头 @120Hz)
  - **无人机**: Crazyflie 2.x

### 1.3 安装 IRIS, Mosek, CasADi

安装细节请参考各自的官方文档。确保依赖项已正确安装并配置环境变量。

## 2\. 编译

使用 Catkin Tools (推荐):

首先，确保 [`catkin_simple`](https://www.google.com/search?q=%5Bhttps://github.com/catkin/catkin_simple%5D\(https://github.com/catkin/catkin_simple\)) 和 `GeoSafe` 都位于 `/你的路径/catkin_ws/src` 目录下。然后，在终端中执行以下命令：

```bash
sudo apt-get install python-catkin-tools
cd /你的路径/catkin_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

或者，如果您想进行调试，请在终端中输入以下命令：

```bash
sudo apt-get install python-catkin-tools
cd /你的路径/catkin_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
catkin build
```

## 3\. 使用方法

首先，启动 ROS Master：

```bash
roscore
```

然后，启动 CoppeliaSim，打开对应的场景文件 `cross.ttt`，并开始仿真。

最后，在新的终端中运行启动文件：

```bash
cd /你的路径/catkin_ws
source devel/setup.bash 
roslaunch GeoSafe_Flight_node cross.launch 
```

实验场景文件路径：`/你的路径/GeoSafe/cooperative_controller/Experimental_scene/cross.ttt`

## 4\. 如何引用

如果您在您的研究中使用了本项工作，请引用我们的相关论文：

```bibtex
@inproceedings{li2025geosafe,
  title={{GeoSafe}: A Unified Unconstrained Multi-DOF Optimization Framework for Multi-UAV Cooperative Hoisting and Obstacle Avoidance},
  author={Li, Xingyu and Nie, Hongyu and Xu, Haoxuan and Liu, Xingrui and Tan, Zhaotong and Jiang, Chunyu and Feng, Yang and Mei, Sen},
  booktitle={Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)},
  year={2025}
}
```

## 5\. 致谢

我们衷心感谢以下开源项目提供的宝贵、高质量的代码：

[`mpl_ros`](https://github.com/sikang/mpl_ros)

[`iris-distro`](https://github.com/rdeits/iris-distro)

[`Vision-encoder-based-Payload-State-Estimator`](https://github.com/jianhengLiu/Vision-encoder-based-Payload-State-Estimator)
