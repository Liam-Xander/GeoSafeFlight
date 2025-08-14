# GeoSafe  

#### \[[简体中文](https://github.com/Liam-Xander/GeoSafeFlight/blob/main/README_zh.md)\]
 
**GeoSafe** is a ROS package designed to address the challenges of multi-UAV cooperative payload transport in complex and narrow environments, such as warehouse aisles and post-disaster pipelines. Traditional 4-DOF (translation and scaling) trajectory planning methods often fall short in such constrained scenarios.

This project proposes **GeoSafe**, a unified, optimization-based framework that expands the solution space to five dimensions by introducing an additional **rotational degree of freedom**. We leverage the MINCO transformation to reformulate the complex constrained optimization problem into an unconstrained one via smooth mappings and penalty functions. This enables the simultaneous planning of obstacle avoidance and formation control, proving superior to traditional sampling-based and IF-based methods in extensive simulations and real-world experiments.

## Key Features

  - **Expanded 5-DOF Solution Space**: By innovatively adding a rotational degree of freedom to the traditional 4-DOF of translation and scaling, our framework creates a 5-dimensional solution space, significantly increasing the success rate of traversing narrow passages.
  - **Unified Unconstrained Optimization Framework**: Leveraging the MINCO transformation, we convert the constrained formation adjustment problem—including geometric safety and dynamic equilibrium—into a unified unconstrained optimization problem. This achieves a synchronized solution for geometric planning and trajectory optimization, ensuring global consistency and enhancing computational efficiency.
  - **Continuous Parameter Optimization**: Unlike step-by-step decision strategies (e.g., the IF strategy), our method employs continuous parameter optimization to generate smoother, dynamically feasible trajectories. This avoids flight pauses and payload oscillations caused by recalculating formations.

## Package Structure

  - `geosafe_ros_msgs`: Custom ROS messages used in the project.
  - `GeoSafe_Flight_node`: The core flight node that implements the GeoSafe algorithm, integrating the complete unconstrained optimization process.
  - `GeoSafeUtil`: A C++ utility library implementing the core GeoSafe geometric safety strategy.
  - `cooperative_controller`: The cooperative controller for the multi-UAV payload transportation system.
  - `waypoint_generator`: A utility for processing waypoints.
  - `iris-distro`: The IRIS (Iterative Regional Inflation by Semidefinite programming) algorithm for computing large convex regions of obstacle-free space.

## Video Demonstration

For a demonstration of the project in action, we recommend watching the related video: https://www.youtube.com/watch?v=eZcdj34zb84

## 1\. Installation

### 1.1 Prerequisites (Software)

  - `ROS (Melodic)`
  - [`CoppeliaSim (V4.3.0 for Ubuntu 18.04)`](https://www.coppeliarobotics.com/previousVersions)
  - [`catkin_simple`](https://github.com/catkin/catkin_simple)
  - [`LBFGS-Lite`](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) (The optimization solver used in this project)
  - `Mosek` (A dependency for IRIS)
  - `CasADi`

Please refer to the [CoppeliaSim ROS Tutorial](https://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm) to set up the CoppeliaSim ROS environment.

If the git submodules have not been initialized, run the following command first:

```bash
cd /PATH/TO/GeoSafe
git submodule update --init --recursive
```

### 1.2 Hardware Reference

The following hardware was used for real-world experiments and can be used as a reference for replication:

  - **Motion Capture System**: OptiTrack (12 cameras @ 120Hz)
  - **UAVs**: Crazyflie 2.x

### 1.3 Installing Dependencies

Please refer to the official documentation for IRIS, Mosek, and CasADi for detailed installation instructions. Ensure all dependencies are correctly installed and their environment variables are configured.

## 2\. Compilation

Using Catkin Tools (Recommended):

First, ensure that `catkin_simple` and `GeoSafe` are both located in your `/PATH/TO/catkin_ws/src` directory. Then, execute the following commands in your terminal:

```bash
sudo apt-get install python-catkin-tools
cd /PATH/TO/catkin_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

Alternatively, for debugging, use the following commands:

```bash
sudo apt-get install python-catkin-tools
cd /PATH/TO/catkin_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
catkin build
```

## 3\. Usage

First, start the ROS Master:

```bash
roscore
```

Next, launch CoppeliaSim, open the corresponding scene file `cross.ttt`, and start the simulation.

Finally, in a new terminal, run the launch file:

```bash
cd /PATH/TO/catkin_ws
source devel/setup.bash 
roslaunch GeoSafe_Flight_node cross.launch 
```

The experiment scene file is located at: `/PATH/TO/GeoSafe/cooperative_controller/Experimental_scene/cross.ttt`

## 4\. How to Cite

If you use this work in your research, please cite our related paper:

```bibtex
@inproceedings{li2025geosafe,
  title={{GeoSafe}: A Unified Unconstrained Multi-DOF Optimization Framework for Multi-UAV Cooperative Hoisting and Obstacle Avoidance},
  author={Li, Xingyu and Nie, Hongyu and Xu, Haoxuan and Liu, Xingrui and Tan, Zhaotong and Jiang, Chunyu and Feng, Yang and Mei, Sen},
  booktitle={Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)},
  year={2025}
}
```

## 5\. Acknowledgements

We sincerely thank the following open-source projects for providing valuable, high-quality code:

[`mpl_ros`](https://github.com/sikang/mpl_ros)

[`iris-distro`](https://github.com/rdeits/iris-distro)

[`Vision-encoder-based-Payload-State-Estimator`](https://github.com/jianhengLiu/Vision-encoder-based-Payload-State-Estimator)
