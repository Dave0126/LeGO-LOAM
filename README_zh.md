# LeGO-LOAM

该仓库包含一个轻量级和地面优化的激光雷达里程计和地图生成系统（LeGO-LOAM）的代码，适用于与 ROS 兼容的无人地面车辆（UGVs）。该系统接收来自水平放置的 Velodyne VLP-16 激光雷达的点云和可选的 IMU 数据作为输入，并实时输出 6D 位姿估计。系统的演示可在此观看 -> [YouTube 演示](https://www.youtube.com/watch?v=O3tz_ftHV48)

![观看视频](LeGO-LOAM/launch/demo.gif)



## 本地部署修改部分说明

### 测试环境

- `Ubuntu 20.04 LTS` with kernel `5.15.0-124-generic` in platform `x86_64`
- `ROS noetic: 1.16.0`
- `PCL 1.10.0`
- `opencv 4.2.0`
- `eigen 3.7.0`



### 相关错误

#### 错误1：

```
fatal error: opencv/cv.h: No such file or directory
```

`OpenCV 4` 库相关名称修改，所以需要修改 `LeGO LOAM` 的 `Utility.h` 文件：

```c++
// #include <opencv/cv.h>		// 改为下方文件
#include <opencv2/opencv.hpp>
```



#### 错误2：

```
error: #error PCL requires C++14 or above
```

如字面意思，使用的 `PCL` 版本要求 C++ 14，需要对 `Lego LOAM` 的 `CMakeLists.txt` 做修改：

```cmake
### set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")		# 改为下方配置
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
```



#### 错误3：

```
error: ‘Index’ is not a member of ‘Eigen’
```

`LeGO LOAM` 使用的 `gtsam` 版本为 `4.0.0-alpha2`，自带的 `Eigen` 版本较旧，没有 `PCL` 使用的  `Eigen::Index` 定义。

- 官方讨论区（[GitHub: issues](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/215)）提到的做法是将 `/usr/include/pcl-1.10/pcl/filters/voxel_grid.h`，在 `line 340` and `line 669`  中的 `Eigen::Index` 改为 `int`：

  ```c++
  // for (Eigen::Index ni = 0; ni < relative_coordinates.cols (); ni++)	// 改为下方代码
  for (int ni = 0; ni < relative_coordinates.cols (); ni++)
  ```

- 另一种方式是设置使用自己环境的 `Eigen`，取代 `gtsam` 自带的 `Eigen`：

  ```bash
  mkdir build && cd build
  cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
  sudo make install
  ```



#### 错误4：

```
/usr/bin/ld: cannot find -lBoost::serialization
/usr/bin/ld: cannot find -lBoost::thread
/usr/bin/ld: cannot find -lBoost::timer
/usr/bin/ld: cannot find -lBoost::chrono
```

在 `CmakeList` 中加入：

```cmake
find_package(Boost REQUIRED COMPONENTS timer thread serialization chrono)
```



#### 错误5：

在 `RVIZ` 中报错：

```
Failed to transform from frame [/camera] to frame [map]
```

官方讨论区（[GitHub: issues](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/247)）提到的做法是将 `src/` 下的所有源代码中的 `/camera` 和 `/camera_init` 替换为  `camera` 和 `camera_init`



#### 错误6：

```
lego_loam/mapOptmization: error while loading shared libraries: libmetis.so: cannot open shared object file: No such file or directory
```

官方讨论区（[GitHub: issues](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/160)）提到的做法是安装 `libparmetis-dev` 库：

```bash
sudo apt install libparmetis-dev
```





## 依赖

- ROS（已在 Indigo、Kinetic 和 Melodic 上测试）

- gtsam（乔治亚理工大学平滑和映射库，版本 4.0.0-alpha2）

  ```bash
  wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.0-alpha2/
  mkdir build && cd build
  cmake ..
  sudo make install
  ```

  

## 编译

您可以使用以下命令下载并编译该包。

```bash
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
cd ..
catkin_make -j1
```

第一次编译代码时，需要在 `catkin_make` 后添加 `-j1` 以生成某些消息类型。后续编译不需要 `-j1`。



## 系统

LeGO-LOAM 专门针对水平放置的 VLP-16 激光雷达进行优化，适用于地面车辆。它假设扫描中始终存在地面平面。我们使用的 UGV 是 Clearpath Jackal，它内置有 IMU。

![chassis](LeGO-LOAM/launch/jackal-label.jpg)

本功能包在特征提取之前执行分割。

![segmentation](LeGO-LOAM/launch/seg-total.jpg)

激光雷达里程计执行两步 Levenberg-Marquardt 优化以获得 6D 自由度变换。

![2LM](LeGO-LOAM/launch/odometry.jpg)





## 新激光雷达

适应新传感器的关键在于确保点云能够正确投影到范围图像，并能正确检测到地面。例如，VLP-16 的角分辨率为 0.2° 和 2°，并具有 16 梯度光束。底部光束的角度为 -15°。因此，在 `utility.h` 中的参数列示如下。当您实现新传感器时，确保 ground_cloud 拥有足够的点以进行匹配。在您发布任何问题之前，请先阅读以下内容。

```c++
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0;
extern const int groundScanInd = 7;
```

对于 Velodyne HDL-32e 范围图像投影的另一个示例：

```c++
extern const int N_SCAN = 32;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 360.0 / Horizon_SCAN;
extern const float ang_res_y = 41.333 / float(N_SCAN - 1);
extern const float ang_bottom = 30.666666;
extern const int groundScanInd = 20;
```

<u>***新功能***</u>: 添加了新的 `useCloudRing` 标志以帮助点云投影（例如，VLP-32C、VLS-128）。Velodyne 点云具有 `ring` 通道，直接提供范围图像中的点行 ID。其他激光雷达也可能有类似的通道，例如 Ouster 中的`r`。如果您使用的是非 Velodyne 激光雷达，但具有类似的 `ring` 通道，您可以在 `utility.h` 中更改 `PointXYZIR` 定义及 `imageProjection.cpp` 中相应的代码。

对于 **KITTI** 用户，如果您希望使用我们的算法与 HDL-64e，您需要编写自己的投影实现。如果点云没有正确投影，您将丢失许多点和性能。

如果您将激光雷达与 IMU 一起使用，请确保 IMU 与激光雷达的对齐是正确的。算法使用 IMU 数据来校正由于传感器运动造成的点云失真。如果 IMU 没有正确对齐，使用 IMU 数据将会降低结果的准确性。Ouster 激光雷达的 IMU 在该包中不受支持，因为 LeGO-LOAM 需要一个 9 自由度 IMU。



## 运行

1. 运行启动文件：

   ```bash
   roslaunch lego_loam run.launch
   ```

   注意: 参数 `/use_sim_time` 设置为 `true` 用于模拟，设置为 `false` 用于真实机器人使用。

2. 播放现有的包文件：

   ```bash
   rosbag play *.bag --clock --topic /velodyne_points /imu/data
   ```

   注意: 尽管 `/imu/data` 是可选的，但如果提供可以大大提高估计精度。一些示例的数据包可以从 [这里](https://github.com/RobustFieldAutonomyLab/jackal_dataset_20170608) 下载。



## 保存全局地图

在 `LeGO-LOAM` 模块会将每次处理之后生成的最新地图发布在话题 `/laser_cloud_surround` 中，所以我们可以使用 `rosbag` 工具来录制话题中的数据：

```bash
rosbag record -O <YOUR_BAG_FILE> /laser_cloud_surround 
```

然后使用 `ROS` 提供的工具将 `.bag` 数据转换为 `.pcd` 文件，当录制到若干帧 `/laser_cloud_surround` 中消息时，此处会有若干个 `.pcd` 点云文件，选择最新的那一个（时间戳最大）：

```bash
rosrun pcl_ros bag_to_pcd <YOUR_BAG_FILE> /laser_cloud_surround <YOUR_PCD_DIR>
ls <YOUR_PCD_DIR>
```

使用 `pcl_viewer` 工具查看点云地图：

```bash
pcl_viewer <YOUR_PCD_DIR>/xxxx.pcd
```



## 新数据集

[Stevens 数据集](https://github.com/TixiaoShan/Stevens-VLP16-Dataset)，是使用安装在无人地面车辆 Clearpath Jackal 上的 Velodyne VLP-16 捕获的，地点为史蒂文斯理工学院。VLP-16 的旋转速率设置为 10Hz。该数据集包含超过 20K 次扫描和许多回环闭合。

![data_set_demo](LeGO-LOAM/launch/dataset-demo.gif)

![google-earth](LeGO-LOAM/launch/google-earth.png)



## 更新优化

### 激光雷达惯性里程计

一个更新的激光雷达初始里程计包 [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) 已开源并可供测试。



### 回环闭合

该包中实现的回环闭合方法是简单的基于 ICP 的方法。当里程计漂移过大时，它通常会失败。对于更高级的回环闭合方法，有一个名为 [SC-LeGO-LOAM](https://github.com/irapkaist/SC-LeGO-LOAM) 的包，利用点云描述符。



### 速度优化

可以在 [这里](https://github.com/facontidavide/LeGO-LOAM/tree/speed_optimization) 找到 LeGO-LOAM 的优化版本。该目录中的改进包括但不限于：

- 改善代码质量，使其更易读、一致且易于理解和修改。
- 移除硬编码值，并使用适当的配置文件描述硬件。
- 改进性能，在计算相同结果所需的 CPU 使用量方面。
- 将多进程应用程序转换为单进程/多线程应用，使算法更确定性且略微更快。
- 更加方便和快速地处理 rosbag：处理 rosbag 应该以 CPU 允许的最大速度和确定性方式进行。
- 因为上述原因，创建单元测试和回归测试将变得更加容易。



## 引用 LeGO-LOAM

如果您使用了任何此代码，请引用的 [LeGO-LOAM 论文](https://github.com/Dave0126/LeGO-LOAM/blob/master/Shan_Englot_IROS_2018_Preprint.pdf)：

```
@inproceedings{legoloam2018,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```

