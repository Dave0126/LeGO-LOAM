#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <fstream>
#include <string>
#include <iomanip>  // 用于设置精度

std::ofstream poseFile;  // 全局文件流，避免每次写入时都打开关闭文件

// 设置话题名和文件路径
std::string topic_name;
std::string file_path;  // 默认路径可以通过命令行参数或参数服务器来指定

// 回调函数：保存轨迹
void path_save(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
{
    if (!poseFile.is_open()) {
        ROS_ERROR("File not open! Check if the file path is correct.");
        return;
    }

    // 计算从开始时刻到当前时刻的时间差
    static double timeStart = odomAftMapped->header.stamp.toSec();
    double timeElapsed = odomAftMapped->header.stamp.toSec() - timeStart;

    // 获取机器人位置和姿态数据
    const auto& position = odomAftMapped->pose.pose.position;
    const auto& orientation = odomAftMapped->pose.pose.orientation;

    // 以科学计数法格式化输出
    poseFile << std::fixed << std::setprecision(9)  // 精度设置为9位
          << timeElapsed << " "
          << position.z << " "
          << position.x << " "
          << position.y << " "
          << orientation.x << " "
          << orientation.y << " "
          << orientation.z << " "
          << orientation.w << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_save");
    ros::NodeHandle nh;

    // 从参数服务器或命令行获取文件路径（如果有提供）
    // if (nh.hasParam("file_path")) {
    //     nh.getParam("file_path", file_path);
    // }
    nh.param<std::string>("/lego_loam/save_path/topic_name", topic_name, "/aft_mapped_to_init");
    nh.param<std::string>("/lego_loam/save_path/file_path", file_path, "/home/mnf/Workspace/test_catkin_ws/src/LeGO-LOAM/LeGO-LOAM/data/legoloam_path_stevens_dataset_VL16.txt");

    // 打开文件并检查是否成功
    poseFile.open(file_path, std::ios::app);
    if (!poseFile.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return 1;
    }

    ROS_INFO("File opened successfully. Saving data to: %s", file_path.c_str());

    // 订阅消息
    ros::Subscriber save_path = nh.subscribe<nav_msgs::Odometry>(topic_name, 100, path_save);

    // 循环处理消息
    ros::spin();

    // 关闭文件
    poseFile.close();
    return 0;
}
