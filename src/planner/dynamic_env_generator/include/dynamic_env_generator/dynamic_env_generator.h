#ifndef _DYNAMIC_ENV_GENERATOR_H_
#define _DYNAMIC_ENV_GENERATOR_H_

/* INCLUDE */
#include <iostream>
#include <vector>

/* ROS Headers */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/* PCL Headers */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

/* 动态环境生成 */
class DynamicEnvGenerator
{
    public:
        pcl::PointCloud<pcl::PointXYZ> dynamic_cloud;   // 动态物体点云
        void generate_dynamic_env(ros::NodeHandle& n);    // 建立动态环境

    private:
        /* 基本参数 */
        double voxel_resolution;    // 栅格分辨率
        int dynamic_obs_num;    // 动态障碍物个数
        std::vector<double> obs_poses; // 障碍物位置
        std::vector<double> origin_poses;
        std::vector<double> obs_sizes; // 障碍物尺寸
        std::vector<double> obs_velocities;    // 障碍物速度
        std::vector<double> moving_ranges;  
        int rings_num;  // 圆环的个数
        std::vector<double> rings_poses;    // 圆环位置
        std::vector<double> rings_origin_poses;
        std::vector<double> rings_sizes;    // 圆环尺寸
        std::vector<double> rings_velocities;   // 圆环速度 
        std::vector<double> rings_directions;    // 圆环朝向
        std::vector<double> rings_ranges;   // 圆环运动范围
        

        pcl::PointCloud<pcl::PointXYZ> generate_dynamic_obs(std::vector<double> obs_size, std::vector<double> obs_pos, double resolution, int type);  // 创建动态障碍物的点云
        pcl::PointCloud<pcl::PointXYZ> generate_ring(std::vector<double> ring_size, std::vector<double> ring_pos, double ring_direct, double resolution); // 创建圆环的点云

        /* ROS subers, timers and pubers */
        ros::NodeHandle nh;
        ros::Subscriber static_cloud_suber;
        void static_cloud_subCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
        // ros::Subscriber uav_pos_suber;
        ros::Publisher dynamic_env_cloud_puber;
        ros::Timer msg_pub_timer;
        void msg_pub_timerCallback(const ros::TimerEvent & /*event*/);
};

#endif // !_DYNAMIC_ENV_gEnERATOR_H_