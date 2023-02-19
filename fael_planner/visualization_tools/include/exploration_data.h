//
// Created by hjl on 2021/7/30.
//

#ifndef MY_PLANNER_WS_EXPLORATION_DATA_H
#define MY_PLANNER_WS_EXPLORATION_DATA_H
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include <iostream>
#include <fstream>

#include <ufomap_manager/UfomapWithFrontiers.h>
#include <visualization_tools/ExploredVolumeTime.h>
#include <visualization_tools/ExploredVolumeTravedDist.h>
#include <visualization_tools/TravedDistTime.h>
#include <visualization_tools/ExploredVolumeTravedDistTime.h>
#include <visualization_tools/IterationTime.h>

class exploration_data
{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber odom_sub;
    ros::Subscriber map_frontiers_sub;
    ros::Subscriber iteration_time_sub;
    ros::Subscriber init_sub;
    ros::Subscriber finish_sub;

    ros::Publisher traved_dist_pub;
    ros::Publisher explorated_volume_traved_dist_time_pub;
    ros::Publisher explorated_volume_pub;
    ros::Publisher iteration_time_pub;
    ros::Publisher explore_finish_pub;

    ros::Timer pub_timer;

    double path_length_sum;
    tf::Vector3 last_position;

    std::size_t known_cell_num;
    std::size_t known_plane_cell_num;
    double known_map_resolution;
    double known_space_volume;

    bool system_inited;
    double system_init_time;
    double current_time;

    double max_time;
    bool exploration_finish;
    double finish_time;

    int seq;

    double max_volume;

    std::string distance_volume_txt_name;
    std::string iteration_time_txt_name;
    std::string trajectory_txt_name;

    double sum_iteration_time;
    int iteration_num;

    double sum_path_find_time;
    int path_find_num;

    explicit exploration_data(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void explorationInitCallback(const std_msgs::Float64ConstPtr &init);

    void explorationFinishCallback(const std_msgs::Float64ConstPtr &finish);

    void odomCallback(const nav_msgs::OdometryConstPtr &input);

    void pubExplorationData(const ros::TimerEvent &event);

    void mapAndFrontiersCallback(const ufomap_manager::UfomapWithFrontiersConstPtr &msg);

    void iterationTimeCallback(const visualization_tools::IterationTimeConstPtr &msg);

    void pathFindTimeCallback(const visualization_tools::IterationTimeConstPtr &msg);
};

#endif // MY_PLANNER_WS_EXPLORATION_DATA_H
