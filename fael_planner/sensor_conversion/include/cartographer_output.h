//
// Created by hjl on 2022/3/25.
//

#ifndef ROBO_PLANNER_WS_CARTOGRAPHER_OUTPUT_H
#define ROBO_PLANNER_WS_CARTOGRAPHER_OUTPUT_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class CartographerOutput {
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformListener tf_listener;
    ros::Publisher odom_pub;
    ros::Publisher reg_pub;
    ros::Publisher dwz_cloud_pub;
    ros::Subscriber scan_sub;
    ros::Timer aligen_timer;

    std::string frame_id;
    std::string child_frame_id;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    float down_voxel_size;

    std::vector<pcl::PointCloud<pcl::PointXYZI>> points_vector;

    CartographerOutput(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void scanCallback(const sensor_msgs::PointCloud2ConstPtr & scanIn);

    void aligenPoints(const ros::TimerEvent &event);

};


#endif //ROBO_PLANNER_WS_CARTOGRAPHER_OUTPUT_H
