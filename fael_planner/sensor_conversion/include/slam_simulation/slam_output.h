//
// Created by hjl on 2021/9/18.
//
#ifndef TOPO_PLANNER_WS_SLAM_OUTPUT_H
#define TOPO_PLANNER_WS_SLAM_OUTPUT_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class SlamOutput {
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener tf_listener;
    ros::Publisher odom_pub;
    ros::Publisher reg_pub;
    ros::Publisher dwz_cloud_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber scan_sub;
    ros::Timer global_down_timer;

    tf::StampedTransform transform;
    ros::Rate rate;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyLocalCloudOdom;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> local_cloud_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> local_odom_sub_;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>> SynchronizerLocalCloudOdom;
    SynchronizerLocalCloudOdom sync_local_cloud_odom_;

    std::string frame_id;
    std::string child_frame_id;

    bool is_get_first;

    tf::Transform T_B_W ;

    std::vector<tf::StampedTransform> ST_B_Bi_vec;
    double vec_length;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    float down_voxel_size;

    SlamOutput(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void pointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud,const nav_msgs::OdometryConstPtr& input);
};



#endif //TOPO_PLANNER_WS_SLAM_OUTPUT_H
