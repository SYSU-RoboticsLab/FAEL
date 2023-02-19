//
// Created by hjl on 2021/12/30.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/impl/transforms.hpp>

class OnlineLocalMapUpdater {

public:
    OnlineLocalMapUpdater(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

    void setParams();

    void pointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud,
                                const nav_msgs::OdometryConstPtr &odom);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher local_map_pub_;

    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyLocalCloudOdom;
    std::shared_ptr <message_filters::Subscriber<sensor_msgs::PointCloud2>> local_cloud_sub_;
    std::shared_ptr <message_filters::Subscriber<nav_msgs::Odometry>> local_odom_sub_;
    typedef std::shared_ptr <message_filters::Synchronizer<SyncPolicyLocalCloudOdom>> SynchronizerLocalCloudOdom;
    SynchronizerLocalCloudOdom sync_local_cloud_odom_;

    pcl::PointCloud<pcl::PointXYZI> local_map_; 
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
    tf::TransformListener tf_listener_;


    double map_voxel_size_; 
    std::string frame_id_;
    double local_map_range_;
};


OnlineLocalMapUpdater::OnlineLocalMapUpdater(ros::NodeHandle &nh, ros::NodeHandle &nh_private):nh_(nh), nh_private_(nh_private) {

    local_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "registered_scan", 1));
    local_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odometry", 100));
    sync_local_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyLocalCloudOdom>(
            SyncPolicyLocalCloudOdom(100), *local_cloud_sub_, *local_odom_sub_));
    sync_local_cloud_odom_->registerCallback(
            boost::bind(&OnlineLocalMapUpdater::pointCloudOdomCallback, this, _1, _2));

    local_map_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("local_map", 1);

    setParams();

    voxel_filter_.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
    local_map_.clear();
}

void OnlineLocalMapUpdater::setParams() {
    nh_private_.param("local_max_range", local_map_range_, 20.0);
    nh_private_.param("map_voxel_size", map_voxel_size_, 0.05);
    nh_private_.param("frame_id", frame_id_, static_cast<std::string>("map"));
}

void OnlineLocalMapUpdater::pointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud,
                                                   const nav_msgs::OdometryConstPtr &odom) {
    geometry_msgs::Pose current_pose = odom->pose.pose;

    pcl::PointCloud<pcl::PointXYZI> scan;
    pcl::fromROSMsg(*point_cloud, scan);
    pcl::PointCloud<pcl::PointXYZI> scan_cloud;
    std::vector<int> scan_index;
    pcl::removeNaNFromPointCloud(scan, scan_cloud, scan_index);
    pcl::PointCloud<pcl::PointXYZI> transform_cloud;
    pcl_ros::transformPointCloud(frame_id_, scan_cloud, transform_cloud, tf_listener_); 

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_local_map = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    *ptr_local_map = local_map_;
    *ptr_local_map += scan_cloud; 

    pcl::PointCloud<pcl::PointXYZI>::Ptr voxelized_local_map = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    voxel_filter_.setInputCloud(ptr_local_map);
    voxel_filter_.filter(*voxelized_local_map); 

    pcl::KdTreeFLANN<pcl::PointXYZI> kd_tree;
    kd_tree.setInputCloud(voxelized_local_map);
     
    local_map_.clear();
    for (auto const &point:voxelized_local_map->points) {
        if (fabs(point.x - current_pose.position.x) < local_map_range_ / 2 &&
            fabs(point.y - current_pose.position.y) < local_map_range_ / 2 )
            local_map_.push_back(point);
    }
    sensor_msgs::PointCloud2 local_map;
    pcl::toROSMsg(local_map_, local_map);
    local_map.header.stamp = point_cloud->header.stamp;
    local_map.header.seq = point_cloud->header.seq;
    local_map.header.frame_id = frame_id_;
    local_map_pub_.publish(local_map);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "online_local_map_node");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    OnlineLocalMapUpdater online_local_map_updater(nh,nh_private);

    ros::spin();

    return 0;
}