//
// Created by hjl on 2021/9/18.
//

#include "slam_simulation/slam_output.h"

SlamOutput::SlamOutput(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private),
                                                                                       rate(100), vec_length(2.0),
                                                                                       is_get_first(false) {
    const std::string &ns = ros::this_node::getName();
    frame_id = "map";
    if (!ros::param::get(ns + "/frame_id", frame_id)) {
        ROS_WARN("No frame_id specified. Looking for %s. Default is 'map'.",
                 (ns + "/frame_id").c_str());
    }

    child_frame_id = "sensor";
    if (!ros::param::get(ns + "/child_frame_id", child_frame_id)) {
        ROS_WARN("No child_frame_id specified. Looking for %s. Default is 'sensor'.",
                 (ns + "/child_frame_id").c_str());
    }

    down_voxel_size = 0.02;
    if (!ros::param::get(ns + "/down_voxel_size", down_voxel_size)) {
        ROS_WARN("No down_voxel_size specified. Looking for %s. Default is 'down_voxel_size'.",
                 (ns + "/down_voxel_size").c_str());
    }

    T_B_W = tf::Transform::getIdentity();

    downSizeFilter.setLeafSize(down_voxel_size, down_voxel_size, down_voxel_size);

    odom_pub = nh_.advertise<nav_msgs::Odometry>("odometry_init", 1);
    reg_pub = nh_.advertise<sensor_msgs::PointCloud2>("registered_scan", 1);
    dwz_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("dwz_scan_cloud", 1);

    local_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "point_cloud", 1));
    local_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odometry", 100));
    sync_local_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyLocalCloudOdom>(
            SyncPolicyLocalCloudOdom(100), *local_cloud_sub_, *local_odom_sub_));
    sync_local_cloud_odom_->registerCallback(boost::bind(&SlamOutput::pointCloudOdomCallback, this, _1, _2));
}

void SlamOutput::pointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &scanIn,
                                        const nav_msgs::OdometryConstPtr &input) {

    tf::Quaternion quaternion(input->pose.pose.orientation.x, input->pose.pose.orientation.y,
                              input->pose.pose.orientation.z, input->pose.pose.orientation.w);
    tf::Vector3 vector3(input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z);

    tf::Transform T_W_Bi(quaternion, vector3);

    if (!is_get_first) {
        T_B_W = T_W_Bi.inverse();
        is_get_first = true;
    }

    tf::StampedTransform ST_B_Bi = tf::StampedTransform(T_B_W * T_W_Bi, scanIn->header.stamp, frame_id,
                                                        child_frame_id); 
    broadcaster.sendTransform(ST_B_Bi);

    nav_msgs::Odometry odom_msg;
    odom_msg.child_frame_id = child_frame_id;
    odom_msg.header.frame_id = frame_id;
    odom_msg.header.stamp = scanIn->header.stamp; 
    odom_msg.header.seq = input->header.seq;
    odom_msg.pose.pose.orientation.x = ST_B_Bi.getRotation().getX();
    odom_msg.pose.pose.orientation.y = ST_B_Bi.getRotation().getY();
    odom_msg.pose.pose.orientation.z = ST_B_Bi.getRotation().getZ();
    odom_msg.pose.pose.orientation.w = ST_B_Bi.getRotation().getW();
    odom_msg.pose.pose.position.x = ST_B_Bi.getOrigin().getX();
    odom_msg.pose.pose.position.y = ST_B_Bi.getOrigin().getY();
    odom_msg.pose.pose.position.z = ST_B_Bi.getOrigin().getZ();

    odom_msg.twist = input->twist; 

    odom_pub.publish(odom_msg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*scanIn, *scan);

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_data = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::vector<int> scan_index;
    pcl::removeNaNFromPointCloud(*scan, *scan_data, scan_index);

    downSizeFilter.setInputCloud(scan_data);
    pcl::PointCloud<pcl::PointXYZI> scan_dwz;
    downSizeFilter.filter(scan_dwz);

    sensor_msgs::PointCloud2 scan_dwz_msg;
    pcl::toROSMsg(scan_dwz, scan_dwz_msg);
    scan_dwz_msg.header= scanIn->header;
    dwz_cloud_pub.publish(scan_dwz_msg);// publish downsample point cloud

    tf::StampedTransform T_b_bi = ST_B_Bi;

    Eigen::Matrix4f pose;
    pose << T_b_bi.getBasis()[0][0], T_b_bi.getBasis()[0][1], T_b_bi.getBasis()[0][2], T_b_bi.getOrigin()[0],
            T_b_bi.getBasis()[1][0], T_b_bi.getBasis()[1][1], T_b_bi.getBasis()[1][2], T_b_bi.getOrigin()[1],
            T_b_bi.getBasis()[2][0], T_b_bi.getBasis()[2][1], T_b_bi.getBasis()[2][2], T_b_bi.getOrigin()[2],
            0, 0, 0, 1;

    pcl::PointCloud<pcl::PointXYZI>::Ptr registered_scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    for (auto &point: scan_dwz.points) {
        pcl::PointXYZI reg_point;
        reg_point.x = point.x * pose(0, 0) + point.y * pose(0, 1) + point.z * pose(0, 2) + pose(0, 3);
        reg_point.y = point.x * pose(1, 0) + point.y * pose(1, 1) + point.z * pose(1, 2) + pose(1, 3);
        reg_point.z = point.x * pose(2, 0) + point.y * pose(2, 1) + point.z * pose(2, 2) + pose(2, 3);
        reg_point.intensity = point.intensity;
        registered_scan->points.push_back(reg_point);
    }

    // publish registered point cloud messages
    sensor_msgs::PointCloud2 scan_data_msg;
    pcl::toROSMsg(*registered_scan, scan_data_msg);
    scan_data_msg.header.stamp = scanIn->header.stamp;
    scan_data_msg.header.frame_id = frame_id;
    scan_data_msg.header.seq = scanIn->header.seq;
    reg_pub.publish(scan_data_msg);
}
