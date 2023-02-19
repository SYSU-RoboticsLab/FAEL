//
// Created by hjl on 2022/1/11.
//

#include "perception/map_2d_manager.h"

namespace perception {

    Map2DManager::Map2DManager(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
            : nh_(nh), nh_private_(nh_private), is_map_updated_(false) {
        getParamsFromRos();
        grid_map_2d_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("gird_map_2d", 1);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("base_odometry", 1, &Map2DManager::odomCallback, this);

        terrain_map_sub_ = nh_.subscribe<traversability_analysis::TerrainMap>("terrain_map", 1,
                                                                              &Map2DManager::terrainMapCallback, this);

        terrain_cloud_sub_.reset(
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "terrain_point_cloud", 1));
        local_cloud_sub_.reset(
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "voxelized_local_point_cloud", 1));
        sync_terrain_local_cloud_.reset(
                new message_filters::Synchronizer<SyncPolicyLocalCloud>(SyncPolicyLocalCloud(1), *terrain_cloud_sub_,
                                                                        *local_cloud_sub_));
        sync_terrain_local_cloud_->registerCallback(
                boost::bind(&Map2DManager::terrainLocalCloudCallback, this, _1, _2));

    }

    void Map2DManager::getParamsFromRos() {
        std::string ns = ros::this_node::getName() + "/GridMap2D";

        frame_id_ = "world";
        if (!ros::param::get(ns + "/frame_id", frame_id_)) {
            ROS_WARN("No frame_id specified. Looking for %s. Default is 'map'.",
                     (ns + "/frame_id").c_str());
        }

        grid_size_ = 0.3;
        if (!ros::param::get(ns + "/grid_size", grid_size_)) {
            ROS_WARN("No grid_size specified. Looking for %s. Default is 0.3.",
                     (ns + "/grid_size").c_str());
        }

        inflate_radius_ = 0.3;
        if (!ros::param::get(ns + "/inflate_radius", inflate_radius_)) {
            ROS_WARN("No inflate_radius specified. Looking for %s. Default is 0.3 .",
                     (ns + "/inflate_radius").c_str());
        }

        inflate_empty_radius_ = 0.6;
        if (!ros::param::get(ns + "/inflate_empty_radius", inflate_empty_radius_)) {
            ROS_WARN("No inflate_empty_radius specified. Looking for %s. Default is 0.6 .",
                     (ns + "/inflate_empty_radius").c_str());
        }

        lower_z_ = 0.03;
        if (!ros::param::get(ns + "/lower_z", lower_z_)) {
            ROS_WARN("No lower_z specified. Looking for %s. Default is 0.03 .",
                     (ns + "/lower_z").c_str());
        }

        connectivity_thre_ = 0.1;
        if (!ros::param::get(ns + "/connectivity_thre", connectivity_thre_)) {
            ROS_WARN("No connectivity_thre specified. Looking for %s. Default is 0.1 .",
                     (ns + "/connectivity_thre").c_str());
        }

    }

    void Map2DManager::odomCallback(const nav_msgs::OdometryConstPtr &odom) {
        current_pose_ = odom->pose.pose;
    }

    void Map2DManager::terrainLocalCloudCallback(const sensor_msgs::PointCloud2ConstPtr &terrain_cloud,
                                                 const sensor_msgs::PointCloud2ConstPtr &local_cloud) {
        pcl::fromROSMsg(*terrain_cloud, terrain_cloud_);
        pcl::fromROSMsg(*local_cloud, local_cloud_);

        if (terrain_cloud->header.frame_id != frame_id_) {
            if (!pcl_ros::transformPointCloud(frame_id_, terrain_cloud_, terrain_cloud_, tf_listener_))//Convert to global frame.
                return;
        }

        if (local_cloud->header.frame_id != frame_id_) {
            if (!pcl_ros::transformPointCloud(frame_id_, local_cloud_, local_cloud_, tf_listener_))//Convert to global frame.
                return;
        }

        updateGridMap2D(terrain_cloud_, local_cloud_);//Use terrain point cloud to update 2D grid status, and alloc grid to local point cloud
        grid_map_2d_pub_.publish(inflate_map_.generateMapMarkers(inflate_map_.grids_, current_pose_));
    }

    void Map2DManager::terrainMapCallback(const traversability_analysis::TerrainMapConstPtr &terrain_map) {
        
        TerrainMap terrain_map_;

        pcl::PointXYZI bottom_point;
        pcl_conversions::toPCL(terrain_map->header, terrain_map_.bottom_points.header);
        for (const auto &item :terrain_map->grids) {
            terrain_map_.status.push_back(item.status);
            bottom_point.x = item.bottom_point.x;
            bottom_point.y = item.bottom_point.y;
            bottom_point.z = item.bottom_point.z;
            terrain_map_.bottom_points.push_back(bottom_point);
        }

        if (terrain_map->header.frame_id == frame_id_) { 
            terrain_map_.frame_id = terrain_map->header.frame_id;
            terrain_map_.min_x = terrain_map->min_x;
            terrain_map_.min_y = terrain_map->min_y;
            terrain_map_.z_value = terrain_map->z_value;
        } else {
            geometry_msgs::PoseStamped min_pose;
            min_pose.header = terrain_map->header;
            min_pose.pose.position.x = terrain_map->min_x;
            min_pose.pose.position.y = terrain_map->min_y;
            min_pose.pose.position.z = terrain_map->z_value;
            min_pose.pose.orientation.w = 1.0;
            try {
                tf_listener_.transformPose(frame_id_, min_pose, min_pose);
            }
            catch (const tf::TransformException &ex) {
                ROS_WARN_THROTTLE(1, " get terrain map acquire---    %s ", ex.what());
                return;
            }
            terrain_map_.frame_id = frame_id_;
            terrain_map_.min_x = min_pose.pose.position.x;
            terrain_map_.min_y = min_pose.pose.position.y;
            terrain_map_.z_value = min_pose.pose.position.z;

            if (!pcl_ros::transformPointCloud(frame_id_, terrain_map_.bottom_points, terrain_map_.bottom_points,
                                              tf_listener_))//Convert to global frame
                return;
        }

        terrain_map_.grid_size = terrain_map->grid_size;
        terrain_map_.grid_width_num = terrain_map->grid_width_num;
        terrain_map_.max_x = terrain_map_.min_x + terrain_map_.grid_size * terrain_map_.grid_width_num;
        terrain_map_.max_y = terrain_map_.min_y + terrain_map_.grid_size * terrain_map_.grid_width_num;


        updateGridMap2D(terrain_map_);
        grid_map_2d_pub_.publish(inflate_map_.generateMapMarkers(inflate_map_.grids_, current_pose_));

    }

    void Map2DManager::updateGridMap2D(const pcl::PointCloud<pcl::PointXYZI> &terrain_cloud,
                                       const pcl::PointCloud<pcl::PointXYZI> &local_cloud) {
        map_2d_update_mutex_.lock();
        map_.clearMap();
        inflate_map_.clearMap();
        pcl::PointXYZI min_p;
        pcl::PointXYZI max_p;
        pcl::getMinMax3D(terrain_cloud, min_p, max_p);
        double x_length = max_p.x - min_p.x;
        double y_length = max_p.y - min_p.y;
        map_.initialize(grid_size_, x_length, y_length, Status2D::Unknown);

        Eigen::Vector2d center_point(current_pose_.position.x, current_pose_.position.y);//center point relative to the global frame
        map_.setMapCenterAndBoundary(center_point);

        for (const auto &point: terrain_cloud) {//relative to the base link plane
            if (map_.isInMapRange2D(point)) {
                if (point.z < -0.1) continue;
                if (point.z - current_pose_.position.z > lower_z_) {//z is the relative height
                    if (point.intensity == 1) {
                        map_.setOccupied(map_.getIndexInMap2D(point));
                    }
                    if (point.intensity == 2) {
                        map_.setEmpty(map_.getIndexInMap2D(point));
                    }
                } else {
                    map_.setFree(map_.getIndexInMap2D(point));
                }
            }
        }

        for (auto &point: local_cloud) { 
            if (map_.isInMapRange2D(point))
                map_.addPointInGrid(map_.getIndexInMap2D(point), point);
        }

        inflate_map_ = map_;
        inflate_map_.inflateGridMap2D(inflate_radius_, inflate_empty_radius_);//Use inflated grid

        is_map_updated_ = true;

        map_2d_update_mutex_.unlock();
    }

    void Map2DManager::updateGridMap2D(const TerrainMap &terrain_map) {
        map_2d_update_mutex_.lock();
        map_.clearMap();
        inflate_map_.clearMap();
        double x_length = terrain_map.max_x - terrain_map.min_x;
        double y_length = terrain_map.max_y - terrain_map.min_y;
        map_.initialize(grid_size_, x_length, y_length, Status2D::Unknown);

        Eigen::Vector2d center_point(current_pose_.position.x, current_pose_.position.y);
        map_.setMapCenterAndBoundary(center_point);

        Point2D grid_center;
        for (int i = 0; i < map_.grid_x_num_; ++i) {
            for (int j = 0; j < map_.grid_y_num_; ++j) {
                grid_center = map_.getGridCenter(i, j);
                if (terrain_map.isInTerrainMap(grid_center)) {
                    if (terrain_map.status[terrain_map.getGridID(grid_center)] == 0)
                        map_.setFree(i, j);
                    if (terrain_map.status[terrain_map.getGridID(grid_center)] == 1)
                        map_.setOccupied(i, j);
                    if (terrain_map.status[terrain_map.getGridID(grid_center)] == 2)
                        map_.setEmpty(i, j);
                    if (terrain_map.status[terrain_map.getGridID(grid_center)] == 3)
                        map_.setUnknown(i, j);

                    map_.addPointInGrid(i, j, terrain_map.bottom_points[terrain_map.getGridID(grid_center)]);

                } else {
                    map_.setUnknown(i, j);
                }
            }
        }

        inflate_map_ = map_;
        inflate_map_.inflateGridMap2D(inflate_radius_, inflate_empty_radius_);//Use inflated grid

        is_map_updated_ = true;

        map_2d_update_mutex_.unlock();
    }

}

