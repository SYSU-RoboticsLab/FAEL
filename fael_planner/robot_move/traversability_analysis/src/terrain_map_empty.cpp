//
// Created by hjl on 2022/10/3.
//

#include "traversability_analysis/terrain_map_empty.h"

namespace traversability_analysis {

    TerrainMapEmpty::TerrainMapEmpty(ros::NodeHandle &nh, ros::NodeHandle &nh_private) :
            nh_(nh), nh_private_(nh_private), no_data_init_(0) {

        static_point_cloud_sub_.reset(
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "static_point_cloud", 1));
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odometry", 100));
        sync_registered_scan_odom_.reset(new message_filters::Synchronizer<SyncPolicyLocalCloudOdom>(
                SyncPolicyLocalCloudOdom(100), *static_point_cloud_sub_, *odom_sub_));
        sync_registered_scan_odom_->registerCallback(
                boost::bind(&TerrainMapEmpty::staticPointCloudOdomCallback, this, _1, _2));

        terrain_map_pub_ = nh_private_.advertise<traversability_analysis::TerrainMap>("terrain_map", 1);
        terrain_grids_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("terrain_grids", 1);
        voxelized_local_points_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("voxelized_local_point_cloud", 1);

        nh_private_.param("scan_voxel_size", scan_voxel_size_, 0.05);
        nh_private_.param("vehicle_height", vehicle_height_, 0.5);
        nh_private_.param("lower_bound_z", lower_bound_z_, -0.5);
        nh_private_.param("upper_bound_z", upper_bound_z_, 0.0);
        nh_private_.param("dis_ratio_z", dis_ratio_z_, 0.17);
        nh_private_.param("grid_size", grid_size_, 0.3);
        nh_private_.param("grid_width", grid_width_num_, 30);
        nh_private_.param("lower_z", lower_z_, 0.08);

        grid_half_width_num_ = grid_width_num_ / 2;
        fill_point_num_ = static_cast<int>(grid_size_ / scan_voxel_size_);

        nh_private_.param("min_grid_point_num", min_grid_point_num_, 10);

        grids_cloud_ = vector<vector<pcl::PointCloud<pcl::PointXYZ>>>(grid_width_num_,
                                                                      vector<pcl::PointCloud<pcl::PointXYZ>>(
                                                                              grid_width_num_));

        grids_min_elevation_ = vector<vector<float>>(grid_width_num_, vector<float>(grid_width_num_, 0));

        downSizeFilter_.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);

        Grid2D grid;
        terrain_grids_ = vector<vector<Grid2D>>(grid_width_num_, vector<Grid2D>(grid_width_num_, grid));

        pcl::PointXYZ point;
        grids_bottom_point_ = vector<vector<pcl::PointXYZ>>(grid_width_num_,
                                                            vector<pcl::PointXYZ>(grid_width_num_, point));
        ROS_INFO("terrain analysis construct finish. ");

    }

    void TerrainMapEmpty::staticPointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud,
                                                       const nav_msgs::OdometryConstPtr &odom) {
        current_pose_ = odom->pose.pose;

        if (no_data_init_ == 0) {
            init_pose_ = current_pose_;
            no_data_init_ = 1;
        }
        if (no_data_init_ == 1) {
            double dis = sqrt(std::pow((current_pose_.position.x - init_pose_.position.x), 2) +
                              std::pow((current_pose_.position.y - init_pose_.position.y), 2));
            if (dis > 2 * vehicle_height_ / (tan(15 * M_PI / 180)))
                no_data_init_ = 2; 
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr static_point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*point_cloud, *static_point_cloud);

        local_static_point_cloud_.clear();
        downSizeFilter_.setInputCloud(static_point_cloud);
        downSizeFilter_.filter(local_static_point_cloud_);

        voxelized_local_cloud_.clear();
        for(const auto & point:local_static_point_cloud_){
            double dis = sqrt(std::pow((point.x - current_pose_.position.x), 2) +
                              std::pow((point.y - current_pose_.position.y), 2));
            if (point.z - current_pose_.position.z > lower_bound_z_ - dis_ratio_z_ * dis &&
                point.z - current_pose_.position.z < upper_bound_z_ + dis_ratio_z_ * dis &&
                point.z - current_pose_.position.z > -2 && point.z-current_pose_.position.z < 1 && 
                isInGridMapRange2d(point)) { 
                voxelized_local_cloud_.push_back(point);
            }
        }

        auto start = ros::Time::now();

        terrainGridsMove();
        stackPointCloudAndElevations();
        computerTerrainCloudReletiveElevationAndNoDataAreaObstacled();
        auto end = ros::Time::now();

        TerrainMap terrain_map_msg;
        terrain_map_msg.header = point_cloud->header;
        terrain_map_msg.min_x = min_x_;
        terrain_map_msg.min_y = min_y_;
        terrain_map_msg.z_value = current_pose_.position.z;
        terrain_map_msg.grid_size = grid_size_;
        terrain_map_msg.grid_width_num = grid_width_num_;

        TerrainGrid grid;
        sensor_msgs::PointCloud2 points;
        for (int i = 0; i < grid_width_num_; ++i) {
            for (int j = 0; j < grid_width_num_; ++j) {
                grid.status = terrain_grids_[i][j].status;
                grid.bottom_point.x = grids_bottom_point_[i][j].x;
                grid.bottom_point.y = grids_bottom_point_[i][j].y;
                grid.bottom_point.z = grids_bottom_point_[i][j].z;

                terrain_map_msg.grids.push_back(grid);
            }
        }
        terrain_map_pub_.publish(terrain_map_msg);
        auto pub_end = ros::Time::now();

        sensor_msgs::PointCloud2 voxelized_local_cloud;
        pcl::toROSMsg(voxelized_local_cloud_, voxelized_local_cloud);
        voxelized_local_cloud.header = point_cloud->header;
        voxelized_local_points_pub_.publish(voxelized_local_cloud);

        visualization_msgs::MarkerArray terrain_grids_markers = generateGridMarkers(terrain_grids_,current_pose_);
        terrain_grids_pub_.publish(terrain_grids_markers);
    }

    void TerrainMapEmpty::terrainGridsMove() {
        Eigen::Vector2d grid_map_center(current_pose_.position.x, current_pose_.position.y);

        min_x_ = grid_map_center.x() - grid_size_ * grid_half_width_num_;
        max_x_ = grid_map_center.x() + grid_size_ * grid_half_width_num_;
        min_y_ = grid_map_center.y() - grid_size_ * grid_half_width_num_;
        max_y_ = grid_map_center.y() + grid_size_ * grid_half_width_num_;

    }

    void TerrainMapEmpty::stackPointCloudAndElevations() {
        for (int i = 0; i < grid_width_num_; ++i) {
            for (int j = 0; j < grid_width_num_; ++j) {
                grids_cloud_[i][j].clear();
                grids_bottom_point_[i][j] = pcl::PointXYZ();
                grids_min_elevation_[i][j] = 10000.0;
                terrain_grids_[i][j].clear();
            }
        }

        Eigen::Vector2i index;
        for (auto &point: voxelized_local_cloud_.points) {
            index = getIndexInGridMap(point);
            grids_cloud_[index.x()][index.y()].push_back(point);
            if (point.z < grids_min_elevation_[index.x()][index.y()]) {
                grids_min_elevation_[index.x()][index.y()] = point.z;
                grids_bottom_point_[index.x()][index.y()] = point;
            }
        }
    }

    void TerrainMapEmpty::computerTerrainCloudReletiveElevationAndNoDataAreaObstacled() {
        if (no_data_init_ == 1) {
            for (int i = 0; i < grid_width_num_; ++i) {
                for (int j = 0; j < grid_width_num_; ++j) {
                    double grid_center_x = grid_size_ * i + min_x_ + grid_size_ / 2;
                    double grid_center_y = grid_size_ * j + min_y_ + grid_size_ / 2;
                    double dis = sqrt(std::pow((grid_center_x - init_pose_.position.x), 2) +
                                      std::pow((grid_center_y - init_pose_.position.y), 2));
                    if (dis < 2 * vehicle_height_ / (tan(15 * M_PI / 180))) {
                        if (grids_cloud_[i][j].size() < min_grid_point_num_) {  
                            terrain_grids_[i][j].status = Status::Free;
                        } else {
                            bool occupied = false;
                            for (auto point: grids_cloud_[i][j].points) {
                                float dis_z = fabs(point.z - grids_min_elevation_[i][j]);
                                if (dis_z > lower_z_) { 
                                    occupied = true;
                                    break;
                                }
                            }
                            if (occupied) { 
                                terrain_grids_[i][j].status = Status::Occupied;
                            } else {
                                terrain_grids_[i][j].status = Status::Free;
                            }
                        }
                    } else if (grids_cloud_[i][j].size() < min_grid_point_num_) {
                        terrain_grids_[i][j].status = Status::Occupied;
                    } else {
                        bool occupied = false;
                        for (auto point: grids_cloud_[i][j].points) {
                            float dis_z = fabs(point.z - grids_min_elevation_[i][j]);
                            if (dis_z > lower_z_) { 
                                occupied = true;
                                break;
                            }
                        }
                        if (occupied) { 
                            terrain_grids_[i][j].status = Status::Empty;
                        } else {
                            terrain_grids_[i][j].status = Status::Free;
                        }
                    }
                }
            }
        }
        if (no_data_init_ == 2) { 
            for (int i = 0; i < grid_width_num_; ++i) {
                for (int j = 0; j < grid_width_num_; ++j) {
                    if (grids_cloud_[i][j].size() < min_grid_point_num_) {                       
                        terrain_grids_[i][j].status = Status::Empty;
                    } else {
                        bool occupied = false;
                        for (auto point: grids_cloud_[i][j].points) {
                            float dis_z = fabs(point.z - grids_min_elevation_[i][j]);
                            if (dis_z > lower_z_) { 
                                occupied = true;
                                break;
                            }
                        }
                        if (occupied) { 
                            terrain_grids_[i][j].status = Status::Occupied;
                        } else {
                            terrain_grids_[i][j].status = Status::Free;
                        }
                    }
                }
            }
        }

    }

    visualization_msgs::MarkerArray TerrainMapEmpty::generateGridMarkers(
            std::vector<std::vector<Grid2D>> &terrain_grids, geometry_msgs::Pose &current_pose) {

        visualization_msgs::Marker free;
        visualization_msgs::Marker occupied;
        visualization_msgs::Marker empty;
        visualization_msgs::Marker unknow;
        visualization_msgs::Marker center_point;

        empty.header.frame_id = occupied.header.frame_id = free.header.frame_id = unknow.header.frame_id = center_point.header.frame_id = "world";
        empty.header.stamp = occupied.header.stamp = free.header.stamp = unknow.header.stamp = center_point.header.stamp = ros::Time::now();
        occupied.ns = "occupied";
        free.ns = "free";
        empty.ns = "empty";
        unknow.ns = "unknown";
        center_point.ns = "center";
        empty.action = occupied.action = free.action = unknow.action = center_point.action = visualization_msgs::Marker::ADD;
        empty.pose.orientation.w = occupied.pose.orientation.w = free.pose.orientation.w = unknow.pose.orientation.w = center_point.pose.orientation.w = 1.0;

        free.id = 0;
        free.type = visualization_msgs::Marker::CUBE_LIST;

        occupied.id = 1;
        occupied.type = visualization_msgs::Marker::CUBE_LIST;

        empty.id = 2;
        empty.type = visualization_msgs::Marker::CUBE_LIST;

        unknow.id = 3;
        unknow.type = visualization_msgs::Marker::CUBE_LIST;

        center_point.id = 4;
        center_point.type = visualization_msgs::Marker::SPHERE_LIST;

        //setting scale
        free.scale.x = grid_size_;
        free.scale.y = grid_size_;
        free.scale.z = 0.01;

        //assigning colors
        free.color.r = 0.5f;
        free.color.g = 1.0f;
        free.color.b = 0.5f;
        free.color.a = 0.5f;

        occupied.scale.x = grid_size_;
        occupied.scale.y = grid_size_;
        occupied.scale.z = 0.01;

        occupied.color.r = 0.1f;
        occupied.color.g = 0.5f;
        occupied.color.b = 0.5f;
        occupied.color.a = 1.0f;

        empty.scale.x = grid_size_;
        empty.scale.y = grid_size_;
        empty.scale.z = 0.01;

        empty.color.r = 0.5f;
        empty.color.g = 0.5f;
        empty.color.b = 0.5f;
        empty.color.a = 0.75f;

        unknow.scale.x = grid_size_;
        unknow.scale.y = grid_size_;
        unknow.scale.z = 0.01;

        unknow.color.r = 1.0f;
        unknow.color.g = 0.1f;
        unknow.color.b = 0.1f;
        unknow.color.a = 0.5f;

        center_point.scale.x = grid_size_;
        center_point.scale.y = grid_size_;
        center_point.scale.z = grid_size_;
        center_point.color.r = 1.0f;
        center_point.color.a = 1.0f;

        geometry_msgs::Point point;
        point.x = current_pose.position.x;
        point.y = current_pose.position.y;
        point.z = current_pose.position.z;
        center_point.points.push_back(point);

        for (int i = 0; i < grid_width_num_; i++) {
            for (int j = 0; j < grid_width_num_; j++) {
                point.x = min_x_ + grid_size_ / 2 + i * grid_size_;
                point.y = min_y_ + grid_size_ / 2 + j * grid_size_;
                point.z = current_pose.position.z;
                if (terrain_grids[i][j].status == Status::Free) {
                    free.points.push_back(point);
                }
                if (terrain_grids[i][j].status == Status::Occupied) {
                    occupied.points.push_back(point);
                }
                if (terrain_grids[i][j].status == Status::Empty) {
                    empty.points.push_back(point);
                }
                if (terrain_grids[i][j].status == Status::Unknown) {
                    unknow.points.push_back(point);
                }
            }
        }


        visualization_msgs::MarkerArray grid_map_markers;
        grid_map_markers.markers.resize(5);
        grid_map_markers.markers[0] = free;
        grid_map_markers.markers[1] = occupied;
        grid_map_markers.markers[2] = empty;
        grid_map_markers.markers[3] = unknow;
        grid_map_markers.markers[4] = center_point;

        return grid_map_markers;
    }


}