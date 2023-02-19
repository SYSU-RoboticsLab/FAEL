//
// Created by hjl on 2022/10/3.
//

#ifndef ROBO_PLANNER_WS_TEST_NEW_TERRAIN_MAP_EMPTY_H
#define ROBO_PLANNER_WS_TEST_NEW_TERRAIN_MAP_EMPTY_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include<laser_geometry/laser_geometry.h>
#include <traversability_analysis/TerrainMap.h>
#include <visualization_msgs/MarkerArray.h>

namespace traversability_analysis {
    using namespace std;

    enum Status {
        Free = 0, Occupied = 1, Empty = 2, Unknown = 3
    };

    struct Grid2D {
        Status status;

        Grid2D() : status(Status::Unknown) {};

        explicit Grid2D(Status sta) : status(sta) {};

        void clear() {
            status = Status::Unknown;
        };
    };

    class TerrainMapEmpty {

    public:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Publisher terrain_map_pub_;
        ros::Publisher terrain_grids_pub_;
        ros::Publisher voxelized_local_points_pub_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyLocalCloudOdom;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> static_point_cloud_sub_;
        std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
        typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>> SynchronizerLocalCloudOdom;
        SynchronizerLocalCloudOdom sync_registered_scan_odom_;

        double scan_voxel_size_;
        double vehicle_height_;
        double lower_bound_z_;
        double upper_bound_z_;
        double dis_ratio_z_; 
        double lower_z_;

        double grid_size_;
        int grid_width_num_;
        int grid_half_width_num_;
        int min_grid_point_num_;

        double min_x_; 
        double min_y_;
        double max_x_;
        double max_y_;
        vector<vector<Grid2D>> terrain_grids_; 


        pcl::PointCloud<pcl::PointXYZ> local_static_point_cloud_;
        pcl::PointCloud<pcl::PointXYZ> voxelized_local_cloud_;//downsampling local point clouds
        vector<vector<pcl::PointCloud<pcl::PointXYZ>>> grids_cloud_;
        std::vector<std::vector<float>> grids_min_elevation_;//minimum z value of each grid
        std::vector<std::vector<pcl::PointXYZ>> grids_bottom_point_;//lowest point of each grid
        pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter_;

        tf::TransformListener tf_listener_;
        geometry_msgs::Pose current_pose_;
        geometry_msgs::Pose init_pose_;
        int no_data_init_;
        int fill_point_num_;

        TerrainMapEmpty(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

        void staticPointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud,
                                          const nav_msgs::OdometryConstPtr &odom);

        void terrainGridsMove();

        void stackPointCloudAndElevations();

        void getMinTerrainGridsElevation();

        void computerTerrainCloudReletiveElevationAndNoDataAreaObstacled();

        visualization_msgs::MarkerArray generateGridMarkers(std::vector<std::vector<Grid2D>> &terrain_grids,geometry_msgs::Pose &current_pose);


        inline bool isInGridMapRange2d(const pcl::PointXYZ &point) const {
            if (point.x < min_x_ + 1e-4 || point.x > max_x_ - 1e-4 ||
                point.y < min_y_ + 1e-4 || point.y > max_y_ - 1e-4) {
                return false;
            } else {
                return true;
            }
        };

        inline Eigen::Vector2i getIndexInGridMap(pcl::PointXYZ &point) {
            Eigen::Vector2i index;
            index.x() = floor((point.x - min_x_) / grid_size_);
            index.y() = floor((point.y - min_y_) / grid_size_);
            return index;
        };

        inline bool isInGridMapRange2d(const Eigen::Vector2d &point) const {
            if (point.x() < min_x_ + 1e-4 || point.x() > max_x_ - 1e-4 ||
                point.y() < min_y_ + 1e-4 || point.y() > max_y_ - 1e-4) {
                return false;
            } else {
                return true;
            }
        };

        inline Eigen::Vector2i getIndexInGridMap(const Eigen::Vector2d &point) {
            Eigen::Vector2i index;
            index.x() = floor((point.x() - min_x_) / grid_size_);
            index.y() = floor((point.y() - min_y_) / grid_size_);
            return index;
        };

        inline Eigen::Vector2d getGridCenter(const int& index_x, const int &index_y){
            Eigen::Vector2d center;
            center.x() = min_x_ + grid_size_ / 2 + index_x * grid_size_;
            center.y() = min_y_ + grid_size_ / 2 + index_y * grid_size_;
            return center;

        };
    };
}



#endif //ROBO_PLANNER_WS_TEST_NEW_TERRAIN_MAP_EMPTY_H
