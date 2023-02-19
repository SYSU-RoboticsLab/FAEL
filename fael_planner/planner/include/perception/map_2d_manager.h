//
// Created by hjl on 2022/1/11.
//

#ifndef ROBO_PLANNER_WS_MAP_2D_MANAGER_H
#define ROBO_PLANNER_WS_MAP_2D_MANAGER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common.h>

#include "grid_map_2d.h"
#include <traversability_analysis/TerrainMap.h>

#include "perception/ufomap.h"

namespace perception{

    struct TerrainMap{
        std::string frame_id;
        double min_x;
        double min_y;
        double max_x;
        double max_y;
        double z_value;
        double grid_size;
        int grid_width_num;
        std::vector<unsigned int> status;
        pcl::PointCloud<pcl::PointXYZI> bottom_points;  

        void clear(){
            status.clear();
        };

        inline bool isInTerrainMap(const Point2D &point) const {
            if (point.x() < min_x + 1e-4 || point.x() > max_x - 1e-4 ||
                point.y() < min_y + 1e-4 || point.y() > max_y - 1e-4) {
                return false;
            } else {
                return true;
            }
        };

        inline int getGridID(const Point2D &point) const {
            return floor((point.x() - min_x) / grid_size) * grid_width_num + floor((point.y() - min_y) / grid_size);
        };
    };

    class Map2DManager {
    public:
        typedef std::shared_ptr<Map2DManager> Ptr;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber odom_sub_;
        ros::Subscriber terrain_map_sub_;

        ros::Publisher grid_map_2d_pub_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicyLocalCloud;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> terrain_cloud_sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> local_cloud_sub_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloud>> sync_terrain_local_cloud_;
        pcl::PointCloud<pcl::PointXYZI> terrain_cloud_;
        pcl::PointCloud<pcl::PointXYZI> local_cloud_;

        tf::TransformListener tf_listener_;


        GridMap2D inflate_map_;
        GridMap2D map_;  
        geometry_msgs::Pose current_pose_;
        bool is_map_updated_;

        std::string frame_id_;
        double grid_size_;
        double inflate_radius_;
        double inflate_empty_radius_;
        double lower_z_;  
        double connectivity_thre_;

        std::mutex map_2d_update_mutex_;

        Map2DManager(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

        void getParamsFromRos();

        void odomCallback(const nav_msgs::OdometryConstPtr &odom);

        void terrainLocalCloudCallback(const sensor_msgs::PointCloud2ConstPtr &terrain_cloud,
                                       const sensor_msgs::PointCloud2ConstPtr &local_cloud);

        void terrainMapCallback(const traversability_analysis::TerrainMapConstPtr &terrain_map);

        void updateGridMap2D(const pcl::PointCloud<pcl::PointXYZI> &terrain_cloud,
                             const pcl::PointCloud<pcl::PointXYZI> &local_cloud);

        void updateGridMap2D(const TerrainMap &terrain_map);

    };

}



#endif //ROBO_PLANNER_WS_MAP_2D_MANAGER_H
