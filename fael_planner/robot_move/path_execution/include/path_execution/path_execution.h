//
// Created by hjl on 2021/12/15.
//

#ifndef TOPO_PLANNER_WS_PATH_EXECUTION_H
#define TOPO_PLANNER_WS_PATH_EXECUTION_H

#include <Eigen/Eigen>
#include <iostream>
#include <set>
#include <map>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/transforms.h>
#include <visualization_msgs/MarkerArray.h>

#include <actionlib/server/simple_action_server.h>
#include <control_planner_interface/VehicleExecuteAction.h>

#include "path_execution/occupancy_map_2d.h"
#include "traversability_analysis/TerrainMap.h"

namespace path_execution {
    using namespace std;

    typedef std::vector<geometry_msgs::Pose> Path;
    typedef std::vector<Point2D> Path2D;

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

        inline int getGridID(const Point2D &point) {
          return floor((point.x() - min_x) / grid_size) * grid_width_num + floor((point.y() - min_y) / grid_size);
        };
    };


    class PathExecution {

    public:
        typedef actionlib::SimpleActionServer<control_planner_interface::VehicleExecuteAction> JackalExecuteActionServer;

        PathExecution(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

        void odomCallback(const nav_msgs::OdometryConstPtr &odom);

        void terrainMapCallback(const traversability_analysis::TerrainMapConstPtr &terrain_map);

        void setOccupancyMap(OccupancyMap2D &occupancy_map, TerrainMap &terrain_map);

        void clearRobotNeighbor(const Point2D &center_point, double clear_radius = 0.0);

        void executeCallback(const control_planner_interface::VehicleExecuteGoalConstPtr &executed_path);

        void stopMoveCallback(const control_planner_interface::VehicleExecuteGoalConstPtr &stop_goal);

        void goalCallback(const geometry_msgs::PoseStampedConstPtr &goal);

        bool setPathToFollow(const std::vector<control_planner_interface::Path> &path_segments);

        Path interpolatePath(const Path &path) const;

        std::vector<Point2D> makeLocalPlan(geometry_msgs::Pose &goal, geometry_msgs::Pose &start, OccupancyMap2D &map);

        std::vector<Point2D>
        getShortestPath(geometry_msgs::Pose &goal, geometry_msgs::Pose &start, OccupancyMap2D &map);

        std::vector<Point2D> optimalToStraight(std::vector<Point2D> &path, OccupancyMap2D &map);

        Path generatePath(std::vector<Point2D> &path_2d, geometry_msgs::Pose &end_pose);

        bool isStopped();

        void publishPath(const Path &path, const ros::Publisher &pub);

        bool lookAheadPointControlLoop();

        visualization_msgs::MarkerArray generatePath2DListMarker(const std::vector<Path2D> &path_list) const;
        visualization_msgs::Marker generateSmoothPath2DMarker(const Path2D &path) const;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        JackalExecuteActionServer execute_server_;
        JackalExecuteActionServer stop_move_server_;

        ros::Subscriber odom_sub_;
        ros::Subscriber cloud_sub_;
        ros::Subscriber terrain_map_sub_;
        ros::Subscriber goal_pose_sub_;

        ros::Publisher local_path_pub_;
        ros::Publisher way_pose_pub_;
        ros::Publisher stop_move_pub_;
        ros::Publisher occupancy_pub_;
        ros::Publisher inflate_pub_;
        ros::Publisher executed_path_pub_;

        ros::Publisher path_list_pub_;
        ros::Publisher smooth_path_pub_;
        ros::Publisher look_ahead_goal_pub_;

        tf::TransformListener tf_listener_;

        std::string global_frame_;
        std::string local_frame_;
        double cloud_voxel_size_;
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_;
        pcl::PointCloud<pcl::PointXYZI> terrain_cloud_;

        OccupancyMap2D occupancy_map_;
        double grid_size_;
        int grid_x_num_;
        int grid_y_num_;
        double inflate_radius_;
        double inflate_empty_radius_;
        double lower_z_; 

        boost::mutex occupancy_map_mutex_;

        geometry_msgs::Pose current_pose_;
        geometry_msgs::Pose last_pose_;

        nav_msgs::Odometry current_odom_;
        nav_msgs::Odometry last_odom_;

        Path global_path_to_follow_;
        std::vector<Path> path_segments_;
        int current_waypose_id_;
        int loop_num_;

        double control_freq_;
        double guide_dist_;
        double reach_dist_thres_;
        double reach_rot_thres_;
        double stop_vel_thres_;
        double stop_rot_thres_;

        std::vector<Path2D> path2d_list_;

    };

}


#endif //TOPO_PLANNER_WS_PATH_EXECUTION_H
