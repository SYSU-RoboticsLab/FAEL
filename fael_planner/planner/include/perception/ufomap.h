//
// Created by hjl on 2022/1/9.
//

#ifndef ROBO_PLANNER_WS_UFOMAP_H
#define ROBO_PLANNER_WS_UFOMAP_H

#include <geometry_msgs/PolygonStamped.h>

#include "perception/map_3d.h"
#include "perception/lidar_model.h"
#include <ufomap_manager/frontier_manager.h>
#include <ros/package.h>

namespace perception {
    typedef std::unordered_set<ufo::map::Code, ufo::map::Code::Hash> CodeUnorderSet;

    class Ufomap : public Map3D {
    public:
        typedef std::shared_ptr<Ufomap> Ptr;

        Ufomap(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

        void setParametersFromROS();

        utils::FrontierSet getFrontiers() const override {
            FrontierSet frontier_set;
            if(!global_frontier_cells_.empty()){
                for(const auto &cell:global_frontier_cells_){
                    ufo::map::Point3 point = map_.toCoord(cell);
                    frontier_set.emplace(point.x(),point.y(),point.z());
                }
            }
            return frontier_set;
        };

        bool isFrontier(const Frontier &frontier) const override {
            return isFrontier(map_.toCode(frontier.x(),frontier.y(),frontier.z()));
        };

        // map update
        void
        pointCloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &scan, const nav_msgs::OdometryConstPtr &odom);

        void ufomapPublishTimer(const ros::TimerEvent &event);

        void writeUfomapCallback(const ros::TimerEvent &event);

        ufo::map::OccupancyMap const &getUFOMap() const { return map_; };

        void frontierCallback(const ros::TimerEvent &event);

        void frontierUpdate();

        void frontierSearch();

        void findLocalFrontier();

        void updateGlobalFrontier();

        void findPlaneLocalFrontier();

        void updatePlaneGlobalFrontier();

        CodeUnorderSet get_3D_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth);

        CodeUnorderSet get_XY_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth) const;

        CodeUnorderSet get_Z_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth) const;

        bool isFrontier(const ufo::map::Code &frontier) const;

        bool isInExplorationArea(const double &point_x, const double & point_y) const;

        // statistic

        void knownCellOutput();

        std::size_t getKnownNodeNum(const CodeUnorderSet &knownCellCodes);

        std::size_t getKnownPlaneNodeNum(const CodeUnorderSet &knownCellCodes);

        // pub msgs
        void generateMarkerArray(const std::string &tf_frame, visualization_msgs::MarkerArray *frontier_cells,
                                 CodeUnorderSet &frontier_cell_codes, std_msgs::ColorRGBA &rgba);

        void statisticAndPubMarkers();

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyLocalCloudOdom;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> point_cloud_sub_;
        std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
        typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>> SynchronizerLocalCloudOdom;
        SynchronizerLocalCloudOdom sync_point_cloud_odom_;

        ros::Publisher map_pub_;
        ros::Publisher cloud_pub_;
        ros::Publisher expand_cloud_pub_;
        ros::Publisher map_and_frontiers_pub_;
        ros::Publisher local_frontiers_pub_;
        ros::Publisher global_frontiers_pub_;
        ros::Publisher ploygon_pub_;

        ros::Timer pub_timer_;
        ros::Timer frontier_timer_;
        ros::Timer write_ufomap_timer_;

        // TF2
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        ros::Duration transform_timeout_; // "How long to wait for transform (s)",

        // map
        ufo::map::OccupancyMap map_;
        std::string frame_id_;

        // position
        ufo::math::Pose6 current_robot_pose_;  
        ufo::math::Pose6 current_sensor_pose_;  

        // param
        std::string sensor_frame_id_;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;  
        float scan_voxel_size_;  

        LidarModel lidar_;

        // integration
        double max_range_;// "Max range (m) when integrating data into map",
        bool insert_discrete_;// "Enable discrete integration",
        int insert_depth_;
        bool simple_ray_casting_;   
        int early_stopping_;

        // clear robot
        bool clear_robot_enabled_;  // "Clear map at robot position",
        std::string robot_base_frame_id_;
        double robot_height_;   // "Robot height (m)",
        double robot_bottom_;    // "Robot horizontal line (m)",
        double sensor_height_;  // "sensor to bottom (m)",
        int clearing_depth_;

        double pub_rate_;    // "How often to publish map (hz)",

        // frontiers
        int frontier_depth_;
        CodeUnorderSet global_frontier_cells_;
        CodeUnorderSet local_frontier_cells_;
        CodeUnorderSet history_frontier_cells_;

        float width_;
        float length_;
        float min_x_;
        float min_y_;
        float max_x_;
        float max_y_;


        // use code to index frontier points
        CodeUnorderSet changed_cell_codes_;  
        CodeUnorderSet known_cell_codes_;  
        std::size_t known_cell_num_;
        std::size_t known_plane_cell_num_;

        std::mutex map_mutex_;
        std::mutex odom_mutex_;

    protected:
        //statistic
        std::string map_txt_name;

        std::ofstream fout;
        std::string txt_known_name;
        double start_time;

        std::string txt_frontier_name;
        double sum_frontier_time;
        int frontier_iteration;

        std::string txt_insert_cloud_name;
        double sum_insert_time;
        int insert_num;
    };


}


#endif //ROBO_PLANNER_WS_UFOMAP_H
