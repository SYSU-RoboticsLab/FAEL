//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_VIEWPOINT_MANAGER_H
#define ROBO_PLANNER_WS_VIEWPOINT_MANAGER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common.h>

#include <utils/viewpoint.h>
#include <utils/frontier.h>
#include <perception/ufomap.h>
#include <perception/map_2d_manager.h>

namespace preprocess {

    using namespace utils;
    using namespace perception;


    class ViewpointManager {
    public:
        typedef std::shared_ptr<ViewpointManager> Ptr;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher repre_points_pub_;
        ros::Publisher frontiers_viewpoints_pub_;

        Ufomap::Ptr frontier_map_;
        Map2DManager::Ptr map_2d_manager_;

        utils::Point3D current_position_;

        //params
        std::string frame_id_;
        double sample_dist_;
        double frontier_dist_;
        double viewpoint_gain_thre_;

        //statictis
        std::ofstream fout_;
        std::string frontiers_attach_txt_name_;
        double sum_attach_time_;
        int attach_num_;

        ViewpointManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                         const Ufomap::Ptr &frontier_map, const Map2DManager::Ptr &map_2d_manager);

        void getParamsFromRos();

        void setCurrentPosition(const utils::Point3D &current_position);

        void updateViewpoints();

        void frontierAttachInUfomap();

        ViewpointQueue samplePointsInGridMap2D();

        void pubMarkers() const;

        visualization_msgs::MarkerArray generatePointsMarkers(const Point3DSet &sample_points) const;

        visualization_msgs::MarkerArray
        generateViewPointsWithFrontiersMarkers(const ViewpointMap<FrontierQueue> &viewpoints_attached_frontiers_) const;

        //output
        Point3DSet representative_points_;
        FrontierMap<Viewpoint> frontiers_viewpoints_;//store each boundary with the corresponding secondary point
        ViewpointMap<FrontierQueue> viewpoints_attached_frontiers_;//store each representative point and which frontier it represents
        ViewpointSet candidate_viewpoints_;//store those frontier representative points
        ViewpointSet new_viewpoints_;//new viewpoint

    };

}


#endif //ROBO_PLANNER_WS_VIEWPOINT_MANAGER_H
