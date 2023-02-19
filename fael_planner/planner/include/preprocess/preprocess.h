//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_PREPROCESS_H
#define ROBO_PLANNER_WS_PREPROCESS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "perception/map_3d.h"
#include "perception/ufomap.h"
#include "perception/map_2d_manager.h"
#include "preprocess/viewpoint_manager.h"
#include "preprocess/topo_graph.h"

namespace preprocess {

    class Preprocess {
    public:
        typedef std::shared_ptr<Preprocess> Ptr;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        //params
        std::string frame_id_;
        std::string robot_base_frame_id_;

        //odometry
        ros::Subscriber odom_sub_;
        utils::Point3D current_position_;
        geometry_msgs::Pose current_pose_;

        utils::Point3D last_directory_position_;
        utils::Point3D forward_directory_;

        ros::Subscriber explorer_init_sub_;
        bool is_explorer_initilized_;

        //dynamic updating
        perception::Ufomap::Ptr frontier_map_;
        perception::Map2DManager::Ptr map_2d_manager_;
        preprocess::ViewpointManager::Ptr viewpoint_manager_;
        preprocess::TopoGraph::Ptr road_map_;

        //mutex
        std::mutex elements_update_mutex_;

        Preprocess(ros::NodeHandle &nh, ros::NodeHandle &nh_private) :
                nh_(nh), nh_private_(nh_private), is_explorer_initilized_(false) {
            getParamsFromRos();
            odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odometry", 1, &Preprocess::odomCallback, this);
            explorer_init_sub_ = nh_.subscribe<std_msgs::Float64>("explorer_inited", 1, &Preprocess::explorerInitCallback, this);

            init(nh, nh_private);
        };

        void getParamsFromRos() {
            std::string ns = ros::this_node::getName();
            frame_id_ = "world";
            if (!ros::param::get(ns + "/frame_id", frame_id_)) {
                ROS_WARN("No frame_id specified. Looking for %s. Default is 'map'.",
                         (ns + "/frame_id").c_str());
            }
            robot_base_frame_id_ = "base_link";
            if (!ros::param::get(ns + "/robot_base_frame_id", robot_base_frame_id_)) {
                ROS_WARN("No robot_base_frame_id specified. Looking for %s. Default is 'base_link'.",
                         (ns + "/robot_base_frame_id").c_str());
            }
        };

        void odomCallback(const nav_msgs::OdometryConstPtr &odom) {
            current_pose_ = odom->pose.pose;
            current_position_.x() = odom->pose.pose.position.x;
            current_position_.y() = odom->pose.pose.position.y;
            current_position_.z() = odom->pose.pose.position.z;

            viewpoint_manager_->setCurrentPosition(current_position_);
            road_map_->setCurrentPosition(current_position_);

            if(current_position_.distanceXY(last_directory_position_)>0.5){
                forward_directory_ = current_position_-last_directory_position_;
                last_directory_position_ = current_position_;
            }

        };

        void explorerInitCallback(const std_msgs::Float64ConstPtr &msg) {
            is_explorer_initilized_ = true;
        };

        void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private) {
            frontier_map_ = std::make_shared<perception::Ufomap>(nh, nh_private);
            map_2d_manager_ = std::make_shared<perception::Map2DManager>(nh, nh_private);
            viewpoint_manager_ = std::make_shared<preprocess::ViewpointManager>(nh, nh_private, frontier_map_,map_2d_manager_);
            road_map_ = std::make_shared<preprocess::TopoGraph>(nh, nh_private);
        };

        void updateElements(){
            if(is_explorer_initilized_ ){
                viewpoint_manager_->updateViewpoints();
                road_map_->updateTopoGraphByMap2DAndViewpoints(map_2d_manager_,viewpoint_manager_,frontier_map_);
            }
        };


    };

}

#endif //ROBO_PLANNER_WS_PREPROCESS_H
