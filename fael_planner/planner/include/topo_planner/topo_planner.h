//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_TOPO_PLANNER_H
#define ROBO_PLANNER_WS_TOPO_PLANNER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <control_planner_interface/ExplorerPlannerAction.h>
#include <control_planner_interface/PlannerMsgs.h>

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

#include "preprocess/preprocess.h"
#include "rapid_cover_planner/rapid_cover_planner.h"

namespace topo_planner{

    class TopoPlanner{
    public:
        typedef actionlib::SimpleActionServer<control_planner_interface::ExplorerPlannerAction> ExplorerPlannerServer;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber explore_finish_sub_;

        ros::Publisher iteration_time_pub_;

        ExplorerPlannerServer planner_action_server_;
        int planner_goal_id_;

        ros::Timer preprocess_timer_;
        ros::Timer msgs_timer_;
        bool preprocess_inited_;

        ros::Publisher topo_planner_msgs_pub_;

        preprocess::Preprocess::Ptr elements_;

        rapid_cover_planner::RapidCoverPlanner::Ptr planner_;

        TopoPlanner(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

        void plannerCallback(const control_planner_interface::ExplorerPlannerGoalConstPtr &goal);

        std::vector<geometry_msgs::Pose> wayPoseGeneration(rapid_cover_planner::Path &path);

        bool isExplorationFinish();

        bool isCurrentGoalScanned();

        void topo_planner_msgs_publish();

        void explorationFinishCallback(const std_msgs::BoolConstPtr &finish);

    };
}

#endif //ROBO_PLANNER_WS_TOPO_PLANNER_H
