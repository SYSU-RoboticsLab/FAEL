//
// Created by hjl on 2021/11/23.
//

#ifndef TOPO_PLANNER_WS_CONTROL_PLANNER_INTERFACE_H
#define TOPO_PLANNER_WS_CONTROL_PLANNER_INTERFACE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_srvs/Empty.h>
#include <chrono>
#include <thread>

#include "control_planner_interface/pci_manager.h"

#include <actionlib/client/simple_action_client.h>
#include "control_planner_interface/ExplorerPlannerAction.h"

namespace interface {

    class ControlPlannerInterface {

    public:

        enum struct RobotMotionState {
            waiting = 0, executing = 1, stoping = 2
        };

        typedef actionlib::SimpleActionClient<control_planner_interface::ExplorerPlannerAction> ExplorationPlannerClient;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        std::shared_ptr<PCIManager> pci_manager_;

        ros::Subscriber odometry_sub_;

        ExplorationPlannerClient planner_client_;

        std::string frame_id_;
        bool pose_is_ready_;

        double init_x_;
        double init_y_;
        double init_z_;

        geometry_msgs::Pose current_pose_;

        ControlPlannerInterface(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
                                std::shared_ptr<PCIManager> &pci_manager);

        bool loadParams();

        bool init();

        void odometryCallback(const nav_msgs::OdometryConstPtr &odom);

        bool callForPlanner(const int &iteration_id, std::vector<control_planner_interface::Path> &path_segments);

        void executePath(const std::vector<control_planner_interface::Path> &path_segments);

        void goToWayPose(geometry_msgs::Pose &pose);

        bool isGoalReached();

        void cancelCurrentGoal();

        void stopMove();

    };
}


#endif //TOPO_PLANNER_WS_CONTROL_PLANNER_INTERFACE_H
