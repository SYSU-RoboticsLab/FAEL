//
// Created by hjl on 2021/11/23.
//

#include "control_planner_interface/control_planner_interface.h"


namespace interface {

    ControlPlannerInterface::ControlPlannerInterface(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
                                                     std::shared_ptr<PCIManager> &pci_manager)
            : nh_(nh), nh_private_(nh_private), pci_manager_(pci_manager),
              planner_client_(nh_, "topo_planner", true) {

        odometry_sub_ = nh_.subscribe("odometry",1, &ControlPlannerInterface::odometryCallback, this);

        if (!loadParams()) {
            ROS_ERROR("control planner interface can not load params. Shut down ROS node.");
            ros::shutdown();
        }
        ROS_INFO("control planner interface construct");
    }

    bool ControlPlannerInterface::loadParams() {

        const std::string &ns = ros::this_node::getName();

        frame_id_ = "world";
        if (!ros::param::get(ns + "/frame_id", frame_id_)) {
            ROS_WARN("No frame_id specified. Looking for %s. Default is 'world'.",
                     (ns + "/frame_id").c_str());
        }

        pci_manager_->setFrameId(frame_id_);

        if (!pci_manager_->loadParams(ns)) return false;

        return true;
    }

    void ControlPlannerInterface::odometryCallback(const nav_msgs::OdometryConstPtr &odom) {
        current_pose_ = odom->pose.pose;
        pci_manager_->setCurrentPose(current_pose_);

        pose_is_ready_ = true;
    }

    bool ControlPlannerInterface::init() {
        pose_is_ready_ = false;

        // checking odometry is ready.
        ros::Rate rr(1);
        while (!pose_is_ready_) {
            ROS_WARN("Waiting for odometry.");
            ros::spinOnce();
            rr.sleep();
        }

        if (!pci_manager_->initialize()) return false;

        return true;
    }

    void ControlPlannerInterface::goToWayPose(geometry_msgs::Pose &pose) {
        pci_manager_->goToWaypoint(pose);
    }

    bool ControlPlannerInterface::isGoalReached() {
        return pci_manager_->isGoalReached();
    }

    void ControlPlannerInterface::cancelCurrentGoal() {
        pci_manager_->cancelCurrentGoal();
    }

    void ControlPlannerInterface::executePath(const  std::vector<control_planner_interface::Path> &path_segments) {
        std::vector<geometry_msgs::Pose> modified_path;
        pci_manager_->executePath(path_segments,modified_path);
    }

    bool ControlPlannerInterface::callForPlanner(const int &iteration_id,  std::vector<control_planner_interface::Path> &path_segments) {
        control_planner_interface::ExplorerPlannerGoal goal;
        goal.iteration_id = iteration_id;
        ROS_INFO("call for the path to explore..");
        if (planner_client_.sendGoalAndWait(goal, ros::Duration(60.0), ros::Duration(10.0)) ==
            actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("the path get.");
            path_segments = planner_client_.getResult()->paths;
            return true;
        } else {
            ROS_INFO("planner timeout.");
            planner_client_.cancelAllGoals();
            return false;
        }
    }

    void ControlPlannerInterface::stopMove() {
        pci_manager_->stopMove();
    }
}