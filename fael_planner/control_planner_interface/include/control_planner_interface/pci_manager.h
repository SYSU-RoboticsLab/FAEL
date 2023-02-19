//
// Created by hjl on 2021/11/30.
//

#ifndef TOPO_PLANNER_WS_PCI_MANAGER_H
#define TOPO_PLANNER_WS_PCI_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "control_planner_interface/Path.h"

namespace interface {
    class PCIManager {
    public:
        enum struct PCIStatus {
            kReady = 0, kRunning = 1, kError = 2
        };

        PCIManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
                nh_(nh), nh_private_(nh_private),
                pci_status_(PCIStatus::kReady), force_stop_(false) {

            trajectory_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
                    "pci_command_trajectory_vis", 10);

        }

        virtual bool loadParams(const std::string ns) = 0;

        virtual bool initialize() = 0;

        virtual bool goToWaypoint(const geometry_msgs::Pose &pose) = 0;

        /* Send a path to be executed by the robot, return a new path if the
         * PCI modified the original path based on robot's dynamics.
         */
        virtual bool executePath(const std::vector<control_planner_interface::Path> &path_segments,
                                 std::vector<geometry_msgs::Pose> &modified_path) = 0;

        virtual bool isGoalReached() = 0;

        virtual void cancelCurrentGoal() = 0;

        virtual void stopMove() = 0;

        // Set the current pose of the robot based on odometry or pose.
        virtual void setCurrentPose(const geometry_msgs::Pose &pose) = 0;

        virtual void setFrameId(const std::string &frame_id) = 0;

        // Check if the PCI is ready to use.
        bool isReady() { return (pci_status_ == PCIStatus::kReady); }

        const PCIStatus &getStatus() const { return pci_status_; }

        void setStatus(const PCIStatus &status) { pci_status_ = status; }

        const geometry_msgs::Pose &getState() { return current_pose_; }

        void stopPCI() { force_stop_ = true; }

    protected:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher trajectory_vis_pub_;

        PCIStatus pci_status_;
        geometry_msgs::Pose current_pose_;

        bool force_stop_;

    };

}


#endif //TOPO_PLANNER_WS_PCI_MANAGER_H
