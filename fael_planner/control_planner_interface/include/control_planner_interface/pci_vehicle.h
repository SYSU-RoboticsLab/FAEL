//
// Created by hjl on 2021/11/30.
//

#ifndef TOPO_PLANNER_WS_PCI_VEHICLE_H
#define TOPO_PLANNER_WS_PCI_VEHICLE_H

#include "control_planner_interface/pci_manager.h"

#include <actionlib/client/simple_action_client.h>
#include <control_planner_interface/VehicleExecuteAction.h>



namespace interface {

    class PCIVehicle : public PCIManager {
    public:

        typedef actionlib::SimpleActionClient<control_planner_interface::VehicleExecuteAction> VehicleExecuteActionClient;

        PCIVehicle(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

        bool loadParams(const std::string ns) override;

        bool initialize() override;

        void setCurrentPose(const geometry_msgs::Pose &pose) override;

        void setFrameId(const std::string &frame_id) override;

        bool goToWaypoint(const geometry_msgs::Pose &pose) override;

        bool executePath(const std::vector<control_planner_interface::Path> &path_segments,
                         std::vector<geometry_msgs::Pose> &modified_path) override;

        bool isGoalReached() override;

        void cancelCurrentGoal() override;

        void stopMove() override;

    private:

        ros::Subscriber status_sub_;
        ros::Publisher robot_status_pub_;

        VehicleExecuteActionClient vehicle_execute_client_;
        VehicleExecuteActionClient vehicle_stop_client_;

        std::string frame_id_;

        int n_seq_;
    };
}


#endif //TOPO_PLANNER_WS_PCI_VEHICLE_H
