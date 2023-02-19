//
// Created by hjl on 2021/12/9.
//

#include <explorer/explorer.h>
#include <unordered_map>
#include <control_planner_interface/pci_vehicle.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "explorer_node");


    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    std::shared_ptr<interface::PCIManager> pci_manager = std::make_shared<interface::PCIVehicle>(nh, nh_private);
    std::shared_ptr<interface::ControlPlannerInterface> interface = std::make_shared<interface::ControlPlannerInterface>(
            nh, nh_private, pci_manager);


    explorer::Explorer explorer(nh, nh_private, interface);
    
    return 0;
}
