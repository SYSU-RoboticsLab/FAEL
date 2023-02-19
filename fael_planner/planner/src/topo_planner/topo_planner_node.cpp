//
// Created by hjl on 2021/11/16.
//

#include <ros/ros.h>

#include "topo_planner/topo_planner.h"


int main(int argc, char** argv){
    ros::init(argc,argv,"topo_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ROS_INFO_STREAM("start planner");

    topo_planner::TopoPlanner planner(nh, nh_private);

    ros::spin();
    return 0;
}
