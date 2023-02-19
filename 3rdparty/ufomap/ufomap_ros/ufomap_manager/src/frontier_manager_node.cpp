//
// Created by hjl on 2020/6/19.
//
#include <ros/ros.h>
#include <ufomap_manager/frontier_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_manager");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ufomap_manager::FrontierManager frontier(nh,nh_private);

    ros::spin();
    return 0;
}
