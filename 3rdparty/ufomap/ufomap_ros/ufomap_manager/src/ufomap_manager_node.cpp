//
// Created by hjl on 2020/6/15.
//
#include <ros/ros.h>
#include <ufomap_manager/ufomap_manager.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ufomap_manager");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ufomap_manager::UFOMapManager manager(nh,nh_private);

    ros::spin();
    return 0;
}