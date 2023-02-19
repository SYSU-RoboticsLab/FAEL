//
// Created by hjl on 2021/9/1.
//


#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

int main(int argc, char** argv){

    ros::init(argc, argv, "read_trajectory");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher pub = nh_private.advertise<nav_msgs::Odometry>("trajectory_odom", 1);
    ros::Publisher pub_point = nh_private.advertise<sensor_msgs::PointCloud2>("trajectory_points",1);

    std::string ns = ros::this_node::getName();
    std::string txt_name = "tracjectory.txt";
    if (!ros::param::get(ns + "/txt_name", txt_name)) {
        ROS_WARN(
                "No txt_name specified. Looking for %s. Default is 'tracjectory.txt'.",
                (ns + "/txt_name").c_str());
    }

    nav_msgs::Odometry robot_odom;
    sensor_msgs::PointCloud2  point_cloud;
    robot_odom.header.frame_id = "world";
    point_cloud.header.frame_id = "world";

    string line;
    ifstream fin;
    fin.open(txt_name,ios::in);
    if(!fin.is_open()){
        std::cout<<"file open failed"<<std::endl;
    }
    getline(fin,line);

    ROS_INFO("start pub trajectory..");
    int seq = 1;
    while(getline(fin,line)){
        int i=0;
        string tmp;
        stringstream ss(line);
        while (getline(ss, tmp, '\t')) {
            if (i == 1) robot_odom.pose.pose.position.x = stod(tmp);
            if (i == 2) robot_odom.pose.pose.position.y = stod(tmp);
            if (i == 3) robot_odom.pose.pose.position.z = stod(tmp);
            i++;
        }

        robot_odom.header.seq = seq;
        pub.publish(robot_odom);
        ros::spinOnce();
        ros::Duration(0.005).sleep();
        seq++;
    }


    ROS_INFO("pub finish");

    return 0;
}