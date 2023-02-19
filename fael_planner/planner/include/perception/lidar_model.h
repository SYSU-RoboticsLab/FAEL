//
// Created by hjl on 2022/4/7.
//

#ifndef ROBO_PLANNER_WS_LIDAR_MODEL_H
#define ROBO_PLANNER_WS_LIDAR_MODEL_H

#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class LidarModel {

    double max_range_;
    int line_num_;
    double vertical_angle_theta_;
    double vertical_angle_min_;
    double vertical_angle_max_;

    int horizontal_point_num_;
    double horizontal_angle_theta_;
    double horizontal_angle_min_;
    double horizontal_angle_max_;

    std::map<int, std::map<int, pcl::PointXYZ>> lidar_point_cloud_;


public:
    //Initialize lidar model
    LidarModel(double max_range,
               double vertical_angle_theta = 2.0, double vertical_angle_min = -15.0, double vertical_angle_max = 15.0,
               double horizontal_angle_theta = 0.4, double horizontal_angle_min = 0.0, double horizontal_angle_max = 360.0);

    pcl::PointCloud<pcl::PointXYZ> combineCloud(pcl::PointCloud<pcl::PointXYZ> &input_cloud);

    pcl::PointCloud<pcl::PointXYZ>  expandNoRay(pcl::PointCloud<pcl::PointXYZ> &input_cloud);

};


#endif //ROBO_PLANNER_WS_LIDAR_MODEL_H
