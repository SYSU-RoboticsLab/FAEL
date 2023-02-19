//
// Created by hjl on 2021/9/18.
//

#include "slam_simulation/slam_output.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "slam_sim_output");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    SlamOutput slam_out_put(nh, nh_private);

    ros::spin();
    return 0;
}
