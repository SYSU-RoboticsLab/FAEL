//
// Created by hjl on 2021/7/30.
//

#include "exploration_data.h"

exploration_data::exploration_data(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private), path_length_sum(0.0),
                                                                        last_position(0, 0, 0), known_cell_num(0), known_plane_cell_num(0),
                                                                        known_map_resolution(0), known_space_volume(0), system_inited(false),
                                                                        exploration_finish(false), seq(0)
{

    init_sub = nh_.subscribe<std_msgs::Float64>("explorer_inited", 1, &exploration_data::explorationInitCallback, this);
    finish_sub = nh_.subscribe<std_msgs::Float64>("explorer_finish", 1, &exploration_data::explorationFinishCallback, this);
    odom_sub = nh_.subscribe<nav_msgs::Odometry>("odometry", 1, &exploration_data::odomCallback, this);
    map_frontiers_sub = nh_.subscribe<ufomap_manager::UfomapWithFrontiers>("ufomap_and_frontiers", 1,
                                                                           &exploration_data::mapAndFrontiersCallback,
                                                                           this);
    iteration_time_sub = nh_.subscribe<visualization_tools::IterationTime>("iteration_time", 1,
                                                                           &exploration_data::iterationTimeCallback,
                                                                           this);

    traved_dist_pub = nh_.advertise<visualization_tools::TravedDistTime>("traved_distance_time", 1);
    explorated_volume_pub = nh_.advertise<visualization_tools::ExploredVolumeTime>("explored_volume_time", 1);
    explorated_volume_traved_dist_time_pub = nh_.advertise<visualization_tools::ExploredVolumeTravedDistTime>(
        "explored_volume_traved_dist_time", 1);

    iteration_time_pub = nh_.advertise<visualization_tools::IterationTime>("run_time", 1);
    pub_timer = nh_private_.createTimer(ros::Duration(1.0), &exploration_data::pubExplorationData, this);

    explore_finish_pub = nh_.advertise<std_msgs::Bool>("exploration_data_finish", 1);

    const std::string &ns = ros::this_node::getName();
    std::string pkg_path = ros::package::getPath("visualization_tools");
    std::string txt_path = pkg_path + "/../../files/exploration_data/";

    max_volume = 100000000.0;
    if (!ros::param::get(ns + "/map_area", max_volume))
    {
        ROS_WARN("No map_area specified. Looking for %s. Default is 100000000", (ns + "/map_area").c_str());
    }

    max_time = 100000000.0;
    if (!ros::param::get(ns + "/max_time", max_time))
    {
        ROS_WARN("No max_time specified. Looking for %s. Default is 100000000", (ns + "/max_time").c_str());
    }

    distance_volume_txt_name = txt_path + "time_distance_volume.txt";
    iteration_time_txt_name = txt_path + "iteration_time.txt";
    trajectory_txt_name = txt_path + "trajectory.txt";

    std::ofstream fout;
    fout.open(distance_volume_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    fout << "explored time \t"
         << "distance/path-length \t"
         << "explored volume(m2) \t"
         << "known plane cell num \t"
         << "known voxel num(3d) \t"
         << std::endl;
    fout.close();

    sum_iteration_time = 0;
    iteration_num = 0;
    fout.open(iteration_time_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    fout << "explored time \t"
         << " iteration time \t"
         << "iteration_num \t"
         << "average time(s) \t"
         << std::endl;
    fout.close();

    fout.open(trajectory_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    fout << "explored time \t"
         << " x \t"
         << "y \t"
         << "z \t"
         << std::endl;
    fout.close();
}

void exploration_data::explorationInitCallback(const std_msgs::Float64ConstPtr &init)
{

    if (!system_inited)
    {
        system_init_time = init->data;
        system_inited = true;
    }
    ROS_INFO("visualization statics start.");
}

void exploration_data::explorationFinishCallback(const std_msgs::Float64ConstPtr &finish)
{

    exploration_finish = true;
    finish_time = finish->data;
}

void exploration_data::odomCallback(const nav_msgs::OdometryConstPtr &input)
{

    tf::Vector3 current_position(input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z);
    double dist_delta = current_position.distance(last_position);
    last_position = current_position;

    if (system_inited)
    {
        path_length_sum += dist_delta;
    }
}

void exploration_data::mapAndFrontiersCallback(const ufomap_manager::UfomapWithFrontiersConstPtr &msg)
{

    known_map_resolution = msg->ufomap.info.resolution;
    known_cell_num = msg->known_cell_codes.size();
    known_plane_cell_num = msg->known_plane_cell_num;
    known_space_volume =
        static_cast<double>(known_plane_cell_num) * known_map_resolution * known_map_resolution; // 暂时发布面积
}

void exploration_data::iterationTimeCallback(const visualization_tools::IterationTimeConstPtr &msg)
{
    if (system_inited && !exploration_finish)
    {

        visualization_tools::IterationTime time;
        time.timeConsumed = msg->current_time - system_init_time;
        time.iterationTime = msg->iterationTime;
        iteration_time_pub.publish(time);

        sum_iteration_time += time.iterationTime;
        iteration_num++;
        std::ofstream fout;
        fout.open(iteration_time_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << time.timeConsumed << "\t" << time.iterationTime << "\t" << iteration_num << "\t"
             << sum_iteration_time / iteration_num << "\t"
             << std::endl;
        fout.close();
    }
}

void exploration_data::pubExplorationData(const ros::TimerEvent &event)
{

    if (system_inited && !exploration_finish)
    {
        visualization_tools::ExploredVolumeTravedDistTime exploration_data;
        exploration_data.header.stamp = ros::Time::now();
        exploration_data.header.frame_id = "world";
        exploration_data.header.seq = ++seq;

        exploration_data.exploredVolume = known_space_volume;
        exploration_data.travedDist = path_length_sum;
        double time_second = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                     std::chrono::high_resolution_clock::now().time_since_epoch())
                                                     .count()) /
                             1000000;
        exploration_data.timeConsumed = time_second - system_init_time;

        if (exploration_data.exploredVolume > 0.95 * max_volume)
        {
            exploration_finish = true;
            std_msgs::Bool explore_finish;
            explore_finish.data = true;
            explore_finish_pub.publish(explore_finish);
            ROS_INFO("max volume threshold is reach...");
        }

        if (time_second - system_init_time > max_time)
        {
            exploration_finish = true;
            std_msgs::Bool explore_finish;
            explore_finish.data = true;
            explore_finish_pub.publish(explore_finish);
            ROS_INFO("max time is reach...");
        }

        explorated_volume_traved_dist_time_pub.publish(exploration_data);

        std::ofstream fout;
        fout.open(distance_volume_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << exploration_data.timeConsumed << "\t" << exploration_data.travedDist << "\t"
             << exploration_data.exploredVolume << "\t" << known_plane_cell_num << "\t"
             << known_cell_num << std::endl;
        fout.close();

        fout.open(trajectory_txt_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << exploration_data.timeConsumed << "\t" << last_position.x() << "\t" << last_position.y() << "\t"
             << last_position.z() << std::endl;
        fout.close();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration_data");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    exploration_data data(nh, nh_private);

    while (ros::ok())
    {
        ros::spinOnce();

        if (data.exploration_finish)
        {
            std_msgs::Bool explore_finish;
            explore_finish.data = true;
            data.explore_finish_pub.publish(explore_finish);
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            ros::shutdown();
        }
    }
    return 0;
}
