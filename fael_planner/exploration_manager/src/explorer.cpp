//
// Created by hjl on 2021/11/26.
//

#include "explorer/explorer.h"

namespace explorer {

    Explorer::Explorer(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
                       std::shared_ptr<interface::ControlPlannerInterface> &interface) :
            nh_(nh), nh_private_(nh_private), interface_(interface), iteration_num_(0) {

        planner_msgs_sub_ = nh_.subscribe<control_planner_interface::PlannerMsgs>("topo_planner_msgs", 1,
                                                                                  &Explorer::PlannerMsgsCallback, this);
        explore_finish_sub_ = nh_.subscribe<std_msgs::Bool>("exploration_data_finish", 1,
                                                            &Explorer::explorationFinishCallback,
                                                            this);

        init_finish_pub_ = nh_private_.advertise<std_msgs::Float64>("explorer_inited", 1);
        finish_explore_pub_ = nh_private_.advertise<std_msgs::Float64>("explorer_finish", 1);


        loadParams();

        // Wait for the system is ready.
        if (run_mode_ == RunModeType::kSim) {
            std_srvs::Empty srv;
            bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
            unsigned int i = 0;
            while (i <= 10 && !unpaused) {
                ROS_WARN("Wait for 1 second before trying to unpause Gazebo again.");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                unpaused = ros::service::call("/gazebo/unpause_physics", srv);
                ++i;
            }
            if (!unpaused) {
                ROS_FATAL("Could not wake up Gazebo.");
                ros::shutdown();
            } else {
                ROS_INFO("Unpaused the Gazebo simulation.");
            }

            if (!init())
                ros::shutdown();

        } else if (run_mode_ == RunModeType::kReal) {
            if (!init())
                ros::shutdown();
        }

        //publish exploration status
        double start_explore_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count()) / 1000000;
        std_msgs::Float64 is_init_finish;
        is_init_finish.data = start_explore_time;
        init_finish_pub_.publish(is_init_finish);
        ros::spinOnce();
        ROS_WARN("init motion has finished, start exploring...");
        ros::Duration(1.0).sleep();

        exploration_finished_ = false;
        iteration_goal_is_scaned_ = false;
        need_to_next_iteration_ = true;

        while (ros::ok()) {
            if (exploration_finished_) {
                double finish_explore_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::high_resolution_clock::now().time_since_epoch()).count()) / 1000000;
                ROS_WARN("** the exploration process has finished **");
                ROS_WARN("total_time is %f s",(finish_explore_time - start_explore_time));
                stopMove();

                std_msgs::Float64 finish;
                finish.data = finish_explore_time;
                finish_explore_pub_.publish(finish);
                ros::spinOnce();

                ros::Duration(2.0).sleep();
                ros::shutdown();
            }

            if (iteration_goal_is_scaned_ || need_to_next_iteration_) {
                if (iteration_goal_is_scaned_) {
                }
                if (need_to_next_iteration_)

                //Planning And Execute
                iteration_num_++;
                ROS_INFO("**Planning iteration  %i**", iteration_num_);
                std::vector<control_planner_interface::Path> path_segments;
                if (callForPlanner(iteration_num_, path_segments)) {
                    if (path_segments.empty()) {
                        ROS_WARN("this iteration get an empty path ,need next planning iteration");
                        need_to_next_iteration_ = true;
                    } else {
                        //got a reasonable path
                        ROS_INFO("following the path...");
                        followThePath(path_segments);
                        need_to_next_iteration_ = false;

                        float distance = 0.0;//use the first segment to calculate time
                        for (int i = 1; i < path_segments.front().path.size(); i++) {
                            distance += sqrt(std::pow(path_segments.front().path[i].position.x -
                                                      path_segments.front().path[i - 1].position.x, 2) +
                                             std::pow(path_segments.front().path[i].position.y -
                                                      path_segments.front().path[i - 1].position.y, 2));
                        }
                        float ave_speed = 0.3;
                        wait_time_ = distance / ave_speed; 
                    }
                } else {
                    //exploration failed
                    ROS_WARN("the iteration is out time or aborted");
                    need_to_next_iteration_ = true;
                }

            } else if (isFollowFinish()) {
                ROS_INFO("the iteration planning %i is follow path finished", iteration_num_);
                need_to_next_iteration_ = true;
            } else if (isWaitTooLong()) {
                ROS_WARN("this iteration planning %i is run too time to reach the goal", iteration_num_);
                need_to_next_iteration_ = true;
            }

            if ((ros::WallTime::now() - follow_start_time_).toSec() > 0.1)//10hz

                need_to_next_iteration_ = true;

            ros::spinOnce();
        }
    }

    bool Explorer::loadParams() {

        const std::string &ns = ros::this_node::getName();

        std::string parse_str;
        ros::param::get(ns + "/run_mode", parse_str);
        if (!parse_str.compare("kReal"))
            run_mode_ = RunModeType::kReal;
        else if (!parse_str.compare("kSim"))
            run_mode_ = RunModeType::kSim;
        else {
            run_mode_ = RunModeType::kSim;
            ROS_WARN("No run mode setting, set it to kSim.");
        }

        init_motion_enable_ = true;
        if (!ros::param::get(ns + "/init_motion_enable", init_motion_enable_)) {
            ROS_WARN("No motion initialization setting, set it to True.");
        }

        init_x_ = 0.0;
        if (!ros::param::get(ns + "/init_x", init_x_)) {
            ROS_WARN("No init_x specified. Looking for %s. Default is '0.0'.",
                     (ns + "/init_x").c_str());
        }

        init_y_ = 0.0;
        if (!ros::param::get(ns + "/init_y", init_y_)) {
            ROS_WARN("No init_y specified. Looking for %s. Default is '0.0'.",
                     (ns + "/init_y").c_str());
        }

        init_z_ = 0.0;
        if (!ros::param::get(ns + "/init_z", init_z_)) {
            ROS_WARN("No init_z specified. Looking for %s. Default is '0.0'.",
                     (ns + "/init_z").c_str());
        }

        init_need_time_ = 10.0;
        if (!ros::param::get(ns + "/init_need_time", init_need_time_)) {
            ROS_WARN("No init_need_time specified. Looking for %s. Default is '10.0'.",
                     (ns + "/init_need_time").c_str());
        }

        return true;
    }

    bool Explorer::init() {
        if (!interface_->init()) {
            ROS_INFO("control planner interface init failed.");
            return false;
        }

        if (init_motion_enable_) {
            if (!initMotion()) {
                ROS_INFO("explorer init motion failed.");
                return false;
            }
        }
        return true;
    }

    bool Explorer::initMotion() {
        //initialization of exploration
        ros::Duration(1.0).sleep();
        ROS_INFO("Performing initialization motion");
        ROS_INFO("Current pose: %f, %f, %f", interface_->current_pose_.position.x,
                 interface_->current_pose_.position.y, interface_->current_pose_.position.z);

        geometry_msgs::Pose pose;
        pose.position.x = interface_->current_pose_.position.x + init_x_;
        pose.position.y = interface_->current_pose_.position.y + init_y_;
        pose.position.z = interface_->current_pose_.position.z + init_z_;
        pose.orientation.x = interface_->current_pose_.orientation.x;
        pose.orientation.y = interface_->current_pose_.orientation.y;
        pose.orientation.z = interface_->current_pose_.orientation.z;
        pose.orientation.w = interface_->current_pose_.orientation.w;

        interface_->goToWayPose(pose);

        ros::Duration(init_need_time_).sleep();

        return true;
    }

    bool
    Explorer::callForPlanner(const int &iteration_num, std::vector<control_planner_interface::Path> &path_segments) {
        return interface_->callForPlanner(iteration_num, path_segments);
    }

    void Explorer::followThePath(const std::vector<control_planner_interface::Path> &path_segments) {
        follow_start_time_ = ros::WallTime::now();
        interface_->executePath(path_segments);
    }

    bool Explorer::isFollowFinish() {
        return interface_->isGoalReached();
    }

    void Explorer::stopMove() {
        interface_->cancelCurrentGoal();
        ROS_INFO("..... the robot is going to stop .....");
        interface_->stopMove();
    }

    bool Explorer::isWaitTooLong() {
        return (ros::WallTime::now() - follow_start_time_).toSec() > wait_time_;
    }

    void Explorer::PlannerMsgsCallback(const control_planner_interface::PlannerMsgsConstPtr &msg) {
        
        exploration_finished_ = msg->is_exploration_finished;

        if (iteration_num_ == msg->current_goal_id) {
            iteration_goal_is_scaned_ = msg->is_current_goal_scanned;
        } else
            iteration_goal_is_scaned_ = false;
    }

    void Explorer::explorationFinishCallback(const std_msgs::BoolConstPtr &finish) {
        if (finish->data == true)
            exploration_finished_ = true;
    }
}