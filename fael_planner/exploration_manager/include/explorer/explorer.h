//
// Created by hjl on 2021/11/26.
//

#ifndef TOPO_PLANNER_WS_EXPLORER_H
#define TOPO_PLANNER_WS_EXPLORER_H

#include <control_planner_interface/PlannerMsgs.h>
#include <control_planner_interface/control_planner_interface.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

namespace explorer {

    class Explorer {
    public:
        enum struct RunModeType {
            kSim = 0,  // Run in simulation.
            kReal= 1     // Run with real robot.
        };

        enum struct ExecutionPathType {
            kLocalPath = 0,
            kHomingPath = 1,
            kGlobalPath = 2,
            kNarrowEnvPath = 3, // Narrow env.
            kManualPath = 4     // Manually set path.
        };

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber planner_msgs_sub_;

        ros::Publisher init_finish_pub_;
        ros::Publisher finish_explore_pub_;

        ros::Subscriber explore_finish_sub_;


        std::shared_ptr<interface::ControlPlannerInterface> interface_;

        RunModeType run_mode_;
        bool init_motion_enable_;
        double init_x_;
        double init_y_;
        double init_z_;
        double init_need_time_;

        int iteration_num_;

        bool iteration_goal_is_scaned_;
        bool exploration_finished_;
        bool need_to_next_iteration_;

        ros::WallTime follow_start_time_;
        double wait_time_;

        Explorer(ros::NodeHandle &nh, ros::NodeHandle &nh_private, std::shared_ptr<interface::ControlPlannerInterface> &interface);

        bool loadParams();

        bool init();

        bool initMotion();

        bool callForPlanner(const int &iteration_num, std::vector<control_planner_interface::Path> &path_segments);

        void followThePath( const std::vector<control_planner_interface::Path> &path_segments);

        bool isFollowFinish();

        void stopMove();

        bool isWaitTooLong();

        void PlannerMsgsCallback(const control_planner_interface::PlannerMsgsConstPtr &msg);

        void explorationFinishCallback(const std_msgs::BoolConstPtr &finish);

    };
}


#endif //TOPO_PLANNER_WS_EXPLORER_H
