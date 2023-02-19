//
// Created by hjl on 2022/1/13.
//

#ifndef ROBO_PLANNER_WS_RAPID_COVER_PLANNER_H
#define ROBO_PLANNER_WS_RAPID_COVER_PLANNER_H

#include "preprocess/preprocess.h"
#include "tsp_solver/two_opt.h"
#include "graph/plan_graph.h"
#include <ros/package.h>

#include <visualization_tools/IterationTime.h>
#include <visualization_tools/ViewpointGain.h>

namespace rapid_cover_planner {
    using namespace utils;
    using namespace perception;
    using namespace preprocess;

    typedef std::vector<utils::Point3D> Path;

    class RapidCoverPlanner {

    public:
        typedef std::shared_ptr<RapidCoverPlanner> Ptr;
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        //parameters required for planner preprocessing
        int max_tour_point_num_;
        double viewpoint_ignore_thre_;
        double local_range_;
        double frontier_gain_;
        double tourpoint_ignore_distance_; 
        double tourpoint_ignore_thre_;

        //data used for planner calculations
        perception::Ufomap::Ptr frontier_map_;
        preprocess::Map2DManager::Ptr map_2d_manager_;
        preprocess::ViewpointManager::Ptr viewpoint_manager_;
        preprocess::TopoGraph::Ptr road_graph_;

        //data after initialization.
        graph::PlanGraph plan_graph_;
        utils::Point3DSet tour_points_;
        utils::Point3DMap<double> tour_points_gains_;
        double max_gain_;
        std::vector<std::vector<Path>> path_matrix_;
        utils::Point3DMap<utils::Point3DMap<Path>> pre_paths_;
        
        //planner iterative output
        utils::Point3D goal_point_;
        Path path_to_go_;
        Path tsp_path_;
        std::vector<Path> path_segments_;

        FrontierQueue goal_point_frontiers_;

        bool is_local_planning_;

        bool is_directory_;
        double alpha_;

        //statistic
        std::string each_solving_txt_name_;
        double sum_solving_time_;
        int solving_num_;

        std::string each_tourpoints_initilize_txt_name_;
        double sum_initilize_time_;
        int initilize_num_;

        std::mutex planner_mutex_;

        std::string two_opt_time_name_;
        double sum_two_opt_time;

        std::vector<double> frontiers_gains_;
        std::vector<double> unmapped_gains_;
        std::vector<double> mean_errors_;

        RapidCoverPlanner(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
                          const Ufomap::Ptr &frontier_map, const Map2DManager::Ptr &map_2d_manager,
                          const ViewpointManager::Ptr &viewpoint_manager, const TopoGraph::Ptr &road_graph);

        void setParamsFromRos();

        void Initialize(const Point3D &current_position);

        void planGraphConstruct(const graph::PlanGraph &old_graph, graph::PlanGraph &new_graph);

        void getSuitableTourPoints(const Point3D &current_position);

        void viewpointsFrontierGain(utils::Point3DSet &viewpoints,utils::Point3DMap<double> &tour_points_gains, double &max_gain);

        void planning(const geometry_msgs::Pose &current_pose, const utils::Point3D &current_directory, bool &is_successed);

        bool two_opt_solve_planning(const geometry_msgs::Pose &current_pose, const utils::Point3D &current_directory);

        int addCurrentPositionToGraph(const Point3D &current_position, graph::PlanGraph &graph);

        Path getPathInGraph(const int &start_point_id, const int &end_point_id, const graph::PlanGraph &graph);

        Path
        getPathInGridMap2D(const Point3D &start_point, const Point3D &end_point, const GridMap2D &grid_map_2d);

    };

}


#endif //ROBO_PLANNER_WS_RAPID_COVER_PLANNER_H
