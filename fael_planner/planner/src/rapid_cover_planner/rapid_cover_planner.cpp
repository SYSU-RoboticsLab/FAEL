//
// Created by hjl on 2022/1/13.
//

#include "rapid_cover_planner/rapid_cover_planner.h"

namespace rapid_cover_planner {

    RapidCoverPlanner::RapidCoverPlanner(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
                                         const Ufomap::Ptr &frontier_map, const Map2DManager::Ptr &map_2d_manager,
                                         const ViewpointManager::Ptr &viewpoint_manager,
                                         const TopoGraph::Ptr &road_graph) :
            nh_(nh), nh_private_(nh_private) {
        frontier_map_ = frontier_map;
        map_2d_manager_ = map_2d_manager;
        viewpoint_manager_ = viewpoint_manager;
        road_graph_ = road_graph;

        setParamsFromRos();
    }

    void RapidCoverPlanner::setParamsFromRos() {
        std::string ns = ros::this_node::getName() + "/RapidCoverPlanner";

        std::string pkg_path = ros::package::getPath("planner");
        std::string txt_path = pkg_path + "/../../files/exploration_data/";
      
        max_tour_point_num_ = 10;
        if (!ros::param::get(ns + "/max_tour_point_num", max_tour_point_num_)) {
            ROS_WARN("No max_tour_point_num specified. Looking for %s. Default is 10",
                     (ns + "/max_tour_point_num").c_str());
        }

        viewpoint_ignore_thre_ = 1.0;
        if (!ros::param::get(ns + "/viewpoint_ignore_thre", viewpoint_ignore_thre_)) {
            ROS_WARN("No viewpoint_ignore_thre specified. Looking for %s. Default is 1.0",
                     (ns + "/viewpoint_ignore_thre").c_str());
        }

        tourpoint_ignore_distance_ = 3.0;
        if (!ros::param::get(ns + "/tourpoint_ignore_distance", tourpoint_ignore_distance_)) {
            ROS_WARN("No tourpoint_ignore_distance specified. Looking for %s. Default is 3.0",
                     (ns + "/tourpoint_ignore_distance").c_str());
        }

        tourpoint_ignore_thre_ = 2.0;
        if (!ros::param::get(ns + "/tourpoint_ignore_thre", tourpoint_ignore_thre_)) {
            ROS_WARN("No tourpoint_ignore_thre specified. Looking for %s. Default is 2.0",
                     (ns + "/tourpoint_ignore_thre").c_str());
        }

        local_range_ = 10.0;
        if (!ros::param::get(ns + "/local_range", local_range_)) {
            ROS_WARN("No local_range specified. Looking for %s. Default is 1.0",
                     (ns + "/local_range").c_str());
        }
        frontier_gain_ = 1.0;
        if (!ros::param::get(ns + "/frontier_gain", frontier_gain_)) {
            ROS_WARN("No frontier_gain specified. Looking for %s. Default is 1.0",
                     (ns + "/frontier_gain").c_str());
        }

        is_local_planning_ = true;

        is_directory_ = true;
        if (!ros::param::get(ns + "/is_directory", is_directory_)) {
            ROS_WARN("No is_directory specified. Looking for %s. Default is true",
                     (ns + "/is_directory").c_str());
        }

        alpha_ = 0.5;
        if (!ros::param::get(ns + "/alpha", alpha_)) {
            ROS_WARN("No alpha specified. Looking for %s. Default is 0.5",
                     (ns + "/alpha").c_str());
        }

        each_tourpoints_initilize_txt_name_ = txt_path + "each_tourpoints_initilize_time.txt";
        each_solving_txt_name_            = txt_path + "each_solving_time.txt";
        
        std::ofstream fout;

        sum_initilize_time_ = 0;
        initilize_num_ = 0;
        fout.open(each_tourpoints_initilize_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "each detection elisped time \n"
             << "start time \t" << "end time \t" << "elisped time \t"
             << "detection_num \t" << "average time \t"
             << std::endl;
        fout.close();

        sum_solving_time_ = 0;
        solving_num_ = 0;
        fout.open(each_solving_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "each iteration elisped time \n"
             << "iteration start time \t" << " iteration end time \t" << " iteration elisped time \t"
             << "iteration_num \t" << "average time \t"
             << std::endl;
        fout.close();

        two_opt_time_name_ = txt_path + "two_opt_time.txt";
        fout.open(two_opt_time_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "solve_number\t" << "\t" << "solve_time(s)\t" << "mean_time(s)\t" << "points_num\t" << std::endl;
        fout.close();
        sum_two_opt_time = 0.0;
    }

    void RapidCoverPlanner::planGraphConstruct(const graph::PlanGraph &old_graph, graph::PlanGraph &new_graph) {
        new_graph.clearGraph();
        new_graph = old_graph;
    }

    void RapidCoverPlanner::Initialize(const Point3D &current_position) {
        ROS_INFO("start initializing ....");
        ros::WallTime start_time = ros::WallTime::now();
        initilize_num_++;

        getSuitableTourPoints(current_position);

        //calculate frontier gain
        tour_points_gains_.clear();
        max_gain_ = 0.0;
        viewpointsFrontierGain(tour_points_, tour_points_gains_, max_gain_);

        if (tour_points_.size() > 3) {
            utils::Point3DSet tour_points;
            std::map<double, Point3DQueue> distances_viewpoints;

            for (const auto &item: tour_points_) {
                auto distance = item.distanceXY(current_position);
                distances_viewpoints[distance].push_back(item);
            }

            for (const auto &item: distances_viewpoints) {
                if (item.first < tourpoint_ignore_distance_) {
                    for (const auto &point:item.second) {
                        if (viewpoint_manager_->viewpoints_attached_frontiers_[point].size() *
                            frontier_map_->map_.getResolution() * frontier_map_->map_.getResolution()
                            > tourpoint_ignore_thre_) {
                            tour_points.insert(point);
                        }
                    }
                } else {
                    for (const auto &point:item.second) {
                        tour_points.insert(point);
                    }
                }
            }
            tour_points_ = tour_points;
        }
        ROS_INFO("plan graph update...");
        planGraphConstruct(road_graph_->graph_, plan_graph_);
        ROS_INFO(" the plan graph update finish . vertex num is %zu, edge num is %zu ",
                 plan_graph_.getAllVertices().size(), plan_graph_.getAllEdges().size());
        //add the viewpoints to the plan graph
        plan_graph_.updateKdtree();
        for (auto &viewpoint: tour_points_) {
            if (!plan_graph_.isPoint3DExisted(viewpoint)) {
                int a_id = plan_graph_.addVertex(viewpoint);
                int b_id = plan_graph_.getNearestVertexId(plan_graph_.kd_tree_, viewpoint);
                if (b_id != -1) {
                    plan_graph_.addTwoWayEdge(a_id, b_id);
                    ROS_INFO("added a tour point and its edges in plan graph.");
                }
            }
        }

        ros::WallTime end_time = ros::WallTime::now();
        ROS_INFO("fast ray casting gain computed finish,spent %f s", (end_time - start_time).toSec());
        sum_initilize_time_ = sum_initilize_time_ + (end_time - start_time).toSec();
        std::ofstream fout;
        fout.open(each_tourpoints_initilize_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << start_time << "\t" << end_time << "\t" << (end_time - start_time).toSec()
             << "\t" << initilize_num_ << "\t" << sum_initilize_time_ / initilize_num_ << "s \t"
             << std::endl;
        fout.close();
    }

    void RapidCoverPlanner::getSuitableTourPoints(const Point3D &current_position) {

        //tour points detection
        tour_points_.clear();

        utils::Point3DSet local_viewpoints;
        utils::Point3DSet global_viewpoints;
        for (const auto &item: viewpoint_manager_->viewpoints_attached_frontiers_) {
            if (item.first.distanceXY(current_position) > 1.0  
                &&
                item.second.size() *frontier_map_->map_.getResolution() * frontier_map_->map_.getResolution()
                    > viewpoint_ignore_thre_ 
                    ) {

                if (item.first.distanceXY(viewpoint_manager_->current_position_) < local_range_ / 2) {
                    local_viewpoints.insert(item.first);
                } else {
                    global_viewpoints.insert(item.first);
                }
            }
        }

        if (local_viewpoints.size() > max_tour_point_num_) 
        {
          
            std::map<double, Point3DQueue> distances_viewpoints;
            std::vector<double> distances;
            for (const auto &item: local_viewpoints) {
                auto distance = item.distanceXY(current_position);
                if (distance > 1.0) {
                    distances.push_back(distance);
                    distances_viewpoints[distance].push_back(item);
                }
            }
            std::sort(distances.begin(), distances.end());

            int i = 0;
            for (const auto &item: distances_viewpoints) {
                for (const auto &viewpoint: item.second) {
                    tour_points_.insert(viewpoint);
                    ++i;
                    if (i >= max_tour_point_num_)
                        break;
                }
                if (i >= max_tour_point_num_)
                    break;
            }
        } 
        else 
        {
            if (!local_viewpoints.empty()) {
                is_local_planning_ = true;
                tour_points_ = local_viewpoints;
            } else {
                is_local_planning_ = false;
                auto graph = plan_graph_;
                int current_point_id = addCurrentPositionToGraph(current_position, graph);
                int max_num = max_tour_point_num_;
                if (global_viewpoints.size() > max_num) 
                {
                    std::map<double, Point3DQueue> distances_viewpoints;
                    std::vector<double> distances;
                    for (const auto &item: global_viewpoints) {
                        int item_point_id;
                        if (graph.getPointId(item, item_point_id)) {
                            auto path = getPathInGraph(current_point_id, item_point_id, graph);
                            double path_length = 0.0;
                            if (!path.empty()) {
                                for (int i = 0; i < path.size() - 1; ++i) {
                                    path_length = path_length + path[i].distance(path[i + 1]);
                                }
                                if (path_length > 1.0) {
                                    distances.push_back(path_length);
                                    distances_viewpoints[path_length].push_back(item);
                                }
                            } else {
                                auto distance = item.distanceXY(current_position);
                                if (distance > 1.0) {
                                    distances.push_back(distance);
                                    distances_viewpoints[distance].push_back(item);
                                }
                            }
                        } else {
                            auto distance = item.distanceXY(current_position);
                            if (distance > 1.0) {
                                distances.push_back(distance);
                                distances_viewpoints[distance].push_back(item);
                            }
                        }
                    }
                    std::sort(distances.begin(), distances.end());
                    int i = 0;
                    for (const auto &item: distances_viewpoints) 
                    {
                        for (const auto &viewpoint: item.second) 
                        {
                            tour_points_.insert(viewpoint);
                            ++i;
                            if (i >= max_num)
                                break;
                        }
                        if (i >= max_num)
                            break;
                    }
                } 
                else 
                {
                    tour_points_ = global_viewpoints;
                }
            }
        }
        ROS_INFO(" tour points detection finished, get %zu points to view", tour_points_.size());
    }

    void RapidCoverPlanner::viewpointsFrontierGain(utils::Point3DSet &viewpoints,
                                                   utils::Point3DMap<double> &tour_points_gains, double &max_gain) {
        ROS_INFO("start tour points gain computing...");

        for (const auto &viewpoint: viewpoints) {
            FrontierQueue viewpoint_visual_frontiers;
            for (const auto &frontier: frontier_map_->getFrontiers()) {
                Point3D sensor_point(viewpoint.x(), viewpoint.y(), viewpoint.z());
                if (frontier.distanceXY(sensor_point) < frontier_map_->max_range_ &&
                    fabs(frontier.z() - sensor_point.z()) / frontier.distanceXY(sensor_point) < tan(M_PI * 15 / 180) &&
                    frontier_map_->map_.isCollisionFree(
                            ufo::map::Point3(sensor_point.x(), sensor_point.y(), sensor_point.z()),
                            ufo::map::Point3(frontier.x(), frontier.y(), frontier.z()))) {
                    viewpoint_visual_frontiers.push_back(frontier);
                }
            }
            tour_points_gains[viewpoint] = static_cast<double>(viewpoint_visual_frontiers.size()) * frontier_gain_;
            if (tour_points_gains[viewpoint] > max_gain) {
                max_gain = tour_points_gains[viewpoint];
            }
        }
    }

    void RapidCoverPlanner::planning(const geometry_msgs::Pose &current_pose, const utils::Point3D &current_directory, bool &is_successed) {
        planner_mutex_.lock();
        solving_num_++;

        auto start_time = std::chrono::high_resolution_clock::now();
        double start_time_second = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                start_time.time_since_epoch()).count()) / 1000000;

        if (two_opt_solve_planning(current_pose, current_directory)) {
            if (viewpoint_manager_->viewpoints_attached_frontiers_.count(goal_point_) != 0) {
                goal_point_frontiers_ = viewpoint_manager_->viewpoints_attached_frontiers_.at(goal_point_);
            } else {
                goal_point_frontiers_.clear();
            }
            ROS_INFO("this iteration planning successed.");
            is_successed = true;
        } else {
            goal_point_frontiers_.clear();
            ROS_WARN("this iteration planning failed");
            is_successed = false;
        }

        auto finish_time = std::chrono::high_resolution_clock::now();
        double finish_time_second = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                finish_time.time_since_epoch()).count()) / 1000000;

        double iteration_time = finish_time_second - start_time_second;
        sum_solving_time_ = sum_solving_time_ + iteration_time;
        std::ofstream time_fout;
        std::ofstream count_fout;
        time_fout.precision(16);
        count_fout.precision(6);

        time_fout.open(each_solving_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        time_fout << start_time_second << "\t" << finish_time_second << "\t";
        time_fout.close();
        count_fout.open(each_solving_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        count_fout << iteration_time << "\t" << solving_num_ << "\t"
                   << sum_solving_time_ / solving_num_ << "s \t" << tour_points_.size() << std::endl;
        count_fout.close();

        planner_mutex_.unlock();
    }

    bool RapidCoverPlanner::two_opt_solve_planning(const geometry_msgs::Pose &current_pose, const utils::Point3D &current_directory) {
        Point3D current_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);

        path_to_go_.clear();
        tsp_path_.clear();
        path_segments_.clear();
        if (tour_points_.empty()) {
            if (plan_graph_.getAllVertices().size() < 5) {
                ROS_INFO("plan_map is not construct or vertex num is too few, please wait a minute...");
            } else
                ROS_WARN(" the candidate points is empty, exploration finish. ");
            return false;
        } else {
            std::vector<Point3D> need_to_erase;
            for (auto &item1:pre_paths_) {
                if (viewpoint_manager_->viewpoints_attached_frontiers_.count(item1.first) == 0) {
                    need_to_erase.push_back(item1.first);
                } else {
                    std::vector<Point3D> to_erase;
                    for (const auto &item2:item1.second) {
                        if (viewpoint_manager_->viewpoints_attached_frontiers_.count(item2.first) == 0 ||
                            item2.second.empty()) {
                            to_erase.push_back(item2.first);
                        }
                    }
                    for (const auto &point:to_erase) {
                        item1.second.erase(point);
                    }
                }
            }
            for (const auto &point: need_to_erase) {
                pre_paths_.erase(point);
            }
    
            int current_point_id = addCurrentPositionToGraph(current_position, plan_graph_);
            ROS_INFO("current position is x = %f, y= %f, z = %f, id is %d", current_position.x(), current_position.y(),
                     current_position.z(), current_point_id);

            Point3DQueue tour_points_term;
            tour_points_term.push_back(current_position);
            for (const auto &point: tour_points_) {
                tour_points_term.push_back(point);
            }
            int tour_points_num = tour_points_term.size();

            ROS_INFO("start get path maxtirx of tour points");

            Path path_matrix[tour_points_num][tour_points_num];
            Path empty_path;
            for (int i = 0; i < tour_points_num; ++i) {
                for (int j = 0; j < tour_points_num; ++j) {
                    path_matrix[i][j] = empty_path;
                }
            }
            for (int i = 1; i < tour_points_num; i++) {
                ROS_INFO("local point get path in terrain map..");
                Path path = getPathInGridMap2D(current_position, tour_points_term.at(i), map_2d_manager_->inflate_map_);
                if (path.empty()) {
                    int end_point_id;
                    if (plan_graph_.getPointId(tour_points_term.at(i), end_point_id)) {
                        ROS_INFO("local point get path in road map..");
                        path = getPathInGraph(current_point_id, end_point_id, plan_graph_);
                    }
                }
                ROS_INFO("path into matrix..");
                path_matrix[0][i] = path;
                std::reverse(path.begin(), path.end());
                path_matrix[i][0] = path;
            }
            ROS_INFO("start get other tour points paths..");
            for (int i = 1; i < tour_points_num; ++i) {
                for (int j = 1; j < tour_points_num; ++j) {
                    if (i == j) {
                        path_matrix[i][j] = empty_path;
                    } else if (path_matrix[i][j].empty()) {
                        Path path = getPathInGridMap2D(tour_points_term.at(i), tour_points_term.at(j),
                                                       map_2d_manager_->inflate_map_);
                        if (path.size() >= 2) {
                            pre_paths_[tour_points_term.at(i)][tour_points_term.at(j)] = path;
                            auto path_reverse = path;
                            std::reverse(path_reverse.begin(), path_reverse.end());
                            pre_paths_[tour_points_term.at(j)][tour_points_term.at(i)] = path_reverse;
                        }
                        if (path.empty()) {
                            if (pre_paths_.count(tour_points_term.at(i)) != 0 &&
                                pre_paths_[tour_points_term.at(i)].count(tour_points_term.at(j)) != 0) {
                                path = pre_paths_[tour_points_term.at(i)][tour_points_term.at(j)];
                            } else {
                                int start_point_id;
                                int end_point_id;
                                if (plan_graph_.getPointId(tour_points_term.at(i), start_point_id) &&
                                    plan_graph_.getPointId(tour_points_term.at(j), end_point_id)) {
                                    path = getPathInGraph(start_point_id, end_point_id, plan_graph_);
                                    if (path.size() >= 2) {
                                        pre_paths_[tour_points_term.at(i)][tour_points_term.at(j)] = path;
                                        auto path_reverse = path;
                                        std::reverse(path_reverse.begin(), path_reverse.end());
                                        pre_paths_[tour_points_term.at(j)][tour_points_term.at(i)] = path_reverse;
                                    }
                                }
                            }
                        }
                        path_matrix[i][j] = path;
                        std::reverse(path.begin(), path.end());
                        path_matrix[j][i] = path;
                    }
                }
            }
            ROS_INFO("path matrix got, start get cost matrix...");

            std::vector<std::vector<double>> cost_matrix =
                    std::vector<std::vector<
                            double >>(tour_points_num, std::vector<double>(tour_points_num, 10000000.0));
            for (int x = 1; x < tour_points_num; ++x) {
                for (int y = 1; y < tour_points_num; ++y) {
                    if (x == y) {
                        cost_matrix[x][y] = 0.0;
                    } else if (path_matrix[x][y].size() < 2) {
                        cost_matrix[x][y] = 10000000;
                    } else {
                        double path_length = 0.0;
                        for (int k = 0; k < path_matrix[x][y].size() - 1; k++) {
                            path_length = path_length + path_matrix[x][y][k].distance(path_matrix[x][y][k + 1]);
                        }
                        cost_matrix[x][y] = path_length;
                    }
                }
            }
           
            for (int y = 1; y < tour_points_num; y++) {
                if (path_matrix[0][y].size() < 2) {
                    cost_matrix[0][y] = 100000000;
                } else {
                    double path_length = 0.0;
                    for (int k = 0; k < path_matrix[0][y].size() - 1; k++) {
                        path_length = path_length + path_matrix[0][y][k].distance(path_matrix[0][y][k + 1]);
                    }
                    cost_matrix[0][y] = path_length;

                    if(is_directory_){
                        if(tour_points_term[y].distance(current_position)<frontier_map_->max_range_){
                            Point3D diff_vector(tour_points_term[y].x() - current_position.x(),
                                                tour_points_term[y].y() - current_position.y(), 0);
                            double theta = current_directory.angleTo(diff_vector);
                            cost_matrix[0][y] = cost_matrix[0][y] * ((1-alpha_) * (log(theta / M_PI+1)/log(2))+alpha_);
                        }
                    }
                }
            }
            for (int x = 0; x < tour_points_num; ++x) {
                cost_matrix[x][0] = 0;
            }

            ROS_INFO("cost matrix got, start planning..");
            std::vector<std::vector<int>> nodes_indexs;
            std::vector<int> ids; 
            for (int i = 0; i < tour_points_num; ++i) {
                ids.push_back(i);
            }
            //two-opt solver ..
            ROS_INFO("start 2-opt solver..");
            auto start_time = ros::WallTime::now();

            std::vector<double> gains;
            gains.push_back(0.0);
            for (int i = 1; i < tour_points_num; ++i) {
                gains.push_back(tour_points_gains_[tour_points_term[i]]);
            }
         
            double lambda = 3.0;
            std::vector<int> init_route = ids;
            Two_Opt two_opt_solve(init_route, gains, cost_matrix, lambda);
            two_opt_solve.solve();
            auto max_unity_way = two_opt_solve.best_route_;

            auto finish_time = ros::WallTime::now();
            double iteration_time = (finish_time - start_time).toSec();
            sum_two_opt_time = sum_two_opt_time + iteration_time;
            std::ofstream fout;
            fout.open(two_opt_time_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
            fout << solving_num_ << "\t" << iteration_time << "\t"
                 << sum_two_opt_time / solving_num_ << "s \t" << max_unity_way.size() - 1 << "\t"
                 << std::endl;
            fout.close();

            ROS_INFO(
                    "2-opt solver plan is finished, spent %.10f s, finish path node num is %zu, total searched ways size is %d",
                    iteration_time, max_unity_way.size(), two_opt_solve.searched_route_num_);
            ROS_INFO("the best route is:");
            for (auto &i:max_unity_way) {
                std::cout << i << "\t";
            }
            std::cout << std::endl;
            ROS_INFO("the best unity is %.10f", two_opt_solve.best_unity_);

            if (max_unity_way.size() > 1) {
                Path tsp_path;
                for (int i = 0; i < max_unity_way.size() - 1; i++) {
                    tsp_path.insert(tsp_path.end(),
                                    path_matrix[max_unity_way[i]][max_unity_way[i + 1]].begin(),
                                    path_matrix[max_unity_way[i]][max_unity_way[i + 1]].end());
                    path_segments_.push_back(path_matrix[max_unity_way[i]][max_unity_way[i + 1]]);
                }
                tsp_path_ = tsp_path;
                path_to_go_ = path_matrix[0][max_unity_way[1]];
                goal_point_ = tour_points_term.at(max_unity_way[1]);
                ROS_INFO("the goal point is x= %f, y=%f, z=%f", goal_point_.x(), goal_point_.y(), goal_point_.z());
                return true;
            } else {
                return false;
            }
        }
    }


    int RapidCoverPlanner::addCurrentPositionToGraph(const Point3D &current_position, graph::PlanGraph &graph) {
 
        ROS_INFO("add current position to the plan map");
        int current_point_id = graph.addVertex(current_position);

        if (current_point_id != 0) {
            double local_range = 2.0;
            Point3DQueue neighbor_vertexs;
            std::vector<int> neighbor_vertex_ids = graph.getNeighborVertexsIDs(graph.kd_tree_, current_position,
                                                                               local_range,
                                                                               neighbor_vertexs);
            for (const auto &item: neighbor_vertexs) {
                if (map_2d_manager_->inflate_map_.isCollisionFreeStraight(
                        Point2D(current_position.x(), current_position.y()),
                        Point2D(item.x(), item.y()))) {
                    int item_id;
                    if (plan_graph_.getPointId(item, item_id) && current_point_id != item_id) {
                        graph.addTwoWayEdge(current_point_id, item_id);
                        ROS_INFO("added a edge of the current point, end point coord z is %f ", item.z());
                    }
                }
            }
        }
        return current_point_id;
    }


    Path RapidCoverPlanner::getPathInGraph(const int &start_point_id, const int &end_point_id,
                                           const graph::PlanGraph &graph) {

        Path path;
        bool is_get_empty = false;
        std::vector<int> waypoints_ids;
        graph.getShortestPath(start_point_id, end_point_id, waypoints_ids, path);

        ROS_INFO("the path size is %zu", path.size());
        if (path.size() < 2) {
            is_get_empty = true;
        }


        return path;
    }

    Path RapidCoverPlanner::getPathInGridMap2D(const Point3D &start_point, const Point3D &end_point,
                                               const GridMap2D &grid_map_2d) {
        Point2D start_2d(start_point.x(), start_point.y());
        Point2D end_2d(end_point.x(), end_point.y());
        Path path_3d;
        bool is_get_empty = false;
        if (grid_map_2d.isInMapRange2D(start_2d) && grid_map_2d.getStatusInMap2D(start_2d) == Status2D::Free &&
            grid_map_2d.isInMapRange2D(end_2d) && grid_map_2d.getStatusInMap2D(end_2d) == Status2D::Free) {
            ROS_INFO("start search shortest path..");
            std::vector<Point2D> path_2d = grid_map_2d.getShortestPath(end_2d, start_2d);
            if (path_2d.size() < 2) {
                is_get_empty = true;
            }
            ROS_INFO("start optimal to straight..");
            auto optimal_path_2d = grid_map_2d.optimalToStraight(path_2d);
            for (const auto &point: optimal_path_2d) {
                path_3d.emplace_back(point.x(), point.y(), map_2d_manager_->current_pose_.position.z);
            }
        } else {
            ROS_INFO("start point or end point is out of grid map 2d or occupancy");
            path_3d.clear();
        }

        return path_3d;
    }


}