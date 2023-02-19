//
// Created by hjl on 2021/12/15.
//

#include "path_execution/path_execution.h"

namespace path_execution {

    PathExecution::PathExecution(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
            nh_(nh), nh_private_(nh_private), loop_num_(0),
            execute_server_(nh_private_, "vehicle_execute", boost::bind(&PathExecution::executeCallback, this, _1),
                            false),
            stop_move_server_(nh_private_, "vehicle_stop", boost::bind(&PathExecution::stopMoveCallback, this, _1),
                              false) {

        odom_sub_ = nh_.subscribe("odometry", 1, &PathExecution::odomCallback, this);
        terrain_map_sub_ = nh_.subscribe("terrain_map", 1, &PathExecution::terrainMapCallback, this);
        goal_pose_sub_ = nh_.subscribe("goal", 1, &PathExecution::goalCallback, this); 

        local_path_pub_ = nh_private_.advertise<nav_msgs::Path>("local_path", 1);
        way_pose_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("way_pose", 1);
        stop_move_pub_ = nh_private_.advertise<std_msgs::Bool>("stop_move", 1);
        occupancy_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("occupancy_map", 1);
        inflate_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("inflate_map", 1);
        executed_path_pub_ = nh_private_.advertise<nav_msgs::Path>("executed_path", 1);
        path_list_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("path_list", 1);
        smooth_path_pub_ = nh_private_.advertise<visualization_msgs::Marker>("local_smooth_path", 1);
        look_ahead_goal_pub_ = nh_private_.advertise<geometry_msgs::PointStamped>("look_ahead_goal", 1);

        nh_private_.param("global_frame", global_frame_, std::string("world"));
        nh_private_.param("local_frame", local_frame_, std::string("base_link"));
        nh_private_.param("cloud_voxel_size", cloud_voxel_size_, 0.05);
        downSizeFilter_.setLeafSize(cloud_voxel_size_, cloud_voxel_size_, cloud_voxel_size_);

        nh_private_.param("grid_size", grid_size_, 0.1);
        nh_private_.param("grid_x_num", grid_x_num_, 80);
        nh_private_.param("grid_y_num", grid_y_num_, 80);
        nh_private_.param("inflate_radius", inflate_radius_, 0.3);
        nh_private_.param("inflate_empty_radius", inflate_empty_radius_, 0.6);
        nh_private_.param("lower_z", lower_z_, 0.03);

        nh_private_.param("control_freq", control_freq_, 100.0);
        nh_private_.param("guide_dist", guide_dist_, 4.0);
        nh_private_.param("reach_dist_thres", reach_dist_thres_, 0.35);
        nh_private_.param("reach_rot_thres", reach_rot_thres_, 0.26);
        nh_private_.param("stop_vel_thres", stop_vel_thres_, 0.05);
        nh_private_.param("stop_rot_thres", stop_rot_thres_, 0.09);

        execute_server_.start();
        stop_move_server_.start();
        ROS_DEBUG("started server");

        // generate local path primitives
        double dis = 3.0;
        double angle = 60.0;
        double delta_angle = 3.0;

        path2d_list_.clear();
        for (double shift1 = -angle; shift1 <= angle; shift1 += delta_angle) {
            std::vector<double> path_start_R;
            for (double i = 0.0; i <= dis; i += 0.1) {
                path_start_R.push_back(i);
            }
            std::vector<double> path_start_shift;
            for (int i = 0; i < path_start_R.size(); ++i) {
                path_start_shift.push_back(shift1 / dis * path_start_R[i]);
            }
            Path2D path2d;
            for (int i = 0; i < path_start_R.size(); ++i) {
                path2d.emplace_back(path_start_R[i] * cos(path_start_shift[i] * M_PI / 180),
                                    path_start_R[i] * sin(path_start_shift[i] * M_PI / 180));
            }
            path2d_list_.push_back(path2d);
        }
        ROS_INFO("path execution constructed.");
    }

    void PathExecution::odomCallback(const nav_msgs::OdometryConstPtr &odom) {
        last_pose_ = current_pose_;
        current_pose_ = odom->pose.pose;

        last_odom_ = current_odom_;
        current_odom_ = *odom;
    }

    void PathExecution::terrainMapCallback(const traversability_analysis::TerrainMapConstPtr &terrain_map) {

        TerrainMap terrain_map_;

        for (const auto &item :terrain_map->grids) {
            terrain_map_.status.push_back(item.status);
        }

        if (terrain_map->header.frame_id == global_frame_) {  
            terrain_map_.frame_id = terrain_map->header.frame_id;
            terrain_map_.min_x = terrain_map->min_x;
            terrain_map_.min_y = terrain_map->min_y;
            terrain_map_.z_value = terrain_map->z_value;
        } else {
            geometry_msgs::PoseStamped min_pose;
            min_pose.header = terrain_map->header;
            min_pose.pose.position.x = terrain_map->min_x;
            min_pose.pose.position.y = terrain_map->min_y;
            min_pose.pose.position.z = terrain_map->z_value;
            min_pose.pose.orientation.w = 1.0;
            try {
                tf_listener_.transformPose(global_frame_, min_pose, min_pose);
            }
            catch (const tf::TransformException &ex) {
                ROS_WARN_THROTTLE(1, " get terrain map acquire---        %s ", ex.what());
                return;
            }
            terrain_map_.frame_id = global_frame_;
            terrain_map_.min_x = min_pose.pose.position.x;
            terrain_map_.min_y = min_pose.pose.position.y;
            terrain_map_.z_value = min_pose.pose.position.z;
        }

        terrain_map_.grid_size = terrain_map->grid_size;
        terrain_map_.grid_width_num = terrain_map->grid_width_num;
        terrain_map_.max_x = terrain_map_.min_x + terrain_map_.grid_size * terrain_map_.grid_width_num;
        terrain_map_.max_y = terrain_map_.min_y + terrain_map_.grid_size * terrain_map_.grid_width_num;


        occupancy_map_mutex_.lock();

        occupancy_map_.clearMap(); 

        double x_length = terrain_map_.max_x-terrain_map_.min_x;
        double y_length =  terrain_map_.max_y-terrain_map_.min_y;
        occupancy_map_.initialize(grid_size_, grid_x_num_, grid_y_num_, Status::unknown); 

        Eigen::Vector2d center_point(current_pose_.position.x,
                                     current_pose_.position.y); 

        occupancy_map_.setMapCenterAndBoundary(center_point); 

        setOccupancyMap(occupancy_map_, terrain_map_); 

        occupancy_map_.inflateOccupancyMap(inflate_radius_, inflate_empty_radius_); 

        clearRobotNeighbor(center_point, inflate_empty_radius_ +
                                         occupancy_map_.grid_size_); 

        visualization_msgs::MarkerArray occupancy_markers = occupancy_map_.generateMapMarkers(
                occupancy_map_.occupancy_map_, current_pose_);
        occupancy_pub_.publish(occupancy_markers);
        visualization_msgs::MarkerArray inflate_markers = occupancy_map_.generateMapMarkers(
                occupancy_map_.inflate_map_, current_pose_);
        inflate_pub_.publish(inflate_markers);

        occupancy_map_mutex_.unlock();
    }

    void PathExecution::clearRobotNeighbor(const Point2D &center_point, double clear_radius) {
        if (occupancy_map_.getStatusInFlateMap(center_point) != Status::free) { 
             
            Index2D center_index = occupancy_map_.getIndexInMap(center_point);

            double min_distance = 1000000;
            Index2D min_index = center_index;
            Point2D point;
            for (double x = center_point.x() - clear_radius;
                 x < center_point.x() + clear_radius; x += occupancy_map_.grid_size_) {
                for (double y = center_point.y() - clear_radius;
                     y < center_point.y() + clear_radius; y += occupancy_map_.grid_size_) {
                    point.x() = x;
                    point.y() = y;
                    if (occupancy_map_.getStatusInFlateMap(point) == Status::free) {
                        if ((center_point - point).norm() < min_distance) {
                            min_distance = (center_point - point).norm();
                            min_index = occupancy_map_.getIndexInMap(point);
                        }
                    }
                }
            }

            occupancy_map_.inflate_map_[center_index.x()][center_index.y()].status = Status::free;
            if (min_index != center_index) {
                Point2D target = occupancy_map_.getGridCenter(center_index);
                Point2D source = occupancy_map_.getGridCenter(min_index);
                double end_distance = (target - source).norm();
                Eigen::Vector2d directory = (target - source).normalized();
                double step_length = grid_size_ / 4; 
                int step = 0;
                Point2D inner_point = source + directory * step_length * step;
                while ((inner_point - source).norm() < end_distance) {
                    occupancy_map_.inflate_map_[occupancy_map_.getIndexInMap(
                            inner_point).x()][occupancy_map_.getIndexInMap(inner_point).y()].status = Status::free;
                    step++;
                    inner_point = source + directory * step_length * step;
                }
            }

        }

    }

    void PathExecution::setOccupancyMap(OccupancyMap2D &occupancy_map, TerrainMap &terrain_map) {
        Point2D grid_center;
        for (int i = 0; i < occupancy_map.grid_x_num_; ++i) {
            for (int j = 0; j < occupancy_map.grid_y_num_; ++j) {
                grid_center = occupancy_map.getGridCenter(i, j);
                if (terrain_map.isInTerrainMap(grid_center)) {
                    if (terrain_map.status[terrain_map.getGridID(grid_center)] == 0)
                        occupancy_map.setFree(i, j);
                    if (terrain_map.status[terrain_map.getGridID(grid_center)] == 1)
                        occupancy_map.setOccupied(i, j);
                    if (terrain_map.status[terrain_map.getGridID(grid_center)] == 2)
                        occupancy_map.setEmpty(i, j);
                    if (terrain_map.status[terrain_map.getGridID(grid_center)] == 3)
                        occupancy_map.setUnknown(i, j);
                } else {
                    occupancy_map.setUnknown(i, j);
                }
            }
        }
    }


    void PathExecution::stopMoveCallback(const control_planner_interface::VehicleExecuteGoalConstPtr &stop_goal) {
        //wait until we're stopped before returning success
        ROS_INFO("start stop ...");

        ros::Rate r(10.0);
        geometry_msgs::Twist stop_twist;
         
        ros::Time start_time = ros::Time::now();
        while ((ros::Time::now() - start_time).toSec() > 2.0 &&
               !isStopped() &&
               (ros::Time::now() - start_time).toSec() < 7.0) {
            std_msgs::Bool stop;
            stop.data = true;
            stop_move_pub_.publish(stop);
            r.sleep();
        }
        if (isStopped()) {
            ROS_INFO("stop move finished.");
            stop_move_server_.setSucceeded();
        } else {
            ROS_INFO("the robot is not stop");
            stop_move_server_.setAborted();
        }
    }

    bool PathExecution::isStopped() {
        double dist = sqrt(pow(current_pose_.position.x - last_pose_.position.x, 2) +
                           pow(current_pose_.position.y - last_pose_.position.y, 2));

        double yaw_delta = fabs(tf::getYaw(current_pose_.orientation) - tf::getYaw(last_pose_.orientation));
        if (yaw_delta > M_PI) { 
            yaw_delta = 2 * M_PI - yaw_delta;
        }
        double odom_delta_t = (current_odom_.header.stamp - last_odom_.header.stamp).toSec();
        if ((dist / odom_delta_t) < stop_vel_thres_ && (yaw_delta / odom_delta_t) < stop_rot_thres_) {
            ROS_INFO("the robot is stopped.");
            return true;
        } else {
            return false;
        }
    }

    void PathExecution::executeCallback(const control_planner_interface::VehicleExecuteGoalConstPtr &executed_path) {
        ROS_WARN("get a path, executing .... ");
        if (!setPathToFollow(executed_path->paths)) {
            //ABORT and SHUTDOWN COSTMAPS
            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
            execute_server_.setAborted(control_planner_interface::VehicleExecuteResult(),
                                       "Failed to pass global plan to the controller.");
            return;
        }
//
        if (!global_path_to_follow_.empty()) {

            bool success = false;
            loop_num_ = 0;

            success = lookAheadPointControlLoop();
            //based on the control loop's exit status... we'll set our goal status
            if (success) {
                //wait until we're stopped before returning success
                execute_server_.setSucceeded();
            } else {
                //if a preempt was requested... the control loop exits for that reason
                if (execute_server_.isPreemptRequested())
                    execute_server_.setPreempted(); 
                else
                    execute_server_.setAborted();
            }
            return;
        } else {
            ROS_INFO("the rest path size is 0,executed succeed.");
            execute_server_.setSucceeded();
        }
    }

    bool
    PathExecution::setPathToFollow(const std::vector<control_planner_interface::Path> &path_segments) {

        if (path_segments.empty()) {
            ROS_WARN("the received path_segments is emtpy..");
            return false;
        }

        path_segments_.clear();
        global_path_to_follow_.clear();
        Path path;
        for (int i = 0; i < path_segments.size(); i++) {
            ROS_INFO("this path segment size is %zu ", path_segments[i].path.size());
            path.insert(path.end(), path_segments[i].path.begin(), path_segments[i].path.end());
            Path interpolate_segment = interpolatePath(path_segments[i].path);
            path_segments_.push_back(interpolate_segment);
            global_path_to_follow_.insert(global_path_to_follow_.end(), interpolate_segment.begin(),
                                          interpolate_segment.end());
        }

        ROS_INFO(" the followed global path size is %zu", path.size());
        ROS_INFO("global path to be executed size: %lu", global_path_to_follow_.size());
        publishPath(global_path_to_follow_, executed_path_pub_);

        return true;
    }

    Path PathExecution::interpolatePath(const Path &path) const {
        ROS_INFO("interpolate the path..");
        Path interpolated_path;

        if (path.size() >= 2) {
            for (int i = 0; i < (path.size() - 1); ++i) {
                interpolated_path.push_back(path[i]);

                // Interpolate each segment.
                tf::Vector3 start(path[i].position.x, path[i].position.y, path[i].position.z);
                tf::Vector3 end(path[i + 1].position.x, path[i + 1].position.y, path[i + 1].position.z);
                double distance = (end - start).length();
                double step_size = grid_size_;
                tf::Vector3 inner_point = start + (end - start).normalized() * step_size;

                double yaw = std::atan2(end.y() - start.y(), end.x() - start.x()); 
                tf::Quaternion quat(tf::Vector3(0, 0, 1), yaw); 

                while ((inner_point - start).length() < distance) {
                    tf::Pose poseTF(quat, inner_point);
                    geometry_msgs::Pose pose;
                    tf::poseTFToMsg(poseTF, pose);
                    interpolated_path.push_back(pose);
                    inner_point += (end - start).normalized() * step_size;
                }
            }

            interpolated_path.push_back(path.back());

            return interpolated_path;
        } else {
            return path;
        }
    }

    void PathExecution::publishPath(const Path &path, const ros::Publisher &pub) {
        // given an empty path we won't do anything
        if (path.empty())
            return;

        // create a path message
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());
        gui_path.header.frame_id = global_frame_;
        gui_path.header.stamp = ros::Time::now();

        // extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i].pose = path[i];
            gui_path.poses[i].header = gui_path.header;
        }
        pub.publish(gui_path);
    }

    bool PathExecution::lookAheadPointControlLoop() {
        loop_num_++;
        ROS_INFO("start control loop %d...", loop_num_);
        ROS_INFO("the will be executed point num of the interpolated path is %zu", global_path_to_follow_.size());

        if (global_path_to_follow_.empty())
            return true;

        if (control_freq_ == 0.0)
            return false;
        ros::Rate r(control_freq_);
  
        int ind = 0;
        current_waypose_id_ = 0;
        while (ind < path_segments_.size()) {
            if (execute_server_.isPreemptRequested()) {
                return false;
            }

            Path current_path = path_segments_[ind];
            int current_waypoint_id = 0;
            occupancy_map_mutex_.lock();
            for (int i = static_cast<int>(current_path.size() - 1); i >= 0; --i) {
                Point2D point(current_path[i].position.x, current_path[i].position.y);
                if (occupancy_map_.getStatusInFlateMap(point) == Status::free) { 
                    std::vector<Point2D> path_2d = makeLocalPlan(current_path[i], current_pose_, occupancy_map_);
                    if (path_2d.size() > 1) {  
                        ROS_INFO("get local current_path to the point %d currently ", i);
                        current_waypoint_id = i;

                        geometry_msgs::PoseStamped way_pose;
                        way_pose.header.frame_id = global_frame_;
                        way_pose.header.stamp = ros::Time::now();
                        way_pose.pose = current_path[i];
                        way_pose_pub_.publish(way_pose);

                        geometry_msgs::Pose end_pose;
                        end_pose.position.x = path_2d.back().x();
                        end_pose.position.y = path_2d.back().y();
                        end_pose.orientation.w = 1.0;
                        Path path_msg = generatePath(path_2d, end_pose);
                        publishPath(path_msg, local_path_pub_);
  
                        Point2D current_position(current_pose_.position.x, current_pose_.position.y);
                        geometry_msgs::PointStamped look_ahead_point;
                        look_ahead_point.header.frame_id = global_frame_;
                        look_ahead_point.header.stamp = ros::Time::now();
                        Point2D look_ahead_goal;
                        for (int j = static_cast<int>(path_2d.size() - 1); j >= 0; --j) {
                            if (occupancy_map_.isCollisionFreeStraight(current_position, path_2d[j])) { 
                                double distance = sqrt(pow(path_2d[j].x() - current_position.x(), 2) +
                                                       pow(path_2d[j].y() - current_position.y(), 2));

                                if (j != (path_2d.size() - 1) && distance < 1.0) {
                                     
                                    look_ahead_goal = path_2d[j] + (path_2d[j] - current_position).normalized() * 1.0;

                                    if(distance < occupancy_map_.grid_size_){
                                        look_ahead_goal = path_2d[j+1];
                                    }
                                } else {
                                    look_ahead_goal = path_2d[j];
                                }
                                break;
                            }
                        }
                        look_ahead_point.point.x = look_ahead_goal.x();
                        look_ahead_point.point.y = look_ahead_goal.y();
                        look_ahead_point.point.z = current_pose_.position.z;
                        look_ahead_goal_pub_.publish(look_ahead_point);
                        break;
                    }
                }
            }
            occupancy_map_mutex_.unlock();

            ros::spinOnce();
            r.sleep();

            if (current_waypoint_id == current_path.size() - 1) {
                double dist = sqrt(
                        std::pow(current_path[current_waypoint_id].position.x - current_pose_.position.x, 2) +
                        std::pow(current_path[current_waypoint_id].position.y - current_pose_.position.y, 2));

                if (dist < reach_dist_thres_) {
                    if (ind == (path_segments_.size() - 1)) { 
                        return true;
                    } else {
                        ind++;
                    }
                }
            }
        }

        return true;
    }


    std::vector<Point2D>
    PathExecution::makeLocalPlan(geometry_msgs::Pose &goal, geometry_msgs::Pose &start, OccupancyMap2D &map) {

        auto path = getShortestPath(goal, start, map);
        auto pruned_path = optimalToStraight(path, map); 
        return pruned_path;
    }

    std::vector<Point2D>
    PathExecution::getShortestPath(geometry_msgs::Pose &goal, geometry_msgs::Pose &start, OccupancyMap2D &map) {
        Point2D start_2d(start.position.x, start.position.y);
        Point2D goal_2d(goal.position.x, goal.position.y);

        int start_grid_id = map.Index2DToGridId(map.getIndexInMap(start_2d));
        int goal_grid_id = map.Index2DToGridId(map.getIndexInMap(goal_2d));

        std::vector<Eigen::Vector2d> path_points;
        path_points.clear();
        if (start_grid_id == goal_grid_id) { 
            path_points.clear();
            path_points.push_back(start_2d);
            return path_points;
        }

        typedef pair<double, int> iPair; // Vertices are represented by their index in the graph.vertices list
        priority_queue<iPair, vector<iPair>, greater<iPair>> pq; // Priority queue of vertices
        vector<double> dist(grid_x_num_ * grid_y_num_, INFINITY);// Vector of distances
        vector<double> estimation(grid_x_num_ * grid_y_num_, INFINITY);//Vector of G + H .
        const int INF = 0x3f3f3f3f; // integer infinity
        vector<int> backpointers(grid_x_num_ * grid_y_num_, INF);// Vector of backpointers
        vector<bool> in_pq(grid_x_num_ * grid_y_num_, false);


        //**************A star **************
        // Add the start vertex
        dist[start_grid_id] = 0;
        estimation[start_grid_id] = (start_2d - goal_2d).norm();
        pq.push(make_pair(estimation[start_grid_id], start_grid_id));
        in_pq[start_grid_id] = true;
        int u;
        int v;
        while (!pq.empty()) {        // Loop until priority queue is empty
            u = pq.top().second;
            pq.pop();  // Pop the minimum distance vertex
            in_pq[u] = false;
            if (u == goal_grid_id) {            // Early termination
                break;
            }
            for (int i = -1; i <= 1; ++i) {            // Get all adjacent vertices
                for (int j = -1; j <= 1; ++j) {
                    if (!(i == 0 && j == 0) &&
                        (u / grid_y_num_ + i) >= 0 && (u / grid_y_num_ + i) < grid_x_num_ &&
                        (u % grid_y_num_ + j) >= 0 && (u % grid_y_num_ + j) < grid_y_num_) {
                        // Get vertex label and weight of current adjacent edge of u
                        v = (u / grid_y_num_ + i) * grid_y_num_ + (u % grid_y_num_ + j);
                        if (map.inflate_map_[u / grid_y_num_ + i][u % grid_y_num_ + j].status == Status::free) {
                            // If there is a shorter path to v through u
                            if (dist[v] > dist[u] + grid_size_ * sqrt(i * i + j * j)) {
                                // Updating distance of v
                                dist[v] = dist[u] + grid_size_ * sqrt(i * i + j * j);
                                estimation[v] =
                                        dist[v] + (map.getGridCenter(v) - map.getGridCenter(goal_grid_id)).norm();
                                backpointers[v] = u;
                                if (!in_pq[v]) {
                                    pq.push(make_pair(estimation[v], v));
                                    in_pq[v] = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        // Backtrack to find path
        std::vector<int> path;
        vector<int> reverse_path;
        int current = goal_grid_id;
        if (backpointers[current] == INF) {// no path found
            ROS_WARN("no path found");
            path.clear();
        } else {
            // path found
            while (current != INF) {
                reverse_path.push_back(current);
                current = backpointers[current];
            }

            // Reverse the path (constructing it this way since vector is more efficient
            // at push_back than insert[0])
            path.clear();
            for (int i = reverse_path.size() - 1; i >= 0; --i) {
                path.push_back(reverse_path[i]);
            }

            path_points.clear();
            path_points.push_back(start_2d);//起点和终点放入前后.其他格子用网格中心.
            for (int i = 1; i < path.size() - 1; ++i) {
                path_points.push_back(map.getGridCenter(path[i]));
            }
            path_points.push_back(goal_2d);
        }

//        ROS_INFO(" shortest path get, size is %zu", path_points.size());
        return path_points;
    }

    std::vector<Point2D> PathExecution::optimalToStraight(std::vector<Point2D> &path, OccupancyMap2D &map) {
        if (path.size() > 2) {
            std::vector<Point2D> pruned_path;
            std::vector<int> control_point_ids;
            int inner_idx = 0;
            int control_point_id = inner_idx;
            control_point_ids.push_back(control_point_id);
            while (inner_idx < path.size() - 1) {
                control_point_id = inner_idx;
                for (int i = inner_idx + 1; i < path.size(); ++i) {
                    if (map.isCollisionFreeStraight(path[inner_idx], path[i])) { 
                        control_point_id = i;
                    }
                }
                if (control_point_id == inner_idx) { 
                    control_point_id = inner_idx + 1; 
                }
                control_point_ids.push_back(control_point_id);
                inner_idx = control_point_id;
            }

            for (int i = 0; i < control_point_ids.size(); ++i) {
                pruned_path.push_back(path[control_point_ids[i]]);
            }
            return pruned_path;
        } else {
            return path;
        }

    }


    Path PathExecution::generatePath(std::vector<Point2D> &path_2d, geometry_msgs::Pose &end_pose) {
        std::vector<geometry_msgs::Pose> path;

        if (!path_2d.empty()) {
            for (int i = 0; i < path_2d.size(); i++) {
                geometry_msgs::Pose way_pose;
                way_pose.position.x = path_2d[i].x();
                way_pose.position.y = path_2d[i].y();
                way_pose.position.z = current_pose_.position.z;

                tf::Vector3 axis(0, 0, 1); 
                float angle;
                if (path_2d.size() == 1) {
                    angle = std::atan2(path_2d[i].y() - current_pose_.position.y,
                                       path_2d[i].x() - current_pose_.position.x); 
                } else if ((i + 1) < path_2d.size()) {
                    angle = std::atan2((path_2d[i + 1].y() - path_2d[i].y()),
                                       (path_2d[i + 1].x() - path_2d[i].x())); 
                } else {
                    angle = std::atan2((path_2d[i].y() - path_2d[i - 1].y()),
                                       (path_2d[i].x() - path_2d[i - 1].x())); 
                }
                tf::Quaternion quaternion(axis, angle);
                way_pose.orientation.x = quaternion.getX();
                way_pose.orientation.y = quaternion.getY();
                way_pose.orientation.z = quaternion.getZ();
                way_pose.orientation.w = quaternion.getW();

                path.push_back(way_pose);
            }

             
            path.push_back(end_pose);
        } else {
            ROS_WARN("the path_2d is empty, generate path failed");
        }

        return path;
    }

    void PathExecution::goalCallback(const geometry_msgs::PoseStampedConstPtr &goal) {
        ROS_INFO("get a goal..");
        geometry_msgs::PoseStamped current_goal = *goal;
        if (goal->header.frame_id != global_frame_) {
            tf_listener_.transformPose(global_frame_, current_goal, current_goal);
        }

        if (occupancy_map_.isInMapRange2D(Point2D(current_goal.pose.position.x, current_goal.pose.position.y))) {
            geometry_msgs::PoseStamped way_pose;
            way_pose.header.frame_id = global_frame_;
            way_pose.header.stamp = ros::Time::now();
            way_pose.pose = current_goal.pose;
            way_pose_pub_.publish(way_pose); 

            double dist = sqrt(std::pow(current_goal.pose.position.x - current_pose_.position.x, 2) +
                               std::pow(current_goal.pose.position.y - current_pose_.position.y, 2));
            while (dist > reach_dist_thres_) {
                std::vector<Point2D> path_2d = makeLocalPlan(current_goal.pose, current_pose_, occupancy_map_);

                if (!path_2d.empty()) {  
                    Path path = generatePath(path_2d, current_goal.pose);
                    publishPath(path, local_path_pub_);
                    ROS_INFO("publish a path to go.");

                    Point2D look_ahead_goal;
                    Point2D current_position(current_pose_.position.x, current_pose_.position.y);

                    if (path_2d.size() > 2) {
                        look_ahead_goal =
                                path_2d[1] + (path_2d[1] - current_position).normalized() * 1.0; 
                    }

                    if (path_2d.size() == 2) {
                        look_ahead_goal = path_2d[1];
                    }
                    geometry_msgs::PointStamped look_ahead_point;
                    look_ahead_point.header.frame_id = global_frame_;
                    look_ahead_point.header.stamp = ros::Time::now();
                    look_ahead_point.point.x = look_ahead_goal.x();
                    look_ahead_point.point.y = look_ahead_goal.y();
                    look_ahead_point.point.z = current_pose_.position.z;
                    look_ahead_goal_pub_.publish(look_ahead_point);
                }

                dist = sqrt(std::pow(current_goal.pose.position.x - current_pose_.position.x, 2) +
                            std::pow(current_goal.pose.position.y - current_pose_.position.y, 2));

                ros::spinOnce();
                ros::Rate(control_freq_).sleep();
            }

        } else {
            ROS_WARN("the goal is out of map range, please set a suitable goal.");
        }
    }

    visualization_msgs::MarkerArray
    PathExecution::generatePath2DListMarker(const vector<Path2D> &path_list) const {
        visualization_msgs::MarkerArray path_list_markers;
        path_list_markers.markers.resize(path_list.size());
        for (int i = 0; i < path_list.size(); ++i) {
            visualization_msgs::Marker path_marker;

            path_marker.header.frame_id = global_frame_;
            path_marker.header.stamp = ros::Time::now();
            path_marker.ns = "path";
            path_marker.action = visualization_msgs::Marker::ADD;
            path_marker.pose.orientation.w = 1.0;

            //setting id for each marker
            path_marker.id = i;

            //defining types
            path_marker.type = visualization_msgs::Marker::LINE_STRIP; 

            //setting scale
            path_marker.scale.x = 0.05;
            path_marker.scale.y = 0.05;
            path_marker.scale.z = 0.05;

            path_marker.color.r = 1.0f;
            path_marker.color.g = 1.0f;
            path_marker.color.b = 0.0f;
            path_marker.color.a = 1.0f;

            //assignment
            geometry_msgs::Point point;
            for (const auto &node: path_list[i]) {
                point.x = node.x();
                point.y = node.y();
                point.z = current_pose_.position.z;
                path_marker.points.push_back(point);
            }

            path_list_markers.markers[i] = path_marker;
        }

        return path_list_markers;
    }

    visualization_msgs::Marker PathExecution::generateSmoothPath2DMarker(const Path2D &path) const {
        visualization_msgs::Marker path_marker;

        //init headers
        path_marker.header.frame_id = global_frame_;
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "path";
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;

        //setting id for each marker
        path_marker.id = 0;

        //defining types
        path_marker.type = visualization_msgs::Marker::LINE_STRIP; 

        //setting scale
        path_marker.scale.x = 0.1;
        path_marker.scale.y = 0.1;
        path_marker.scale.z = 0.1;

        path_marker.color.r = 1.0f;
        path_marker.color.g = 0.0f;
        path_marker.color.b = 1.0f;
        path_marker.color.a = 1.0f;

        //assignment
        geometry_msgs::Point point;
        for (const auto &node: path) {
            point.x = node.x();
            point.y = node.y();
            point.z = current_pose_.position.z;
            path_marker.points.push_back(point);
        }

        return path_marker;
    }


}


