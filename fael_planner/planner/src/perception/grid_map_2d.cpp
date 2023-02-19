//
// Created by hjl on 2021/11/13.
//

#include "perception/grid_map_2d.h"

namespace perception {


    GridMap2D::GridMap2D() {

    }

    GridMap2D::GridMap2D(double &grid_size, int &grid_x_num, int &grid_y_num, Status2D init_status) {
        initialize(grid_size, grid_x_num, grid_x_num, init_status);
    }

    GridMap2D::GridMap2D(double &grid_size, double &x_length, double &y_length, Status2D init_status) {
        initialize(grid_size, x_length, y_length, init_status);
    }

    void GridMap2D::initialize(double &grid_size, int &grid_x_num, int &grid_y_num, Status2D init_status) {
        grid_size_ = grid_size;
        grid_x_num_ = grid_x_num;
        grid_y_num_ = grid_y_num;

        x_length_ = grid_size * grid_x_num;
        y_length_ = grid_size * grid_y_num;

        grids_ = std::vector<std::vector<Grid2D>>(grid_x_num_, std::vector<Grid2D>(grid_y_num_, Grid2D(init_status)));
    }

    void GridMap2D::initialize(double &grid_size, double &x_length, double &y_length, Status2D init_status) {
        grid_size_ = grid_size;
        x_length_ = x_length;
        y_length_ = y_length;

        grid_x_num_ = std::ceil(x_length / grid_size);
        grid_y_num_ = std::ceil(y_length / grid_size);

        grids_ = std::vector<std::vector<Grid2D>>(grid_x_num_, std::vector<Grid2D>(grid_y_num_, Grid2D(init_status)));
    }

    void GridMap2D::setMapCenterAndBoundary(Point2D &center_point) {
        center_point_ = center_point;
        min_x_ = center_point.x() - x_length_ / 2;
        max_x_ = center_point.x() + x_length_ / 2;
        min_y_ = center_point.y() - y_length_ / 2;
        max_y_ = center_point.y() + y_length_ / 2;
    }

    void GridMap2D::resetMapStatus(Status2D status) {
        for (auto &item: grids_) {
            for (auto &grid: item) {
                grid.status = status;
            }
        }
    }

    Ray2D GridMap2D::castRay(Point2D &start, Point2D &end) {
        Ray2D ray;
        Index2D start_sub = getIndexInMap2D(start);
        Index2D end_sub = getIndexInMap2D(end);
        Index2D diff_sub = end_sub - start_sub;
        double max_dist = diff_sub.squaredNorm();
        int step_x = signum(diff_sub.x());
        int step_y = signum(diff_sub.y());
        double t_max_x = step_x == 0 ? DBL_MAX : intbound(start_sub.x(), diff_sub.x());
        double t_max_y = step_y == 0 ? DBL_MAX : intbound(start_sub.y(), diff_sub.y());
        double t_delta_x = step_x == 0 ? DBL_MAX : (double) step_x / (double) diff_sub.x();
        double t_delta_y = step_y == 0 ? DBL_MAX : (double) step_y / (double) diff_sub.y();
        double dist;
        Index2D cur_sub = start_sub;

        while (isInMapRange2D(cur_sub)) {
            ray.push_back(cur_sub);
            dist = (cur_sub - start_sub).squaredNorm();
            if (cur_sub == end_sub || dist > max_dist || getStatusInMap2D(cur_sub) != Status2D::Free) {
                return ray;
            }
            if (t_max_x < t_max_y) {
                cur_sub.x() += step_x;
                t_max_x += t_delta_x;
            } else {
                cur_sub.y() += step_y;
                t_max_y += t_delta_y;
            }
        }
        return ray;
    }

    std::vector<Index2D> GridMap2D::getAllOccupiedGrids() {
        std::vector<Index2D> occupied_grids;
        for (int i = 0; i < grid_x_num_; ++i) {
            for (int j = 0; j < grid_y_num_; ++j) {
                if (grids_[i][j].status == Status2D::Occupied) {
                    occupied_grids.emplace_back(i, j);
                }
            }
        }
        return occupied_grids;
    }

    void GridMap2D::inflateGridMap2D(const double &inflate_radius,const double &inflate_empty_radius) {
        auto inflate_grids_ = grids_;
        int increment = std::ceil(abs(inflate_radius / grid_size_));
        int increment_empty = std::ceil(abs(inflate_empty_radius / grid_size_));
        for (int I = 0; I < grid_x_num_; ++I) {
            for (int J = 0; J < grid_y_num_; ++J) {
                if (grids_[I][J].status == Status2D::Occupied) {
                    for (int i = -increment; i <= increment; ++i) {
                        for (int j = -increment; j <= increment; ++j) {
                            if (I + i >= 0 && I + i < grid_x_num_ && J + j >= 0 && J + j < grid_y_num_) {
                                inflate_grids_[I + i][J + j].status = Status2D::Occupied;
                            }
                        }
                    }
                }

                if (grids_[I][J].status == Status2D::Empty) {
                    for (int i = -increment_empty; i <= increment_empty; ++i) {
                        for (int j = -increment_empty; j <= increment_empty; ++j) {
                            if (I + i >= 0 && I + i < grid_x_num_ && J + j >= 0 && J + j < grid_y_num_) {
                                inflate_grids_[I + i][J + j].status = Status2D::Empty;
                            }
                        }
                    }
                }
            }
        }
        grids_ = inflate_grids_;
    }

    visualization_msgs::MarkerArray
    GridMap2D::generateMapMarkers(std::vector<std::vector<Grid2D>> &grid_map, geometry_msgs::Pose &current_pose) {

        visualization_msgs::Marker free;
        visualization_msgs::Marker occupied;
        visualization_msgs::Marker empty;
        visualization_msgs::Marker unknow;
        visualization_msgs::Marker center_point;

        empty.header.frame_id = occupied.header.frame_id = free.header.frame_id = unknow.header.frame_id = center_point.header.frame_id = "world";
        empty.header.stamp = occupied.header.stamp = free.header.stamp = unknow.header.stamp = center_point.header.stamp = ros::Time::now();
        occupied.ns = "occupied";
        free.ns = "free";
        empty.ns = "empty";
        unknow.ns = "unknown";
        center_point.ns = "center";
        empty.action = occupied.action = free.action = unknow.action = center_point.action = visualization_msgs::Marker::ADD;
        empty.pose.orientation.w = occupied.pose.orientation.w = free.pose.orientation.w = unknow.pose.orientation.w = center_point.pose.orientation.w = 1.0;

        free.id = 0;
        free.type = visualization_msgs::Marker::CUBE_LIST;

        occupied.id = 1;
        occupied.type = visualization_msgs::Marker::CUBE_LIST;

        empty.id = 2;
        empty.type = visualization_msgs::Marker::CUBE_LIST;

        unknow.id = 3;
        unknow.type = visualization_msgs::Marker::CUBE_LIST;

        center_point.id = 4;
        center_point.type = visualization_msgs::Marker::SPHERE_LIST;

        //setting scale
        free.scale.x = grid_size_;
        free.scale.y = grid_size_;
        free.scale.z = 0.01;

        //assigning colors
        free.color.r = 0.5f;
        free.color.g = 1.0f;
        free.color.b = 0.5f;
        free.color.a = 0.5f;

        occupied.scale.x = grid_size_;
        occupied.scale.y = grid_size_;
        occupied.scale.z = 0.01;

        occupied.color.r = 0.1f;
        occupied.color.g = 0.5f;
        occupied.color.b = 0.5f;
        occupied.color.a = 1.0f;

        empty.scale.x = grid_size_;
        empty.scale.y = grid_size_;
        empty.scale.z = 0.01;

        empty.color.r = 0.5f;
        empty.color.g = 0.5f;
        empty.color.b = 0.5f;
        empty.color.a = 0.75f;

        unknow.scale.x = grid_size_;
        unknow.scale.y = grid_size_;
        unknow.scale.z = 0.01;

        unknow.color.r = 1.0f;
        unknow.color.g = 0.1f;
        unknow.color.b = 0.1f;
        unknow.color.a = 0.5f;

        center_point.scale.x = grid_size_;
        center_point.scale.y = grid_size_;
        center_point.scale.z = grid_size_;
        center_point.color.r = 1.0f;
        center_point.color.a = 1.0f;

        geometry_msgs::Point point;
        point.x = center_point_.x();
        point.y = center_point_.y();
        point.z = current_pose.position.z;
        center_point.points.push_back(point);

        for (int i = 0; i < grid_x_num_; i++) {
            for (int j = 0; j < grid_y_num_; j++) {
                point.x = min_x_ + grid_size_ / 2 + i * grid_size_;
                point.y = min_y_ + grid_size_ / 2 + j * grid_size_;
                point.z = current_pose.position.z;
                if (grid_map[i][j].status == Status2D::Free) {
                    free.points.push_back(point);
                }
                if (grid_map[i][j].status == Status2D::Occupied) {
                    occupied.points.push_back(point);
                }
                if (grid_map[i][j].status == Status2D::Empty) {
                    empty.points.push_back(point);
                }
                if (grid_map[i][j].status == Status2D::Unknown) {
                    unknow.points.push_back(point);
                }
            }
        }

        visualization_msgs::MarkerArray grid_map_markers;
        grid_map_markers.markers.resize(5);
        grid_map_markers.markers[0] = free;
        grid_map_markers.markers[1] = occupied;
        grid_map_markers.markers[2] = empty;
        grid_map_markers.markers[3] = unknow;
        grid_map_markers.markers[4] = center_point;

        return grid_map_markers;
    }

    std::vector<Point2D> GridMap2D::getShortestPath(const Point2D &goal, const Point2D &start) const {
        std::vector<Eigen::Vector2d> path_points;
        path_points.clear();

        if (isInMapRange2D(start) && isInMapRange2D(goal)) {
            int start_grid_id = Index2DToGridId(getIndexInMap2D(start));
            int goal_grid_id = Index2DToGridId(getIndexInMap2D(goal));

            if (start_grid_id == goal_grid_id) {//starting point is the end point
                std::cout << "start point = end point, only return the start point or end point as a path" << std::endl;
                path_points.clear();
                path_points.push_back(start);
                return path_points;
            }

            typedef std::pair<double, int> iPair; // Vertices are represented by their index in the graph.vertices list
            std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq; // Priority queue of vertices
            std::vector<double> dist(grid_x_num_ * grid_y_num_, INFINITY);// Vector of distances
            std::vector<double> estimation(grid_x_num_ * grid_y_num_, INFINITY);//Vector of G + H .
            const int INF = 0x3f3f3f3f; // integer infinity
            std::vector<int> backpointers(grid_x_num_ * grid_y_num_, INF);// Vector of backpointers
            std::vector<bool> in_pq(grid_x_num_ * grid_y_num_, false);

            // ******* A star **********
            // Add the start vertex
            dist[start_grid_id] = 0;
            estimation[start_grid_id] = (start - goal).norm();
            pq.push(std::make_pair(estimation[start_grid_id], start_grid_id));
            in_pq[start_grid_id] = true;

            std::cout<<"A* search.."<<std::endl;
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

                            if (grids_[u / grid_y_num_ + i][u % grid_y_num_ + j].status == Status2D::Free) {
                                // If there is a shorter path to v through u
                                if (dist[v] > dist[u] + grid_size_ * sqrt(i * i + j * j)) {
                                    // Updating distance of v
                                    dist[v] = dist[u] + grid_size_ * sqrt(i * i + j * j);
                                    estimation[v] = dist[v] + (getGridCenter(v) - getGridCenter(goal_grid_id)).norm();
                                    backpointers[v] = u;
                                    if (!in_pq[v]) {
                                        pq.push(std::make_pair(estimation[v], v));
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
            std::vector<int> reverse_path;
            int current = goal_grid_id;
            if (backpointers[current] == INF) {// no path found
                std::cout << "no path found" << std::endl;
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
                path_points.push_back(start);
                for (int i = 1; i < path.size() - 1; ++i) {
                    path_points.push_back(getGridCenter(path[i]));
                }
                path_points.push_back(goal);
            }
        } else {
            std::cout << "start or end point is out of map range" << std::endl;
        }
        return path_points;
    }

    std::vector<Point2D> GridMap2D::optimalToStraight(std::vector<Point2D> &path) const {
        if (path.size() > 2) {
            //The farthest point that can be straightened is the control point.
            std::vector<Point2D> pruned_path;
            std::vector<int> control_point_ids;
            int inner_idx = 0;
            int control_point_id = inner_idx;
            control_point_ids.push_back(control_point_id);
            while (inner_idx < path.size() - 1) {
                control_point_id = inner_idx;
                for (int i = inner_idx + 1; i < path.size(); ++i) {
                    if (isCollisionFreeStraight(path[inner_idx], path[i])) {//Use the straight line between two points
                        control_point_id = i;
                    }
                }
                if (control_point_id == inner_idx) {
                    control_point_id = inner_idx + 1;//next control point must be the next point of inner_idx
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
}

