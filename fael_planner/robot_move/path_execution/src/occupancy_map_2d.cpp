//
// Created by hjl on 2021/10/22.
//

#include "path_execution/occupancy_map_2d.h"

namespace path_execution {
    OccupancyMap2D::OccupancyMap2D() {

    }

    OccupancyMap2D::OccupancyMap2D(double &grid_size, int &grid_x_num, int &grid_y_num, Status init_status) {
        initialize(grid_size, grid_x_num, grid_x_num, init_status);
    }

    OccupancyMap2D::OccupancyMap2D(double &grid_size, double &x_length, double &y_length, Status init_status) {
        initialize(grid_size, x_length, y_length, init_status);
    }

    void OccupancyMap2D::initialize(double &grid_size, int &grid_x_num, int &grid_y_num, Status init_status) {
        grid_size_ = grid_size;
        grid_x_num_ = grid_x_num;
        grid_y_num_ = grid_y_num;

        x_length_ = grid_size * grid_x_num;
        y_length_ = grid_size * grid_y_num;

        occupancy_map_ = std::vector<std::vector<Grid>>(grid_x_num_, std::vector<Grid>(grid_y_num_, Grid(init_status)));
    }

    void OccupancyMap2D::initialize(double &grid_size, double &x_length, double &y_length, Status init_status) {
        grid_size_ = grid_size;
        x_length_ = x_length;
        y_length_ = y_length;

        grid_x_num_ = std::floor(x_length / grid_size);
        grid_y_num_ = std::floor(y_length / grid_size);

        occupancy_map_ = std::vector<std::vector<Grid>>(grid_x_num_, std::vector<Grid>(grid_y_num_, Grid(init_status)));
    }

    void OccupancyMap2D::setMapCenterAndBoundary(Point2D &center_point) {
        center_point_ = center_point;
        min_x_ = center_point.x() - x_length_ / 2;
        max_x_ = center_point.x() + x_length_ / 2;
        min_y_ = center_point.y() - y_length_ / 2;
        max_y_ = center_point.y() + y_length_ / 2;
    }

    Ray2D OccupancyMap2D::castRay(Point2D &start, Point2D &end) {

        double end_distance = (end - start).norm();
        Eigen::Vector2d directory = (end - start).normalized();
        double step_length = grid_size_ / 2; 
        int step = 0;
        Ray2D ray;

        Index2D end_index = getIndexInMap(end);
        Point2D inner_point = start + directory * step_length * step;
        Index2D inner_index = getIndexInMap(inner_point);
        while ((inner_point - start).norm() < end_distance && inner_index != end_index) {
            ray.push_back(inner_index);
            ++step;
            inner_point = start + directory * step_length * step;
            inner_index = getIndexInMap(inner_point);
        }

        return ray;
    }

    void OccupancyMap2D::resetMapStatus(Status status) {
        for (auto &item:occupancy_map_) {
            for (auto &grid:item) {
                grid.status = status;
            }
        }
    }

    std::vector<Index2D> OccupancyMap2D::getAllOccupiedGrids() {
        std::vector<Index2D> occupied_grids;
        for (int i = 0; i < grid_x_num_; ++i) {
            for (int j = 0; j < grid_y_num_; ++j) {
                if (occupancy_map_[i][j].status == Status::occupied) {
                    occupied_grids.emplace_back(i, j);
                }
            }
        }
        return occupied_grids;
    }

    void OccupancyMap2D::inflateOccupancyMap(double &inflate_radius, double &inflate_empty_radius) {
        inflate_map_ = occupancy_map_;
        int increment = std::ceil(abs(inflate_radius / grid_size_));
        int increment_empty = std::ceil(abs(inflate_empty_radius / grid_size_)); 
        for (int I = 0; I < grid_x_num_; ++I) {
            for (int J = 0; J < grid_y_num_; ++J) {
                if (occupancy_map_[I][J].status == Status::occupied) {
                    for (int i = -increment; i <= increment; ++i) {
                        for (int j = -increment; j <= increment; ++j) {
                            if (I + i >= 0 && I + i < grid_x_num_ && J + j >= 0 && J + j < grid_y_num_) {
                                inflate_map_[I + i][J + j].status = Status::occupied;
                            }
                        }
                    }
                }

                if (occupancy_map_[I][J].status == Status::empty) {
                    for (int i = -increment_empty; i <= increment_empty; ++i) {
                        for (int j = -increment_empty; j <= increment_empty; ++j) {
                            if (I + i >= 0 && I + i < grid_x_num_ && J + j >= 0 && J + j < grid_y_num_) {
                                inflate_map_[I + i][J + j].status = Status::empty;
                            }
                        }
                    }
                }
            }
        }
    }

    visualization_msgs::MarkerArray
    OccupancyMap2D::generateMapMarkers(std::vector<std::vector<Grid>> &grid_map, geometry_msgs::Pose &current_pose) {

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
                if (grid_map[i][j].status == Status::free) {
                    free.points.push_back(point);
                }
                if (grid_map[i][j].status == Status::occupied) {
                    occupied.points.push_back(point);
                }
                if (grid_map[i][j].status == Status::empty) {
                    empty.points.push_back(point);
                }
                if (grid_map[i][j].status == Status::unknown) {
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
}

