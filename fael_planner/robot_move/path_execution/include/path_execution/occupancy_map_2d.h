//
// Created by hjl on 2021/10/22.
//

#ifndef TOPO_PLANNER_WS_OCCUPANCY_MAP_2D_H
#define TOPO_PLANNER_WS_OCCUPANCY_MAP_2D_H

#include <cmath>
#include <Eigen/Eigen>
#include <unordered_set>
#include <unordered_map>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace path_execution {

    enum struct Status {
        free = 0, occupied = 1, empty = 2, unknown = 3
    };

    struct Grid {
        Status status;

        Grid() : status(Status::unknown) {};

        explicit Grid(Status sta) : status(sta) {};

        void clear() {
            status = Status::unknown;
        };
    };

    typedef Eigen::Vector2d Point2D;
    typedef Eigen::Vector2i Index2D;
    typedef std::vector<Index2D> Ray2D;

    struct IdxHash2D {
        size_t operator()(const Index2D &key) const {
            return ((std::hash<int>()(key.x()) ^ (std::hash<int>()(key.y()) << 1)) >> 1);
        }
    };

    struct EqualIdx2D {
        bool operator()(const Index2D &lhs, const Index2D &rhs) const {
            return lhs.x() == rhs.x() && lhs.y() == rhs.y();
        }
    };

    typedef std::unordered_set<Index2D, IdxHash2D, EqualIdx2D> Index2DSet;

    class OccupancyMap2D {
    public:

        double grid_size_;
        int grid_x_num_;
        int grid_y_num_;

        double x_length_;
        double y_length_;

        Point2D center_point_;

        double min_x_; 
        double min_y_;
        double max_x_;
        double max_y_;

        std::vector<std::vector<Grid>> occupancy_map_;
        std::vector<std::vector<Grid>> inflate_map_;

        OccupancyMap2D();

        OccupancyMap2D(double &grid_size, int &grid_x_num, int &grid_y_num, Status init_status);

        OccupancyMap2D(double &grid_size, double &x_length, double &y_length, Status init_status);

        void initialize(double &grid_size, int &grid_x_num, int &grid_y_num, Status init_status);

        void initialize(double &grid_size, double &x_length, double &y_length, Status init_status);

        void setMapCenterAndBoundary(Point2D &center_point);

        void clearMap() {
            occupancy_map_.clear();
            inflate_map_.clear();
        };

        void resetMapStatus(Status status);

        Ray2D castRay(Point2D &start, Point2D &end);

        std::vector<Index2D> getAllOccupiedGrids();

        void inflateOccupancyMap(double &inflate_radius, double &inflate_empty_radius);

        visualization_msgs::MarkerArray generateMapMarkers(std::vector<std::vector<Grid>> &grid_map,geometry_msgs::Pose &current_pose);

        inline bool isCollisionFreeStraight(const Point2D &source, const Point2D &target) const {
            if (isInMapRange2D(source) && isInMapRange2D(target)) {
                if (getStatusInFlateMap(source) != Status::free || getStatusInFlateMap(target) != Status::free) {
                    return false;
                } else {
                    Index2D start_sub = getIndexInMap(source);
                    Index2D end_sub = getIndexInMap(target);
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
                        dist = (cur_sub - start_sub).squaredNorm();
                        if (cur_sub == end_sub || dist > max_dist)
                            return true;
                        if (getStatusInFlateMap(cur_sub) != Status::free) {
                            return false;
                        }
                        if (t_max_x < t_max_y) {
                            cur_sub.x() += step_x;
                            t_max_x += t_delta_x;
                        } else {
                            cur_sub.y() += step_y;
                            t_max_y += t_delta_y;
                        }
                    }
                    return false;
                }
            } else {
                return false;
            }
        };

        inline int signum(const int &x) const {
            return x == 0 ? 0 : x < 0 ? -1 : 1;
        }

        inline double mod(const double &value, const double &modulus) const {
            return fmod(fmod(value, modulus) + modulus, modulus);
        }

        inline double intbound(const double &s, const double &ds) const {
            // Find the smallest positive t such that s+t*ds is an integer.
            if (ds < 0) {
                return intbound(-s, -ds);
            } else {
                // problem is now s+t*ds = 1
                return (1 - mod(s, 1)) / ds;
            }
        }

        inline Status getStatusInFlateMap(const Index2D &index) const {
            if (isInMapRange2D(index))
                return inflate_map_[index.x()][index.y()].status;
            else
                return Status::unknown;
        }

        inline Status getStatusInFlateMap(const Point2D &point) const {
            if (isInMapRange2D(point)) {
                return inflate_map_[floor((point.x() - min_x_) / grid_size_)][floor(
                        (point.y() - min_y_) / grid_size_)].status;
            } else {
                return Status::unknown;
            }
        }

        inline void setFree(const Index2D &index) {
            occupancy_map_[index.x()][index.y()].status = Status::free;
        };
        inline void setFree(const int &index_x, const int &index_y) {
            occupancy_map_[index_x][index_y].status = Status::free;
        };

        inline void setOccupied(const Index2D &index) {
            occupancy_map_[index.x()][index.y()].status = Status::occupied;
        };

        inline void setOccupied(const int &index_x, const int &index_y) {
            occupancy_map_[index_x][index_y].status = Status::occupied;
        };

        inline void setUnknown(const Index2D &index) {
            occupancy_map_[index.x()][index.y()].status = Status::unknown;
        };

        inline void setEmpty(const Index2D &index) {
            occupancy_map_[index.x()][index.y()].status = Status::empty;
        };

        inline void setEmpty(const int &index_x, const int &index_y) {
            occupancy_map_[index_x][index_y].status = Status::empty;
        };

        inline void setUnknown(const int &index_x, const int &index_y) {
            occupancy_map_[index_x][index_y].status = Status::unknown;
        };

        inline bool isInMapRange2D(const Index2D &index) const {
            if (index.x() > 0 && index.x() < grid_x_num_ && index.y() > 0 && index.y() < grid_y_num_)
                return true;
            else
                return false;
        }

        inline bool isInMapRange2D(const Point2D &point) const {
            if (point.x() < (min_x_ + 1e-4) || point.x() > (max_x_ - 1e-4) ||
                point.y() < (min_y_ + 1e-4) || point.y() > (max_y_ - 1e-4)) {
                return false;
            } else {
                return true;
            }
        };

        inline bool isInMapRange2D(const pcl::PointXYZI &point) const {
            if (point.x < min_x_ + 1e-4 || point.x > max_x_ - 1e-4 ||
                point.y < min_y_ + 1e-4 || point.y > max_y_ - 1e-4) {
                return false;
            } else {
                return true;
            }
        };

        inline Index2D getIndexInMap(const Point2D &point) const {
            Index2D index;
            index.x() = floor((point.x() - min_x_) / grid_size_);
            index.y() = floor((point.y() - min_y_) / grid_size_);
            return index;
        };

        inline Index2D getIndexInMap2D(const pcl::PointXYZI &point) const {
            Index2D index;
            index.x() = floor((point.x - min_x_) / grid_size_);
            index.y() = floor((point.y - min_y_) / grid_size_);
            return index;
        };

        inline Point2D getGridCenter(const Index2D &index) const {
            Point2D center;
            center.x() = min_x_ + grid_size_ / 2 + index.x() * grid_size_;
            center.y() = min_y_ + grid_size_ / 2 + index.y() * grid_size_;
            return center;
        };

        inline Point2D getGridCenter(const int &index_x, const int &index_y) const {
            Point2D center;
            center.x() = min_x_ + grid_size_ / 2 + index_x * grid_size_;
            center.y() = min_y_ + grid_size_ / 2 + index_y * grid_size_;
            return center;
        };

        inline Point2D getGridCenter(const int &grid_id) const {
            Point2D center;
            center.x() = min_x_ + grid_size_ / 2 + grid_size_ * (grid_id / grid_y_num_);
            center.y() = min_y_ + grid_size_ / 2 + grid_size_ * (grid_id % grid_y_num_);
            return center;
        }

        inline int Index2DToGridId(const Index2D &index) const {
            return index.x() * grid_y_num_ + index.y();
        };

        inline int Index2DToGridId(const int &index_x, const int &index_y) const {
            return index_x * grid_y_num_ + index_y;
        }

        inline Index2D gridIdToIndex2D(const int &id) const {
            Index2D index;
            index.x() = id / grid_y_num_;
            index.y() = id % grid_y_num_;
            return index;
        };

        inline bool isNearOccupy(const Point2D &point, const double delta) const {
            double t_delta = grid_size_;
            for (double x = (point.x() - delta); x <= (point.x() + delta); x += t_delta) {
                for (double y = (point.y() - delta); y <= (point.y() + delta); y += t_delta) {
                    Point2D point2d(x, y);
                    if (getStatusInFlateMap(point2d) == Status::occupied ||
                        getStatusInFlateMap(point2d) == Status::unknown)
                        return true;
                }
            }
            return false;
        };
    };
}


#endif //TOPO_PLANNER_WS_OCCUPANCY_MAP_2D_H
