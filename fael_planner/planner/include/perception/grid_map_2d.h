//
// Created by hjl on 2021/11/13.
//

#ifndef TOPO_PLANNER_WS_GRID_MAP_2D_H
#define TOPO_PLANNER_WS_GRID_MAP_2D_H

#include <future>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace perception {

    enum Status2D {
        Free = 0, Occupied = 1, Empty = 2, Unknown = 3
    };

    struct Grid2D {
        Status2D status;
        pcl::PointCloud<pcl::PointXYZI> points;

        Grid2D() : status(Status2D::Unknown) {};

        explicit Grid2D(Status2D sta) : status(sta) {};

        void clear() {
            status = Status2D::Unknown;
            points.clear();
        };
    };

    typedef Eigen::Vector2d Point2D;
    typedef Eigen::Vector2i Index2D;
    typedef std::vector<Index2D> Ray2D;

    //Convert coordinate index to hash index
    struct IdxHash {
        size_t operator()(const Index2D &key) const {
            return ((std::hash<int>()(key.x()) ^ (std::hash<int>()(key.y()) << 1)) >> 1);
        }
    };

    struct EqualIdx {
        bool operator()(const Index2D &lhs, const Index2D &rhs) const {
            return lhs.x() == rhs.x() && lhs.y() == rhs.y();
        }
    };

    typedef std::unordered_set<Index2D, IdxHash, EqualIdx> Index2DSet;

    class GridMap2D {
    public:
        typedef std::shared_ptr<GridMap2D> Ptr;

        double grid_size_;
        int grid_x_num_;
        int grid_y_num_;//Must be an even number

        double x_length_;
        double y_length_;

        Point2D center_point_;

        double min_x_;//Boundary coordinates
        double min_y_;
        double max_x_;
        double max_y_;

        std::vector<std::vector<Grid2D>> grids_;

        GridMap2D();

        GridMap2D(double &grid_size, int &grid_x_num, int &grid_y_num, Status2D init_status);

        GridMap2D(double &grid_size, double &x_length, double &y_length, Status2D init_status);

        void initialize(double &grid_size, int &grid_x_num, int &grid_y_num, Status2D init_status);

        void initialize(double &grid_size, double &x_length, double &y_length, Status2D init_status);

        void setMapCenterAndBoundary(Point2D &center_point);

        void clearMap() {
            grids_.clear();
        };

        void resetMapStatus(Status2D status);

        Ray2D castRay(Point2D &start, Point2D &end);

        std::vector<Index2D> getAllOccupiedGrids();

        void inflateGridMap2D(const double &inflate_radius,const double &inflate_empty_radius);

        visualization_msgs::MarkerArray
        generateMapMarkers(std::vector<std::vector<Grid2D>> &grid_map, geometry_msgs::Pose &current_pose);

        inline bool isCollisionFreeStraight(const Point2D &source, const Point2D &target) const {
            if (isInMapRange2D(source) && isInMapRange2D(target)) {
                if (getStatusInMap2D(source) != Status2D::Free || getStatusInMap2D(target) != Status2D::Free) {
                    return false;
                } else {
                    Index2D start_sub = getIndexInMap2D(source);
                    Index2D end_sub = getIndexInMap2D(target);
                    if (start_sub == end_sub)
                        return true;
                    Index2D diff_sub = end_sub - start_sub;
                    double max_dist = diff_sub.squaredNorm();
                    int step_x = signum(diff_sub.x());
                    int step_y = signum(diff_sub.y());
                    double t_max_x = step_x == 0 ? DBL_MAX : intbound(start_sub.x(), diff_sub.x());
                    double t_max_y = step_y == 0 ? DBL_MAX : intbound(start_sub.y(), diff_sub.y());
                    double t_delta_x = step_x == 0 ? DBL_MAX : (double) step_x / (double) diff_sub.x();
                    double t_delta_y = step_y == 0 ? DBL_MAX : (double) step_y / (double) diff_sub.y();
                    Index2D cur_sub = start_sub;

                    while (isInMapRange2D(cur_sub) && cur_sub != end_sub &&
                           (cur_sub - start_sub).squaredNorm() <= max_dist) {
                        if (getStatusInMap2D(cur_sub) != Status2D::Free) {
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
                    if(!isInMapRange2D(cur_sub))
                        return false;
                    return true;

                }
            } else {
                return false;
            }
        };

        inline void addPointInGrid(const Index2D &index, const pcl::PointXYZI &point) {
            grids_[index.x()][index.y()].points.push_back(point);
        }

        inline void addPointInGrid(const int &x, const int &y, const pcl::PointXYZI &point) {
            grids_[x][y].points.push_back(point);
        }

        inline void setFree(const Index2D &index) {
            grids_[index.x()][index.y()].status = Status2D::Free;
        };

        inline void setFree(const int &x, const int &y) {
            grids_[x][y].status = Status2D::Free;
        };

        inline void setOccupied(const Index2D &index) {
            grids_[index.x()][index.y()].status = Status2D::Occupied;
        };

        inline void setOccupied(const int &x, const int &y) {
            grids_[x][y].status = Status2D::Occupied;
        };

        inline void setEmpty(const Index2D &index) {
            grids_[index.x()][index.y()].status = Status2D::Empty;
        };

        inline void setEmpty(const int &x, const int &y) {
            grids_[x][y].status = Status2D::Empty;
        };

        inline void setUnknown(const Index2D &index) {
            grids_[index.x()][index.y()].status = Status2D::Unknown;
        };

        inline void setUnknown(const int &x, const int &y) {
            grids_[x][y].status = Status2D::Unknown;
        };

        inline Status2D getStatusInMap2D(const Index2D &index) const {
            if (isInMapRange2D(index))
                return grids_[index.x()][index.y()].status;
            else
                return Status2D::Unknown;
        }

        inline Status2D getStatusInMap2D(const Point2D &point) const {
            if (isInMapRange2D(point)) {
                return grids_[floor((point.x() - min_x_) / grid_size_)][floor(
                        (point.y() - min_y_) / grid_size_)].status;
            } else {
                return Status2D::Unknown;
            }
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

        inline bool isInMapRange2D(const Index2D &index) const {
            if (index.x() > 0 && index.x() < grid_x_num_ && index.y() > 0 && index.y() < grid_y_num_)
                return true;
            else
                return false;
        }

        inline Index2D getIndexInMap2D(const Point2D &point) const {
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

        inline Grid2D getGrid2D(const Index2D &index) const{
            if (isInMapRange2D(index))
                return grids_[index.x()][index.y()];
            else
                return Grid2D();
        }

        inline bool isNearOccupy(const Point2D &point, const double delta) const {
            double t_delta = grid_size_;
            for (double x = (point.x() - delta); x <= (point.x() + delta); x += t_delta) {
                for (double y = (point.y() - delta); y <= (point.y() + delta); y += t_delta) {
                    Point2D point2d(x, y);
                        if (getStatusInMap2D(point2d) == Status2D::Occupied)
                        return true;
                }
            }
            return false;
        };

        inline bool isNearEmpty(const Point2D &point, const double delta) const {
            double t_delta = grid_size_;
            for (double x = (point.x() - delta); x <= (point.x() + delta); x += t_delta) {
                for (double y = (point.y() - delta); y <= (point.y() + delta); y += t_delta) {
                    Point2D point2d(x, y);
                    if (getStatusInMap2D(point2d) == Status2D::Empty)
                        return true;
                }
            }
            return false;
        };

        inline bool isNearUnknown(const Point2D &point, const double delta) const {
            double t_delta = grid_size_;
            for (double x = (point.x() - delta); x <= (point.x() + delta); x += t_delta) {
                for (double y = (point.y() - delta); y <= (point.y() + delta); y += t_delta) {
                    Point2D point2d(x, y);
                    if (getStatusInMap2D(point2d) == Status2D::Unknown)
                        return true;
                }
            }
            return false;
        };

        inline bool isNearFree(const Point2D &point, const double delta) const{
            double t_delta = grid_size_;
            for (double x = (point.x() - delta); x <= (point.x() + delta); x += t_delta) {
                for (double y = (point.y() - delta); y <= (point.y() + delta); y += t_delta) {
                    Point2D point2d(x, y);
                    if (getStatusInMap2D(point2d) == Status2D::Free)
                        return true;
                }
            }
            return false;
        }

        std::vector<Point2D> getShortestPath(const Point2D &goal, const Point2D &start) const;

        std::vector<Point2D> optimalToStraight(std::vector<Point2D> &path) const;

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

        inline double minPositionElement(const Point2D &pos) const {
            return std::min(pos[0], pos[1]);
        }

        inline double maxPositionElement(const Point2D &pos) const {
            return std::max(pos[0], pos[1]);
        }

        inline size_t minElementIndex(const Point2D &pos) const {
            return pos[0] <= pos[1] ? 0 : 1;
        }
        
    };
}


#endif //TOPO_PLANNER_WS_GRID_MAP_2D_H
