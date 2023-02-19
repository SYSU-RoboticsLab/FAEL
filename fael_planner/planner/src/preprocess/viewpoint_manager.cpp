//
// Created by hjl on 2022/1/10.
//

#include "preprocess/viewpoint_manager.h"

namespace preprocess {

    ViewpointManager::ViewpointManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                                       const Ufomap::Ptr &frontier_map, const Map2DManager::Ptr &map_2d_manager_) :
            nh_(nh), nh_private_(nh_private), frontier_map_(frontier_map), map_2d_manager_(map_2d_manager_) {
        getParamsFromRos();

        repre_points_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("repre_points_markers", 1);
        frontiers_viewpoints_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
                "frontiers_viewpoints_markers", 1);
    }

    void ViewpointManager::getParamsFromRos() {
        std::string ns = ros::this_node::getName() + "/ViewpointManager";

        std::string pkg_path = ros::package::getPath("planner");
        std::string txt_path = pkg_path + "/../../files/exploration_data/";

        frame_id_ = "world";
        if (!ros::param::get(ns + "/frame_id", frame_id_)) {
            ROS_WARN("No frame_id specified. Looking for %s. Default is 'world'.",
                     (ns + "/frame_id").c_str());
        }

        sample_dist_ = 1.0;
        if (!ros::param::get(ns + "/sample_dist", sample_dist_)) {
            ROS_WARN("No sample_dist specified. Looking for %s. Default is '1.0'.",
                     (ns + "/sample_dist").c_str());
        }

        frontier_dist_ = 2.0;
        if (!ros::param::get(ns + "/frontier_dist", frontier_dist_)) {
            ROS_WARN("No frontier_dist specified. Looking for %s. Default is '2.0'.",
                     (ns + "/frontier_dist").c_str());
        }

        viewpoint_gain_thre_ = 2.0;
        if (!ros::param::get(ns + "/viewpoint_gain_thre", viewpoint_gain_thre_)) {
            ROS_WARN("No viewpoint_gain_thre specified. Looking for %s. Default is '2.0'.",
                     (ns + "/viewpoint_gain_thre").c_str());
        }

        frontiers_attach_txt_name_ = txt_path + "frontier_attach_time.txt";

        attach_num_ = 0;
        sum_attach_time_ = 0;
        fout_.open(frontiers_attach_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout_ << "each frontier attach time \n"
              << "start time \t" << " end time \t" << " elisped time \t" << "iteration_num \t" << "average time(s) \t"
              << std::endl;
        fout_.close();
    }

    void ViewpointManager::setCurrentPosition(const utils::Point3D &current_position) {
        current_position_ = current_position;
    }

    void ViewpointManager::updateViewpoints() {
        if (map_2d_manager_->is_map_updated_) {
            auto start_time = ros::WallTime::now();

            frontierAttachInUfomap(); 
            
            auto end_time = ros::WallTime::now();

            sum_attach_time_ = sum_attach_time_ + (end_time - start_time).toSec();
            attach_num_++;
            fout_.open(frontiers_attach_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
            fout_ << start_time << "\t" << end_time << "\t" << (end_time - start_time).toSec()
                  << "\t" << attach_num_ << "\t" << sum_attach_time_ / attach_num_ << "s \t"
                  << std::endl;
            fout_.close();
            ROS_INFO("frontier attached spend %f s", (end_time - start_time).toSec());
        }
    }

    void ViewpointManager::frontierAttachInUfomap() {

        Point3DQueue sample_points = samplePointsInGridMap2D();

        ROS_INFO("start attach frontiers to candidate points..");
    
        representative_points_.clear();
        for (const auto &point: sample_points) {
            bool is_suitable = true;
            for (const auto &viewpoint: candidate_viewpoints_) {
                if (point.distance(viewpoint) < 1.0) {
                    is_suitable = false;
                    break;
                }
            }
            if (is_suitable) {
                representative_points_.insert(point);
            }
        }

        representative_points_.insert(candidate_viewpoints_.begin(), candidate_viewpoints_.end());
        ROS_INFO("representative points size is %zu", representative_points_.size());

        FrontierMap<Viewpoint> frontiers_viewpoints_term;
        for (const auto &frontier: frontier_map_->getFrontiers()) {
            
            if (frontier.distanceXY(current_position_) >
                frontier_map_->max_range_ * 1.5 
                && !frontiers_viewpoints_.empty() &&
                frontiers_viewpoints_.count(frontier) != 0) {
                frontiers_viewpoints_term[frontier] = frontiers_viewpoints_[frontier];
            
            } else if (map_2d_manager_->inflate_map_.isNearFree(Point2D(frontier.x(), frontier.y()), frontier_dist_)) {
                double min_distance = 100000;
                ufo::map::Point3 frontier_coord(frontier.x(), frontier.y(), frontier.z());
                ufo::map::Point3 sensor_point;
                double distance;

                for (const auto &point: representative_points_) {
                    sensor_point.x() = point.x();
                    sensor_point.y() = point.y();
                    sensor_point.z() = point.z();
                    distance = frontier_coord.distanceXY(sensor_point);

                    if (distance < frontier_map_->max_range_ - 0.5 &&
                        distance < min_distance &&
                        fabs(frontier_coord.z() - sensor_point.z()) / distance < tan(M_PI * 15 / 180) &&
                        frontier_map_->map_.isCollisionFree(frontier_coord,
                                                            sensor_point)) {
                        min_distance = distance;
                        frontiers_viewpoints_term[frontier] = point;
                    }
                }
            }
        }
        frontiers_viewpoints_.clear();
        frontiers_viewpoints_ = frontiers_viewpoints_term;
        ROS_INFO("viewpoints and frontiers nearest search finish.");

        viewpoints_attached_frontiers_.clear();
        for (const auto &item: frontiers_viewpoints_) {
            viewpoints_attached_frontiers_[item.second].push_back(item.first);
        }

        Point3DQueue need_to_erase;
        for (const auto &term:viewpoints_attached_frontiers_) {
            if (term.first.distanceXY(current_position_) < sample_dist_ / 2) {
                need_to_erase.push_back(term.first);
                for (const auto &frontier:term.second) {
                    auto code = frontier_map_->map_.toCode(frontier.x(), frontier.y(), frontier.z(),
                                                           frontier_map_->frontier_depth_);
                    frontier_map_->global_frontier_cells_.erase(code);
                }
            }
            if (term.second.size()<viewpoint_gain_thre_)
                need_to_erase.push_back(term.first);
        }

        for (const auto &point:need_to_erase) {
            viewpoints_attached_frontiers_.erase(point);
        }

        Point3DSet old_viewpoints = candidate_viewpoints_;
        candidate_viewpoints_.clear();
        for (const auto &item: viewpoints_attached_frontiers_) {
            candidate_viewpoints_.insert(item.first);
        }

        new_viewpoints_.clear();
        for (const auto &point: candidate_viewpoints_) {
            if (old_viewpoints.count(point) == 0) {
                new_viewpoints_.insert(point);
            }
        }

        ROS_INFO("attach frontiers finished, candidate points size is %zu", candidate_viewpoints_.size());
    }

    ViewpointQueue ViewpointManager::samplePointsInGridMap2D() {

        double sample_dist = sample_dist_;

        ViewpointQueue sample_points;//points meeting sampling requirements
        for (double x = -map_2d_manager_->inflate_map_.x_length_ / 2;
             x <= map_2d_manager_->inflate_map_.x_length_ / 2; x += sample_dist) {
            for (double y = -map_2d_manager_->inflate_map_.y_length_ / 2;
                 y <= map_2d_manager_->inflate_map_.y_length_ / 2; y += sample_dist) 
            {
                Point2D sample_2d(current_position_.x() + x, current_position_.y() + y);
                if (frontier_map_->isInExplorationArea(sample_2d.x(), sample_2d.y())
                    && map_2d_manager_->inflate_map_.getStatusInMap2D(sample_2d) == Status2D::Free
                    && !map_2d_manager_->inflate_map_.isNearOccupy(sample_2d, 0.3)
                    && !map_2d_manager_->inflate_map_.isNearEmpty(sample_2d, 0.5)
                    && !map_2d_manager_->inflate_map_.isNearUnknown(sample_2d, 0.3)
                    && (Point2D(current_position_.x(), current_position_.y()) - sample_2d).norm() <
                       (frontier_map_->max_range_ -
                        (frontier_map_->robot_height_ - frontier_map_->sensor_height_) / tan(M_PI * 15 / 180))
                    && map_2d_manager_->inflate_map_.isCollisionFreeStraight(
                        Point2D(current_position_.x(), current_position_.y()), sample_2d)
                        ) {
                    pcl::PointXYZI min; 
                    pcl::PointXYZI max;   
                    pcl::getMinMax3D(map_2d_manager_->inflate_map_.getGrid2D(
                            map_2d_manager_->inflate_map_.getIndexInMap2D(sample_2d)).points, min, max);
                    Point3D sample_point(sample_2d.x(), sample_2d.y(),
                                         min.z + frontier_map_->sensor_height_);
                    sample_points.push_back(sample_point);
                }
            }
        }
        ROS_INFO("sample viewpoint finish, size is %zu", sample_points.size());
        return sample_points;
    }

    visualization_msgs::MarkerArray ViewpointManager::generatePointsMarkers(const Point3DSet &sample_points) const {

        visualization_msgs::Marker points;
        points.header.frame_id = frame_id_;
        points.header.stamp = ros::Time::now();
        points.ns = "representative_points";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;

        points.id = 0;
        points.type = visualization_msgs::Marker::SPHERE_LIST;

        points.scale.x = 0.3;
        points.scale.y = 0.3;
        points.scale.z = 0.3;

        //assigning colors
        points.color.r = 0.5f;
        points.color.g = 0.1f;
        points.color.b = 1.0f;
        points.color.a = 1.0f;

        for (auto node: sample_points) {
            geometry_msgs::Point point;
            point.x = node.x();
            point.y = node.y();
            point.z = node.z();
            points.points.push_back(point);
        }
        visualization_msgs::MarkerArray repre_points_markers;
        repre_points_markers.markers.resize(1);
        repre_points_markers.markers[0] = points;

        return repre_points_markers;
    }

    visualization_msgs::MarkerArray
    ViewpointManager::generateViewPointsWithFrontiersMarkers(
            const ViewpointMap<FrontierQueue> &viewpoints_attached_frontiers) const {
        visualization_msgs::Marker points;
        visualization_msgs::Marker edges;
        visualization_msgs::Marker squares;

        edges.header.frame_id = points.header.frame_id = squares.header.frame_id = frame_id_;
        edges.header.stamp = points.header.stamp = squares.header.stamp = ros::Time::now();
        edges.ns = "connections";
        points.ns = "viewpoints";
        squares.ns = "frontiers";
        edges.action = points.action = squares.action = visualization_msgs::Marker::ADD;
        edges.pose.orientation.w = points.pose.orientation.w = squares.pose.orientation.w = 1.0;

        points.id = 0;
        points.type = visualization_msgs::Marker::SPHERE_LIST;

        edges.id = 1;
        edges.type = visualization_msgs::Marker::LINE_LIST;

        squares.id = 2;
        squares.type = visualization_msgs::Marker::CUBE_LIST;

        points.scale.x = 0.5;
        points.scale.y = 0.5;
        points.scale.z = 0.5;

        //assigning colors
        points.color.r = 1.0f;
        points.color.g = 0.5f;
        points.color.b = 1.0f;
        points.color.a = 1.0f;

        //setting scale
        edges.scale.x = 0.1;
        edges.scale.y = 0.1;
        edges.scale.z = 0.1;

        edges.color.r = 0.5f;
        edges.color.g = 0.5f;
        edges.color.b = 0.1f;
        edges.color.a = 0.5f;

        squares.scale.x = frontier_map_->map_.getResolution();
        squares.scale.y = frontier_map_->map_.getResolution();
        squares.scale.z = frontier_map_->map_.getResolution();

        //assigning colors
        squares.color.r = 0.1f;
        squares.color.g = 0.9f;
        squares.color.b = 0.1f;
        squares.color.a = 1.0f;

        for (const auto &node: viewpoints_attached_frontiers) {
            geometry_msgs::Point point;
            point.x = node.first.x();
            point.y = node.first.y();
            point.z = node.first.z();
            points.points.push_back(point);
        }
        for (const auto &edge: viewpoints_attached_frontiers) {
            auto first = edge.first;
            for (const auto &second:edge.second) {
                geometry_msgs::Point point;
                point.x = first.x();
                point.y = first.y();
                point.z = first.z();
                edges.points.push_back(point);
                point.x = second.x();
                point.y = second.y();
                point.z = second.z();
                edges.points.push_back(point);
                squares.points.push_back(point);
            }
        }

        for (const auto &node: viewpoints_attached_frontiers) {
            geometry_msgs::Point point;
            point.x = node.first.x();
            point.y = node.first.y();
            point.z = node.first.z();
            points.points.push_back(point);
        }
        for (const auto &edge: viewpoints_attached_frontiers) {
            auto first = edge.first;
            for (const auto &second:edge.second) {
                geometry_msgs::Point point;
                point.x = first.x();
                point.y = first.y();
                point.z = first.z();
                edges.points.push_back(point);
                point.x = second.x();
                point.y = second.y();
                point.z = second.z();
                edges.points.push_back(point);
                squares.points.push_back(point);
            }
        }

        visualization_msgs::MarkerArray frontiers_viewpoints_markers;
        frontiers_viewpoints_markers.markers.resize(3);
        frontiers_viewpoints_markers.markers[0] = points;
        frontiers_viewpoints_markers.markers[1] = edges;
        frontiers_viewpoints_markers.markers[2] = squares;

        return frontiers_viewpoints_markers;
    }

    void ViewpointManager::pubMarkers() const {
        repre_points_pub_.publish(generatePointsMarkers(representative_points_));
        frontiers_viewpoints_pub_.publish(generateViewPointsWithFrontiersMarkers(viewpoints_attached_frontiers_));
    }
}

