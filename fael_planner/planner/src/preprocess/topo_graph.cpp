//
// Created by hjl on 2022/1/10.
//

#include "preprocess/topo_graph.h"

namespace preprocess {

    TopoGraph::TopoGraph(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
            nh_(nh), nh_private_(nh_private), is_graph_initialized_(false) {
        getParamsFromRos();
        graph_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("topo_graph_markers", 1);

    }

    void TopoGraph::getParamsFromRos() {
        std::string ns = ros::this_node::getName() + "/Roadmap";

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

        connectable_range_ = 5;
        if (!ros::param::get(ns + "/connectable_range", connectable_range_)) {
            ROS_WARN("No connectable_range specified. Looking for %s. Default is 5.",
                     (ns + "/connectable_range").c_str());
        }

        connectable_num_ = 5;
        if (!ros::param::get(ns + "/connectable_num", connectable_num_)) {
            ROS_WARN("No connectable_num specified. Looking for %s. Default is 5.",
                     (ns + "/connectable_num").c_str());
        }

        each_graph_update_txt_name_ = txt_path + "each_roadmap_update_time.txt";

        sum_graph_update_time_ = 0;
        graph_update_num_ = 0;
        std::ofstream fout;
        fout.open(each_graph_update_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "each roadmap update elisped time \n"
             << " start time \t" << "  end time \t" << "  elisped time \t"
             << "update num \t" << "average time \t"
             << std::endl;
        fout.close();

    }

    void TopoGraph::setCurrentPosition(const Point3D &current_position) {
        current_position_ = current_position;
    }

    void TopoGraph::addVertex(const Point3D &point) {
        if (!graph_.isPoint3DExisted(point)) {
            graph_.addVertex(point);
        }
    }

    void TopoGraph::updateTopoGraphByMap2DAndViewpoints(const Map2DManager::Ptr &map_2d_manager,
                                                        const ViewpointManager::Ptr &viewpoint_manager,
                                                        const Ufomap::Ptr &frontier_map) {
        if (map_2d_manager->is_map_updated_) {
            auto start_time = ros::WallTime::now();
            Point3DSet sample_points;//points meeting sampling requirements
            for (double x = -map_2d_manager->inflate_map_.x_length_ / 2;
                 x <= map_2d_manager->inflate_map_.x_length_ / 2; x += sample_dist_) {
                for (double y = -map_2d_manager->inflate_map_.y_length_ / 2;
                     y <= map_2d_manager->inflate_map_.y_length_ / 2; y += sample_dist_) {
                    Point2D sample_2d(current_position_.x() + x, current_position_.y() + y);
                    if (map_2d_manager->inflate_map_.getStatusInMap2D(sample_2d) == Status2D::Free &&
                            !map_2d_manager->inflate_map_.isNearOccupy(sample_2d, 0.5)) {
                        //if the grid is free, then this place is a relatively flat ground.
                        pcl::PointXYZI min;    
                        pcl::PointXYZI max;    
                        pcl::getMinMax3D(map_2d_manager->inflate_map_.getGrid2D(
                                map_2d_manager->inflate_map_.getIndexInMap2D(sample_2d)).points, min, max);
                        Point3D sample_point(sample_2d.x(), sample_2d.y(), min.z + 0.05);//a little higher than the lowest point on the ground.
                        if (frontier_map->isInExplorationArea(sample_point.x(),sample_point.y()))
                            sample_points.insert(sample_point);
                    }
                }
            }

            if (is_graph_initialized_) {
                //growing
                growingByMap2dAndSamplePoints(map_2d_manager->inflate_map_, sample_points);
            } else {

                initTopoGraphByCurrentPositionAndReprePoints(map_2d_manager->inflate_map_, current_position_,
                                                             sample_points);
            }
            auto end_time = ros::WallTime::now();
            sum_graph_update_time_ = sum_graph_update_time_ + (end_time - start_time).toSec();
            graph_update_num_++;
            std::ofstream fout;
            fout.open(each_graph_update_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
            fout << start_time << "\t" << end_time << "\t" << (end_time - start_time).toSec()
                 << "\t" << graph_update_num_ << "\t" << sum_graph_update_time_ / graph_update_num_ << "s \t"
                 << std::endl;
            fout.close();
        }

    }

    void TopoGraph::initTopoGraphByCurrentPositionAndReprePoints(const GridMap2D &grid_map_2d,
                                                                 const Point3D &current_position,
                                                                 const Point3DSet &repre_points) {
        graph_.clearGraph();
        if (!repre_points.empty()) {
            //first add the origin
            addVertex(current_position);
            int origin_point_id;
            graph_.getPointId(current_position, origin_point_id);
            //use sample points to grow
            growingByMap2dAndSamplePoints(grid_map_2d, repre_points);

            is_graph_initialized_ = true;
        }
    }

    void TopoGraph::growingByMap2dAndSamplePoints(const GridMap2D &grid_map_2d, const Point3DSet &sample_points) {
        
        Point3DQueue local_points;

        for (const auto &vertex: graph_.getAllVertices()) {
            if (current_position_.distance(vertex) <
                (grid_map_2d.x_length_ * 3 / 2 + connectable_range_)) {//range of points to compare
                local_points.push_back(vertex);
            }
        }

        Point3DQueue suitable_points;
        Point3DQueue selected_points = local_points;//store points that need to be searched edges' connection relationship
        double min_intervel = 1.0;
        for (const auto &point: sample_points) {
            bool suitable = true;
            for (const auto &vertex: selected_points) {
                if (point.distance(vertex) < min_intervel) {//make sure the new point is not too close to the local graph node
                    suitable = false;
                    break;
                }
            }
            if (suitable) {
                suitable_points.push_back(point);
                selected_points.push_back(point);
                addVertex(point);
            }
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
        pcl::PointCloud<pcl::PointXYZ> point_cloud;
        for (const auto &point:selected_points) {
            point_cloud.emplace_back(point.x(), point.y(), point.z());
        }
        kd_tree.setInputCloud(point_cloud.makeShared());
        for (const auto &point:selected_points) {
            //connect each local point 
            int point_id = -1;
            if (graph_.getPointId(point, point_id)) {
                pcl::PointXYZ center(point.x(), point.y(), point.z());
                std::vector<int> neighbors_ids;
                std::vector<float> neighbors_distance;
                kd_tree.radiusSearch(center, connectable_range_, neighbors_ids, neighbors_distance);
                Point3DQueue neighbor_vertexs;
                int connected_num = 0;
                for (const auto &id:neighbors_ids) {
                    Point3D vertex(point_cloud[id].x, point_cloud[id].y, point_cloud[id].z);
                    if (point_id != id && grid_map_2d.isCollisionFreeStraight(Point2D(point.x(), point.y()),
                                                                              Point2D(vertex.x(), vertex.y()))) {
                        int vertex_id = -1;
                        graph_.getPointId(vertex, vertex_id);
                        graph_.addTwoWayEdge(point_id, vertex_id);
                        connected_num++;
                        if (connected_num > connectable_num_)
                            break;
                    }
                }
            }
        }
    }

    visualization_msgs::MarkerArray TopoGraph::generateTopoGraphMarkers() const {
        visualization_msgs::Marker vertices;
        visualization_msgs::Marker edges;

        //init headers
        vertices.header.frame_id = edges.header.frame_id = frame_id_;
        vertices.header.stamp = edges.header.stamp = ros::Time::now();
        vertices.ns = edges.ns = "graph";
        vertices.action = edges.action = visualization_msgs::Marker::ADD;
        vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

        //setting id for each marker
        vertices.id = 0;
        edges.id = 1;

        //defining types
        edges.type = visualization_msgs::Marker::LINE_LIST;
        vertices.type = visualization_msgs::Marker::POINTS;

        //setting scale
        edges.scale.x = 0.05;
        edges.scale.y = 0.05;
        edges.scale.z = 0.05;

        vertices.scale.x = 0.1;
        vertices.scale.y = 0.1;
        vertices.scale.z = 0.1;

        //assigning colors
        vertices.color.r = 1.0f;
        vertices.color.g = 0.0f;
        vertices.color.b = 0.0f;

        edges.color.r = 0.1f;
        edges.color.g = 0.5f;
        edges.color.b = 1.0f;

        vertices.color.a = 1.0f;
        edges.color.a = 0.3f;

        //assignment
        int num = 0;
        geometry_msgs::Point point;
        for (const auto &node: graph_.getAllVertices()) {
            point.x = node.x();
            point.y = node.y();
            point.z = node.z();
            vertices.points.push_back(point);
            num++;
        }
        ROS_INFO("the graph vertices num is %i", num);

        int number = 0;
        for (int i = 0; i < graph_.getAllEdges().size(); ++i) {
            for (const auto &end_point: graph_.getAllEdges()[i]) {
                auto s_vp = graph_.getVertex(i);
                auto t_vp = graph_.getVertex(end_point);
                point.x = s_vp.x();
                point.y = s_vp.y();
                point.z = s_vp.z();
                edges.points.push_back(point);
                point.x = t_vp.x();
                point.y = t_vp.y();
                point.z = t_vp.z();
                edges.points.push_back(point);
                number++;
            }

        }
        ROS_INFO("the graph edges num is %i ", number);

        visualization_msgs::MarkerArray graph_markers;
        graph_markers.markers.resize(2);
        graph_markers.markers[0] = vertices;
        graph_markers.markers[1] = edges;

        return graph_markers;
    }

    void TopoGraph::pubGraphMarkers() const {
        graph_pub_.publish(generateTopoGraphMarkers());
    }
}
