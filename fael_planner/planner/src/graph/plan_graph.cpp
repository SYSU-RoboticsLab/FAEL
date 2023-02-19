//
// Created by hjl on 2022/6/13.
//

#include "graph/plan_graph.h"

namespace graph {

    PlanGraph::PlanGraph() {
        vertices_.clear();
        edges_.clear();
        distance_.clear();
        points_ids_.clear();
    }

    void PlanGraph::clearGraph() {
        vertices_.clear();
        edges_.clear();
        distance_.clear();
        points_ids_.clear();
    }

    int PlanGraph::addVertex(const utils::Point3D &point) {
        vertices_.push_back(point);
        points_.emplace_back(point.x(), point.y(), point.z());
        std::vector<int> connection;
        edges_.push_back(connection);
        std::vector<double> edges_distance;
        distance_.push_back(edges_distance);
        points_ids_[point] = vertices_.size() - 1;

        return vertices_.size() - 1;
    }

    bool PlanGraph::addOneWayEdge(const int &a_id, const int &b_id, const double &distance) {
        if (vertexIndexInRange(a_id) && vertexIndexInRange((b_id)) && a_id != b_id
            && std::count(edges_[a_id].begin(), edges_[a_id].end(), b_id) == 0) {
            edges_[a_id].push_back(b_id);
            distance_[a_id].push_back(distance);
            return true;
        } else {
            return false;
        }
    }

    bool PlanGraph::addTwoWayEdge(const int &a_id, const int &b_id) {
        double distance = vertices_[a_id].distance(vertices_[b_id]);
        if (addOneWayEdge(a_id, b_id, distance) && addOneWayEdge(b_id, a_id, distance)) {
            return true;
        } else {
            return false;
        }
    }

    bool PlanGraph::isPoint3DExisted(const utils::Point3D &point) const {
        if (points_ids_.count(point) != 0)
            return true;
        else
            return false;
    }

    utils::Point3D PlanGraph::getVertex(const int &id) const {
        if (vertexIndexInRange(id))
            return vertices_[id];
        else
            return utils::Point3D();
    }

    const std::vector<utils::Point3D> &PlanGraph::getAllVertices() const {
        return vertices_;
    }

    const std::vector<std::vector<int>> &PlanGraph::getAllEdges() const {
        return edges_;
    }

    bool PlanGraph::getPointId(const utils::Point3D &point, int &point_id) {
        if (points_ids_.count(point) != 0) {
            point_id = points_ids_[point];
            return true;
        } else
            return false;
    }

    void PlanGraph::updateKdtree() {
        kd_tree_.setInputCloud(points_.makeShared());
    }

    int PlanGraph::getNearestVertexId(pcl::KdTreeFLANN<pcl::PointXYZ> &kd_tree, const utils::Point3D &point) {
        if (!points_.empty()) {
            int k = 1;
            pcl::PointXYZ center(point.x(), point.y(), point.z());
            std::vector<int> neighbors_ids;
            std::vector<float> neighbors_distance;
            kd_tree.nearestKSearch(center, k, neighbors_ids, neighbors_distance);
            utils::Point3D nearest_point(points_[neighbors_ids.front()].x, points_[neighbors_ids.front()].y,
                                         points_[neighbors_ids.front()].z);
            int nearest_point_id = -1;
            if (getPointId(nearest_point, nearest_point_id)) {
                return nearest_point_id;
            } else {
                return -1;
            }
        } else
            return -1;
    }

    std::vector<int>
    PlanGraph::getNeighborVertexsIDs(pcl::KdTreeFLANN<pcl::PointXYZ> &kd_tree, const utils::Point3D &point,
                                     const double &range, std::vector<utils::Point3D> &neighbor_vertexs) {
        pcl::PointXYZ center(point.x(), point.y(), point.z());
        std::vector<int> neighbors_ids;
        std::vector<float> neighbors_distance;
        kd_tree.radiusSearch(center, range, neighbors_ids, neighbors_distance);

        std::vector<int> vertexs_ids;
        neighbor_vertexs.clear();
        int id = -1;
        utils::Point3D neigbor_point;
        for (const auto &item: neighbors_ids) {
            neigbor_point.x() = points_[item].x;
            neigbor_point.y() = points_[item].y;
            neigbor_point.z() = points_[item].z;
            if (getPointId(neigbor_point, id)) {
                vertexs_ids.push_back(id);
                neighbor_vertexs.push_back(neigbor_point);
            }
        }
        return vertexs_ids;
    }

    bool PlanGraph::getShortestPath(const int &start_v_id, const int &end_v_id, vector<int> &waypoint_ids,
                                    utils::Point3DQueue &shortest_path) const {
        std::cout << "start get shortest path in graph" << std::endl;
        waypoint_ids.clear();
        shortest_path.clear();
        if (vertexIndexInRange(start_v_id) && vertexIndexInRange(end_v_id)) {
            if (start_v_id == end_v_id) {
                std::cout << "start point = end point, return the start point" << std::endl;
                waypoint_ids.push_back(start_v_id);
                shortest_path.push_back(vertices_[start_v_id]);
                return false;
            } else {
                if (A_star_search(start_v_id, end_v_id, waypoint_ids)) {
                    for (const auto &id: waypoint_ids) {
                        shortest_path.push_back(vertices_[id]);
                    }
                    return true;

                } else {
                    return false;
                }
            }
        } else {
            std::cout << "start or end point is out of range" << std::endl;
            return false;
        }
    }

    bool PlanGraph::A_star_search(const int &start_v_id, const int &end_v_id, vector<int> &waypoint_ids) const {
        std::cout << "start A_star_search in graph" << std::endl;
        typedef std::pair<double, int> iPair; // Vertices are represented by their index in the graph.vertices list
        std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq; // Priority queue of vertices
        std::vector<double> dist(vertices_.size(), INFINITY);// Vector of distances
        std::vector<double> estimation(vertices_.size(), INFINITY);
        const int INF = 0x3f3f3f3f; // integer infinity
        std::vector<int> backpointers(vertices_.size(), INF);// Vector of backpointers
        std::vector<bool> in_pq(vertices_.size(), false);

        std::cout << "searching.." << std::endl;

        // Add the start vertex
        dist[start_v_id] = 0;
        estimation[start_v_id] = vertices_[start_v_id].distance(vertices_[end_v_id]);
        pq.push(std::make_pair(estimation[start_v_id], start_v_id));
        in_pq[start_v_id] = true;

        int u;
        int v;
        while (!pq.empty()) {// Loop until priority queue is empty
            u = pq.top().second;
            pq.pop();  // Pop the minimum distance vertex
            in_pq[u] = false;
            if (u == end_v_id) { // Early termination
                break;
            }
            for (int i = 0; i < edges_[u].size(); ++i) {// Get all adjacent vertices
                // Get vertex label and weight of current adjacent edge of u
                v = edges_[u][i];
                // If there is a shorter path to v through u
                if (dist[v] > dist[u] + distance_[u][i]) {
                    // Updating distance of v
                    dist[v] = dist[u] + distance_[u][i];
                    estimation[v] = dist[v] + vertices_[v].distance(vertices_[end_v_id]);
                    backpointers[v] = u;
                    if (!in_pq[v]) {
                        pq.push(std::make_pair(estimation[v], v));
                        in_pq[v] = true;
                    }
                }
            }
        }

        std::cout << "back pointer to path.." << std::endl;
        // Backtrack to find path
        waypoint_ids.clear();
        int current = end_v_id;
        if (backpointers[current] == INF) {// no path found
            std::cout << "no path found, return empty path" << std::endl;
            return false;
        } else {
            // path found
            while (current != INF) {
                waypoint_ids.push_back(current);
                current = backpointers[current];
            }
            // Reverse the path (constructing it this way since vector is more efficient
            // at push_back than insert[0])
            std::reverse(waypoint_ids.begin(), waypoint_ids.end());
            std::cout << "reverse a path. " << std::endl;
            std::cout << "A_star search finish." << std::endl;
            return true;
        }
    }
}
