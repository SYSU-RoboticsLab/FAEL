//
// Created by hjl on 2022/6/13.
//

#ifndef ROBO_PLANNER_WS_TEST_NEW_PLAN_GRAPH_H
#define ROBO_PLANNER_WS_TEST_NEW_PLAN_GRAPH_H

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <queue>

#include "utils/point3d.h"
#include "graph/kdtree.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace graph {
    using namespace std;

    class PlanGraph {
        
    public:
        typedef shared_ptr<PlanGraph> Ptr;

        explicit PlanGraph();

        void clearGraph();

        //add and remove elements;
        int addVertex(const utils::Point3D &point);

        bool addOneWayEdge(const int &a_id, const int &b_id, const double &distance);

        bool addTwoWayEdge(const int &a_id, const int &b_id);

        bool isPoint3DExisted(const utils::Point3D &point) const;

        // extraction
        utils::Point3D getVertex(const int &id) const;

        const std::vector<utils::Point3D> &getAllVertices() const;

        const std::vector<std::vector<int>> &getAllEdges() const;

        bool getPointId(const utils::Point3D &point, int &point_id);

        void updateKdtree();

        int getNearestVertexId(pcl::KdTreeFLANN<pcl::PointXYZ> &kd_tree,
                               const utils::Point3D &point); 

        std::vector<int> getNeighborVertexsIDs(pcl::KdTreeFLANN<pcl::PointXYZ> &kd_tree, const utils::Point3D &point,
                                               const double &range, std::vector<utils::Point3D> &neighbor_vertexs);

        bool getShortestPath(const int &start_v_id, const int &end_v_id, std::vector<int> &waypoint_ids,
                             utils::Point3DQueue &shortest_path) const;

        bool A_star_search(const int &start_v_id, const int &end_v_id, std::vector<int> &waypoint_ids) const;

        pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_;

    private:
        bool vertexIndexInRange(const int &index) const {
            return index >= 0 && index < vertices_.size();
        }

        std::vector<utils::Point3D> vertices_;  // vertex  positions
        std::vector<std::vector<int>> edges_;   //  edges.
        // Distances between two vertexs corresponding to edges.
        std::vector<std::vector<double>> distance_;
        utils::Point3DMap<int> points_ids_;     // store the ids of points.

        pcl::PointCloud<pcl::PointXYZ> points_;


    };
}

#endif //ROBO_PLANNER_WS_TEST_NEW_PLANGRAPH_H
