//
// Created by hjl on 2022/1/9.
//

#include "perception/ufomap.h"

namespace perception {

    Ufomap::Ufomap(ros::NodeHandle &nh, ros::NodeHandle &nh_private) :
            nh_(nh), nh_private_(nh_private), tf_listener_(tf_buffer_),
            transform_timeout_(0, 100000000), known_plane_cell_num_(0),
            lidar_(nh_private_.param("UFOMap/no_ray_range", 50)),
            map_(nh_private_.param("UFOMap/resolution", 0.1),
                 nh_private_.param("UFOMap/depth_levels", 16), true) {
        setParametersFromROS();

        ros::Duration(0.5).sleep();

        start_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count()) / 1000000;

        fout.open(txt_known_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "record the known space size, starting at " << start_time << "\n" << "now time" << "\t"
             << " total elapsed time(s)" << "\t" << "known node num \t" << "known area(m2 or m3)" << std::endl;
        fout.close();

        frontier_iteration = 0;
        sum_frontier_time = 0;
        fout.open(txt_frontier_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "record the each frontier detected time" << "\n" << "start_time" << "\t" << "end_time" << "\t"
             << "elapsed time(s)" << "\t" << "detected_iteration" << "\t" << "average_time(s)" << std::endl;
        fout.close();

        insert_num = 0;
        sum_insert_time = 0;
        fout.open(txt_insert_cloud_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "record the each insert cloud time" << "\n" << "start_time" << "\t" << "end_time" << "\t"
             << "elapsed time(s)" << "\t" << "insert iteration" << "\t" << "average_time(s)" << std::endl;
        fout.close();

        map_.enableChangeDetection(true);

        point_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "point_cloud", 1));
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "sensor_odometry", 1000));
        sync_point_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyLocalCloudOdom>(
                SyncPolicyLocalCloudOdom(100), *point_cloud_sub_, *odom_sub_));
        sync_point_cloud_odom_->registerCallback(boost::bind(&Ufomap::pointCloudOdomCallback, this, _1, _2));

        voxel_filter_.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);

        write_ufomap_timer_ = nh_private_.createTimer(ros::Duration(2.0), &Ufomap::writeUfomapCallback, this);

        if (0 < pub_rate_) {
            map_pub_ = nh_private_.advertise<ufomap_msgs::UFOMapStamped>("ufomap", 1);
            cloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("ufomap_cloud", 1);
            map_and_frontiers_pub_ = nh_private_.advertise<ufomap_manager::UfomapWithFrontiers>("ufomap_and_frontiers",
                                                                                                1);
            pub_timer_ = nh_private_.createTimer(ros::Rate(pub_rate_), &Ufomap::ufomapPublishTimer, this);
        }

        expand_cloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("expand_cloud", 1);

        local_frontiers_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("local_frontier_marker", 1);
        global_frontiers_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("global_frontier_marker", 1);
        ploygon_pub_ = nh_private_.advertise<geometry_msgs::PolygonStamped>("polygon",1);

        ROS_INFO("frontier_manager construct finish");
    }

    void Ufomap::setParametersFromROS() {
        std::string ns = ros::this_node::getName() + "/UFOMap";

        std::string pkg_path = ros::package::getPath("planner");
        std::string txt_path = pkg_path + "/../../files/exploration_data/";

        txt_known_name          = txt_path + "known_node_num.txt";
        txt_frontier_name       = txt_path + "frontier_detected_time.txt";
        map_txt_name            = txt_path + "ufomap.txt";
        txt_insert_cloud_name   = txt_path + "insert_cloud.txt";
   
        frame_id_ = "world";
        if (!ros::param::get(ns + "/frame_id", frame_id_)) {
            ROS_WARN("No frame_id specified. Looking for %s. Default is 'map'.",
                     (ns + "/frame_id").c_str());
        }

        robot_base_frame_id_ = "base_link";
        if (!ros::param::get(ns + "/robot_base_frame_id", robot_base_frame_id_)) {
            ROS_WARN("No robot_base_frame_id specified. Looking for %s. Default is 'base_link'.",
                     (ns + "/robot_base_frame_id").c_str());
        }

        max_range_ = 7;
        if (!ros::param::get(ns + "/max_range", max_range_)) {
            ROS_WARN("No max_range specified. Looking for %s. Default is '7'.",
                     (ns + "/max_range").c_str());
        }

        insert_discrete_ = false;
        if (!ros::param::get(ns + "/insert_discrete", insert_discrete_)) {
            ROS_WARN("No insert_discrete specified. Looking for %s. Default is 'false'.",
                     (ns + "/insert_discrete").c_str());
        }

        insert_depth_ = 0;
        if (!ros::param::get(ns + "/insert_depth", insert_depth_)) {
            ROS_WARN("No insert_depth specified. Looking for %s. Default is '0'.",
                     (ns + "/insert_depth").c_str());
        }

        simple_ray_casting_ = false;
        if (!ros::param::get(ns + "/simple_ray_casting", simple_ray_casting_)) {
            ROS_WARN("No simple_ray_casting specified. Looking for %s. Default is 'false'.",
                     (ns + "/simple_ray_casting").c_str());
        }

        early_stopping_ = 0;
        if (!ros::param::get(ns + "/early_stopping", early_stopping_)) {
            ROS_WARN("No early_stopping specified. Looking for %s. Default is '0'.",
                     (ns + "/early_stopping").c_str());
        }

        clear_robot_enabled_ = true;
        if (!ros::param::get(ns + "/clear_robot_enabled", clear_robot_enabled_)) {
            ROS_WARN("No clear_robot_enabled_ specified. Looking for %s. Default is 'true'.",
                     (ns + "/clear_robot_enabled_").c_str());
        }

        robot_height_ = 0.2;
        if (!ros::param::get(ns + "/robot_height", robot_height_)) {
            ROS_WARN("No robot_height specified. Looking for %s. Default is '0.2m'.",
                     (ns + "/robot_height").c_str());
        }

        robot_bottom_ = 0;
        if (!ros::param::get(ns + "/robot_bottom", robot_bottom_)) {
            ROS_WARN("No robot_bottom specified. Looking for %s. Default is '0 m'.",
                     (ns + "/robot_bottom_").c_str());
        }

        sensor_height_ = 0.5;
        if (!ros::param::get(ns + "/sensor_height", sensor_height_)) {
            ROS_WARN("No sensor_height specified. Looking for %s. Default is '0.5m'.",
                     (ns + "/sensor_height_").c_str());
        }

        frontier_depth_ = 0;
        if (!ros::param::get(ns + "/frontier_depth", frontier_depth_)) {
            ROS_WARN("No frontier_depth specified. Looking for %s. Default is '0'.",
                     (ns + "/frontier_depth").c_str());
        }

        pub_rate_ = 10;
        if (!ros::param::get(ns + "/pub_rate", pub_rate_)) {
            ROS_WARN("No pub_rate specified. Looking for %s. Default is '10hz'.",
                     (ns + "/pub_rate").c_str());
        }

        scan_voxel_size_ = 0.1;
        if (!ros::param::get(ns + "/scan_voxel_size", scan_voxel_size_)) {
            ROS_WARN("No scan_voxel_size specified. Looking for %s. Default is '0.1'.",
                     (ns + "/scan_voxel_size").c_str());
        }

        min_x_ = 0.0;
        if (!ros::param::get(ns + "/min_x", min_x_)) {
            ROS_WARN("No min_x specified. Looking for %s. Default is '0.0'.",
                     (ns + "/min_x").c_str());
        }

        min_y_ = 0.0;
        if (!ros::param::get(ns + "/min_y", min_y_)) {
            ROS_WARN("No min_y specified. Looking for %s. Default is '0.0'.",
                     (ns + "/min_y").c_str());
        }

        max_x_ = 100.0;
        if (!ros::param::get(ns + "/max_x", max_x_)) {
            ROS_WARN("No max_x specified. Looking for %s. Default is '100.0'.",
                     (ns + "/max_x").c_str());
        }

        max_y_ = 100.0;
        if (!ros::param::get(ns + "/max_y", max_y_)) {
            ROS_WARN("No max_y specified. Looking for %s. Default is '100.0'.",
                     (ns + "/max_y").c_str());
        }
    }

    void Ufomap::pointCloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &scan,
                                        const nav_msgs::OdometryConstPtr &odom) {
        sensor_frame_id_ = odom->child_frame_id;

        current_sensor_pose_.translation().x() = odom->pose.pose.position.x;
        current_sensor_pose_.translation().y() = odom->pose.pose.position.y;
        current_sensor_pose_.translation().z() = odom->pose.pose.position.z;
        current_sensor_pose_.rotation().w() = odom->pose.pose.orientation.w;
        current_sensor_pose_.rotation().x() = odom->pose.pose.orientation.x;
        current_sensor_pose_.rotation().y() = odom->pose.pose.orientation.y;
        current_sensor_pose_.rotation().z() = odom->pose.pose.orientation.z;

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*scan, *scan_cloud);

        map_mutex_.lock();
        insert_num++;
        auto insert_start_time = ros::WallTime::now();

        voxel_filter_.setInputCloud(scan_cloud);
        pcl::PointCloud<pcl::PointXYZ> downsize_scan_cloud;
        voxel_filter_.filter(downsize_scan_cloud);

        ufo::map::PointCloud downsize_cloud;
        for (auto &point: downsize_scan_cloud) {
            downsize_cloud.push_back(ufo::map::Point3(point.x, point.y, point.z));
        }
        downsize_cloud.transform(current_sensor_pose_);

        ufo::map::KeySet key_set;
        ufo::map::PointCloud cloud;
        for (const auto &point: downsize_cloud) {
            if (key_set.insert(map_.toKey(point, 0)).second) {
                cloud.push_back(map_.toCoord(map_.toKey(point, 0), 0));  
            }
        }
        if (insert_discrete_) {
            map_.insertPointCloudDiscrete(current_sensor_pose_.translation(), cloud, max_range_, insert_depth_,
                                          simple_ray_casting_, early_stopping_, false);
        } else {
            map_.insertPointCloud(current_sensor_pose_.translation(), cloud, max_range_, insert_depth_,
                                  simple_ray_casting_, early_stopping_, false);
        }
        auto insert_end_time = ros::WallTime::now();

        sum_insert_time = sum_insert_time + (insert_end_time - insert_start_time).toSec();
        fout.open(txt_insert_cloud_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << insert_start_time << "\t" << insert_end_time << "\t" << (insert_end_time - insert_start_time).toSec()
             << "\t" << insert_num << "\t" << sum_insert_time / insert_num << "s \t" << std::endl;
        fout.close();

        try {
            geometry_msgs::TransformStamped T_s_b = tf_buffer_.lookupTransform(sensor_frame_id_, robot_base_frame_id_,
                                                                               odom->header.stamp);
            ufo::math::Pose6 T_S_B = ufomap_ros::rosToUfo(T_s_b.transform);
            current_robot_pose_ = current_sensor_pose_ * T_S_B;

            if (clear_robot_enabled_) {
                ufo::map::Point3 robot_bbx_min(current_robot_pose_.x() - sensor_height_,
                                               current_robot_pose_.y() - sensor_height_,
                                               current_robot_pose_.z() - robot_bottom_);
                ufo::map::Point3 robot_bbx_max(current_robot_pose_.x() + sensor_height_,
                                               current_robot_pose_.y() + sensor_height_,
                                               current_robot_pose_.z() + robot_height_);
                ufo::geometry::AABB aabb(robot_bbx_min, robot_bbx_max);  
                map_.setValueVolume(aabb, map_.getClampingThresMin(), clearing_depth_);
            }
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("ufomap with frontier current pose acquire---      %s ", ex.what());
        }
        map_mutex_.unlock();
    }

    void Ufomap::ufomapPublishTimer(const ros::TimerEvent &event) {
        map_mutex_.lock();
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = frame_id_;

        ufomap_msgs::UFOMapStamped msg;
        ufomap_msgs::ufoToMsg(map_, msg.map, false);
        msg.header = header;
        map_pub_.publish(msg);//publish map

        if (0 < cloud_pub_.getNumSubscribers() || cloud_pub_.isLatched()) {
            ufo::map::PointCloud cloud;
            for (auto it = map_.beginLeaves(true, false, false, false, 0),
                         it_end = map_.endLeaves();
                 it != it_end; ++it) {
                cloud.push_back(it.getCenter());
            }
            sensor_msgs::PointCloud2 cloud_msg;
            ufomap_ros::ufoToRos(cloud, cloud_msg);
            cloud_msg.header = header;
            cloud_pub_.publish(cloud_msg);
        }

        statisticAndPubMarkers();

        geometry_msgs::PolygonStamped polygon;
        polygon.header.frame_id = frame_id_;
        polygon.header.stamp = ros::Time::now();
        geometry_msgs::Point32 point_1;
        point_1.x = min_x_;
        point_1.y = min_y_;
        point_1.z = current_sensor_pose_.z();
        polygon.polygon.points.push_back(point_1);

        geometry_msgs::Point32 point_2;
        point_2.x = max_x_;
        point_2.y = min_y_;
        point_2.z = current_sensor_pose_.z();
        polygon.polygon.points.push_back(point_2);

        geometry_msgs::Point32 point_3;
        point_3.x = max_x_;
        point_3.y = max_y_;
        point_3.z = current_sensor_pose_.z();
        polygon.polygon.points.push_back(point_3);

        geometry_msgs::Point32 point_4;
        point_4.x = min_x_;
        point_4.y = max_y_;
        point_4.z = current_sensor_pose_.z();
        polygon.polygon.points.push_back(point_4);

        ploygon_pub_.publish(polygon);

        map_mutex_.unlock();
    }

    void Ufomap::frontierCallback(const ros::TimerEvent &event) {

        frontier_iteration++;
        auto s_time = ros::WallTime::now();

        frontierSearch();

        auto finish_time = ros::WallTime::now();

        sum_frontier_time = sum_frontier_time + (finish_time - s_time).toSec();
        fout.open(txt_frontier_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << s_time << "\t" << finish_time << "\t" << (finish_time - s_time).toSec()
             << "\t" << frontier_iteration << "\t" << sum_frontier_time / frontier_iteration << "s \t" << std::endl;
        fout.close();

        knownCellOutput();

        std_msgs::ColorRGBA global_rgba;
        global_rgba.a = 0.3;
        global_rgba.r = 0;
        global_rgba.g = 1;
        global_rgba.b = 0;
        visualization_msgs::MarkerArray global_frontier_cells;
        generateMarkerArray(frame_id_, &global_frontier_cells, global_frontier_cells_, global_rgba);
        global_frontiers_pub_.publish(global_frontier_cells);

        std_msgs::ColorRGBA local_rgba;
        local_rgba.a = 0.3;
        local_rgba.r = 1;
        local_rgba.g = 0;
        local_rgba.b = 0;
        visualization_msgs::MarkerArray local_frontier_cells;
        generateMarkerArray(frame_id_, &local_frontier_cells, local_frontier_cells_, local_rgba);
        local_frontiers_pub_.publish(local_frontier_cells);

        // map and frontiers pub..
        ufomap_manager::UfomapWithFrontiers ufomap_with_frontiers;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = frame_id_;
        ufomap_with_frontiers.header = header;

        ufomap_with_frontiers.known_plane_cell_num = known_plane_cell_num_;
        ufomap_with_frontiers.max_range = max_range_;
        ufomap_with_frontiers.frame_id = frame_id_;
        ufomap_with_frontiers.robot_base_frame_id = robot_base_frame_id_;
        ufomap_with_frontiers.robot_height = robot_height_;
        ufomap_with_frontiers.robot_bottom = robot_bottom_;
        ufomap_with_frontiers.robot_radius = sensor_height_;

        for (auto &item: local_frontier_cells_) {
            ufomap_manager::CellCode local_frontier;
            local_frontier.depth = item.getDepth();
            local_frontier.Code = item.getCode();
            ufomap_with_frontiers.local_frontiers.push_back(local_frontier);
        }

        for (auto &item: global_frontier_cells_) {
            ufomap_manager::CellCode global_frontier;
            global_frontier.depth = item.getDepth();
            global_frontier.Code = item.getCode();
            ufomap_with_frontiers.global_frontiers.push_back(global_frontier);
        }

        for (auto &item: changed_cell_codes_) {
            ufomap_manager::CellCode changed_cell_code;
            changed_cell_code.depth = item.getDepth();
            changed_cell_code.Code = item.getCode();
            ufomap_with_frontiers.changed_cell_codes.push_back(changed_cell_code);
        }

        for (auto &item: known_cell_codes_) {  
            if (0 == item.getDepth()) {
                ufomap_manager::CellCode known_cell_code;
                known_cell_code.depth = item.getDepth();
                known_cell_code.Code = item.getCode();
                ufomap_with_frontiers.known_cell_codes.push_back(known_cell_code);
            }
        }

        ufomap_msgs::UFOMap ufomap;
        ufomap_msgs::ufoToMsg(map_, ufomap, false);
        ufomap_with_frontiers.ufomap = ufomap;

        map_and_frontiers_pub_.publish(ufomap_with_frontiers);
    }

    void Ufomap::frontierSearch() {
        changed_cell_codes_.clear();
        for (auto it = map_.changesBegin(); it != map_.changesEnd(); it++) {
            changed_cell_codes_.insert(*it);
        }

        known_cell_codes_.insert(changed_cell_codes_.begin(), changed_cell_codes_.end());

        map_.resetChangeDetection();

        findPlaneLocalFrontier();
        updatePlaneGlobalFrontier();
    }

    void Ufomap::writeUfomapCallback(const ros::TimerEvent &event) {
        map_.write(map_txt_name);
    }


    void Ufomap::findLocalFrontier() {
        local_frontier_cells_.clear();

        for (const auto &iter_cell: changed_cell_codes_) {
            if (frontier_depth_ == iter_cell.getDepth()) {
                ufo::map::Point3 iter_coord = map_.toCoord(iter_cell.toKey());
                double sensor_dist_xy = iter_coord.distanceXY(current_sensor_pose_.translation());
                double robot_dist_xy = iter_coord.distanceXY(current_robot_pose_.translation());
                if (robot_dist_xy > 0.6 && fabs(iter_coord.z() - current_sensor_pose_.z()) / sensor_dist_xy <
                                           tan(M_PI * 15 / 180)) {  
                    if (isFrontier(iter_cell)) {
                        local_frontier_cells_.insert(iter_cell);
                    }
                }

            }
        }
        changed_cell_codes_.clear();
    }

    void Ufomap::updateGlobalFrontier() {

        CodeUnorderSet global_frontiers;
        if (!global_frontier_cells_.empty()) {
            for (const auto &cell_iter: global_frontier_cells_) {
                ufo::map::Point3 cell_coord = map_.toCoord(cell_iter.toKey());
                double sensor_dist_xy = cell_coord.distanceXY(current_sensor_pose_.translation());
                if (sensor_dist_xy < max_range_ + 1.0) {  
                    if (fabs(cell_coord.z() - current_sensor_pose_.z()) / sensor_dist_xy < tan(M_PI * 15 / 180)
                        && isFrontier(cell_iter)) {  
                        global_frontiers.insert(cell_iter);
                    }
                } else {
                    global_frontiers.insert(cell_iter);
                }
            }
        }
        global_frontier_cells_ = global_frontiers;

        for (const auto &iter_cell: local_frontier_cells_) {  
            global_frontier_cells_.insert(iter_cell);
        }
    }

    CodeUnorderSet Ufomap::get_3D_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth) {
        CodeUnorderSet neighbors;
        ufo::map::Point3 cell_center = map_.toCoord(cell_code.toKey(), depth);

        std::vector<double> coord_x{cell_center.x() - map_.getResolution(), cell_center.x(),
                                    cell_center.x() + map_.getResolution()};
        std::vector<double> coord_y{cell_center.y() - map_.getResolution(), cell_center.y(),
                                    cell_center.y() + map_.getResolution()};
        std::vector<double> coord_z{cell_center.z() - map_.getResolution(), cell_center.z(),
                                    cell_center.z() + map_.getResolution()};

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    if (i != 1 && j != 1 && k != 1) {  
                        neighbors.insert(
                                ufo::map::Code(map_.toKey(coord_x[i], coord_y[j], coord_z[k], depth)));
                    }
                }
            }
        }
        return neighbors;
    }

    void Ufomap::findPlaneLocalFrontier() {
        local_frontier_cells_.clear();
        for (const auto &changedCellCode: changed_cell_codes_) {
            if (frontier_depth_ == changedCellCode.getDepth() && history_frontier_cells_.count(changedCellCode)==0) {  
                ufo::map::Point3 point = map_.toCoord(changedCellCode.toKey());
                if (isInExplorationArea(point.x(),point.y()) &&
                    point.z() > current_robot_pose_.z() + robot_bottom_ &&
                    point.z() < current_robot_pose_.z() + robot_height_) {
                    odom_mutex_.lock();
                    ufo::math::Vector3 current = current_robot_pose_.translation();
                    odom_mutex_.unlock();
                    if (point.distanceXY(current) > 0.6) {    
                        if (isFrontier(changedCellCode)) {
                            local_frontier_cells_.insert(changedCellCode);
                            history_frontier_cells_.insert(changedCellCode);
                        }
                    }
                }

            }
        }
    }

    void Ufomap::updatePlaneGlobalFrontier() {
        CodeUnorderSet global_frontiers;
        if (!global_frontier_cells_.empty()) {
            for (const auto &cell_iter: global_frontier_cells_) {  
                ufo::map::Point3 point = map_.toCoord(cell_iter.toKey());
                if (point.distanceXY(current_robot_pose_.translation()) < max_range_ + 1.0) {
                    if (isFrontier(cell_iter))
                        global_frontiers.insert(cell_iter);
                } else {
                    global_frontiers.insert(cell_iter);
                }
            }
        }
        global_frontier_cells_ = global_frontiers;
        //merge the local frontiers
        for (const auto &iter_cell: local_frontier_cells_) {
            global_frontier_cells_.insert(iter_cell);
        }
    }


    CodeUnorderSet
    Ufomap::get_XY_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth) const {
        CodeUnorderSet changed_cell_neighbor;
        ufo::map::Point3 cell_center = map_.toCoord(cell_code.toKey(), depth);

        changed_cell_neighbor.insert(ufo::map::Code(
                map_.toKey(cell_center.x() - map_.getResolution(), cell_center.y(), cell_center.z(), depth)));
        changed_cell_neighbor.insert(ufo::map::Code(
                map_.toKey(cell_center.x() + map_.getResolution(), cell_center.y(), cell_center.z(), depth)));
        changed_cell_neighbor.insert(ufo::map::Code(
                map_.toKey(cell_center.x(), cell_center.y() - map_.getResolution(), cell_center.z(), depth)));
        changed_cell_neighbor.insert(ufo::map::Code(
                map_.toKey(cell_center.x(), cell_center.y() + map_.getResolution(), cell_center.z(), depth)));

        return changed_cell_neighbor;
    }

    CodeUnorderSet
    Ufomap::get_Z_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth) const {
        CodeUnorderSet changed_cell_neighbor;
        ufo::map::Point3 cell_center = map_.toCoord(cell_code.toKey(), depth);

        changed_cell_neighbor.insert(ufo::map::Code(
                map_.toKey(cell_center.x(), cell_center.y(), cell_center.z() - map_.getResolution(), depth)));
        changed_cell_neighbor.insert(ufo::map::Code(
                map_.toKey(cell_center.x(), cell_center.y(), cell_center.z() + map_.getResolution(), depth)));

        return changed_cell_neighbor;
    }


    bool Ufomap::isFrontier(const ufo::map::Code &frontier) const {   

        if (map_.isFree(frontier)) {
            CodeUnorderSet xy_neighbor_cells = get_XY_NeighborCell(frontier, frontier.getDepth());  
            bool unknowFlag = false;
            bool occupiedFlag = false;
            for (const auto &iter: xy_neighbor_cells) {
                if (map_.isUnknown(iter)) {
                    unknowFlag = true;
                }
            }
            if (unknowFlag && !occupiedFlag) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }

    }

    bool Ufomap::isInExplorationArea(const double &point_x, const double & point_y) const {
        if (point_x < min_x_ + 1e-4 || point_x > max_x_ - 1e-4 ||
            point_y < min_y_ + 1e-4 || point_y > max_y_ - 1e-4) {
            return false;
        } else {
            return true;
        }
    }

    void Ufomap::knownCellOutput() {

        known_cell_num_ = getKnownNodeNum(known_cell_codes_);
        known_plane_cell_num_ = getKnownPlaneNodeNum(known_cell_codes_);

        double current_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count()) / 1000000;
        fout.open(txt_known_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << current_time << "\t" << (current_time - start_time) << "\t" << known_plane_cell_num_
             << "\t" << known_plane_cell_num_ * map_.getResolution() * map_.getResolution() << std::endl;
        fout.close();
    }


    std::size_t Ufomap::getKnownNodeNum(const CodeUnorderSet &knownCellCodes) {
        std::size_t size = 0;
        for (const auto &iter: knownCellCodes) {
            if (iter.getDepth() == 0) {
                size++;
            }
        }
        return size;
    }

    std::size_t Ufomap::getKnownPlaneNodeNum(const CodeUnorderSet &knownCellCodes) {
        std::size_t size = 0;
        for (const auto &iter: knownCellCodes) {
            if (iter.getDepth() == 0 &&
                map_.toCoord(iter.toKey()).z() > 0 &&
                map_.toCoord(iter.toKey()).z() < map_.getResolution()) {
                size++;
            }
        }
        return size;
    }

    void
    Ufomap::generateMarkerArray(const std::string &tf_frame, visualization_msgs::MarkerArray *frontier_cells,
                                CodeUnorderSet &frontier_cell_codes, std_msgs::ColorRGBA &rgba) {

        auto tree_depth = map_.getTreeDepthLevels();
        frontier_cells->markers.resize(tree_depth);
        for (int i = 0; i < tree_depth; ++i) {
            double size = map_.getNodeSize(i);
            frontier_cells->markers[i].header.frame_id = tf_frame;
            frontier_cells->markers[i].ns = frame_id_;
            frontier_cells->markers[i].id = i;
            frontier_cells->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            frontier_cells->markers[i].scale.x = size;
            frontier_cells->markers[i].scale.y = size;
            frontier_cells->markers[i].scale.z = size;
            frontier_cells->markers[i].pose.orientation.w = 1;
        }

        for (const auto &iter: frontier_cell_codes) {
            geometry_msgs::Point cube_center;
            cube_center.x = map_.toCoord(iter.toKey()).x();
            cube_center.y = map_.toCoord(iter.toKey()).y();
            cube_center.z = map_.toCoord(iter.toKey()).z();

            auto depth_level = iter.getDepth();
            frontier_cells->markers[depth_level].points.push_back(cube_center);

            frontier_cells->markers[depth_level].colors.push_back(rgba);
        }

        for (int i = 0; i < tree_depth; ++i) {
            if (!frontier_cells->markers[i].points.empty()) {
                frontier_cells->markers[i].action = visualization_msgs::Marker::ADD;
            } else {
                frontier_cells->markers[i].action = visualization_msgs::Marker::DELETE;
            }
        }


    }

    void Ufomap::frontierUpdate() {
        frontier_iteration++;
        auto s_time = ros::WallTime::now();

        frontierSearch();

        auto finish_time = ros::WallTime::now();

        sum_frontier_time = sum_frontier_time + (finish_time - s_time).toSec();
        fout.open(txt_frontier_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << s_time << "\t" << finish_time << "\t" << (finish_time - s_time).toSec()
             << "\t" << frontier_iteration << "\t" << sum_frontier_time / frontier_iteration << "s \t" << std::endl;
        fout.close();
    }


    void Ufomap::statisticAndPubMarkers(){

        knownCellOutput();

        std_msgs::ColorRGBA global_rgba;
        global_rgba.a = 0.3;
        global_rgba.r = 0;
        global_rgba.g = 1;
        global_rgba.b = 0;
        visualization_msgs::MarkerArray global_frontier_cells;
        generateMarkerArray(frame_id_, &global_frontier_cells, global_frontier_cells_, global_rgba);
        global_frontiers_pub_.publish(global_frontier_cells);

        std_msgs::ColorRGBA local_rgba;
        local_rgba.a = 0.3;
        local_rgba.r = 1;
        local_rgba.g = 0;
        local_rgba.b = 0;
        visualization_msgs::MarkerArray local_frontier_cells;
        generateMarkerArray(frame_id_, &local_frontier_cells, local_frontier_cells_, local_rgba);
        local_frontiers_pub_.publish(local_frontier_cells);

        // map and frontiers pub..
        ufomap_manager::UfomapWithFrontiers ufomap_with_frontiers;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = frame_id_;
        ufomap_with_frontiers.header = header;

        ufomap_with_frontiers.known_plane_cell_num = known_plane_cell_num_;
          
        ufomap_with_frontiers.max_range = max_range_;
        ufomap_with_frontiers.frame_id = frame_id_;
        ufomap_with_frontiers.robot_base_frame_id = robot_base_frame_id_;
        ufomap_with_frontiers.robot_height = robot_height_;
        ufomap_with_frontiers.robot_bottom = robot_bottom_;
        ufomap_with_frontiers.robot_radius = sensor_height_;

        for (auto &item: local_frontier_cells_) {
            ufomap_manager::CellCode local_frontier;
            local_frontier.depth = item.getDepth();
            local_frontier.Code = item.getCode();
            ufomap_with_frontiers.local_frontiers.push_back(local_frontier);
        }

        for (auto &item: global_frontier_cells_) {
            ufomap_manager::CellCode global_frontier;
            global_frontier.depth = item.getDepth();
            global_frontier.Code = item.getCode();
            ufomap_with_frontiers.global_frontiers.push_back(global_frontier);
        }

        for (auto &item: changed_cell_codes_) {
            ufomap_manager::CellCode changed_cell_code;
            changed_cell_code.depth = item.getDepth();
            changed_cell_code.Code = item.getCode();
            ufomap_with_frontiers.changed_cell_codes.push_back(changed_cell_code);
        }

        for (auto &item: known_cell_codes_) {  
            if (0 == item.getDepth()) {
                ufomap_manager::CellCode known_cell_code;
                known_cell_code.depth = item.getDepth();
                known_cell_code.Code = item.getCode();
                ufomap_with_frontiers.known_cell_codes.push_back(known_cell_code);
            }
        }

        ufomap_msgs::UFOMap ufomap;
        ufomap_msgs::ufoToMsg(map_, ufomap, false);
        ufomap_with_frontiers.ufomap = ufomap;

        map_and_frontiers_pub_.publish(ufomap_with_frontiers);

    }
}