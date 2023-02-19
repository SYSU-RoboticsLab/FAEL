//
// Created by hjl on 2020/6/15.
//
#include <ufomap_manager/ufomap_manager.h>
#include <ufomap_msgs/UFOMap.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <future>

namespace ufomap_manager {

    UFOMapManager::UFOMapManager(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
            : nh_(nh),nh_private_(nh_private),tf_listener_(tf_buffer_),
              map_(nh_private.param("resolution", 0.1), nh_private.param("depth_levels", 16),
                   !nh_private.param("multithreaded", false)) {
        setParametersFromROS();
        cloud_sub_ = nh.subscribe("point_cloud", 10, &UFOMapManager::insertCloudCallback, this);

        if (0 < pub_rate_) {
            map_pub_ = nh_private.advertise<ufomap_msgs::UFOMapStamped>(
                    "map", 10, nh_private.param("map_latch", false));
            cloud_pub_ = nh_private.advertise<sensor_msgs::PointCloud2>(
                    "map_cloud", 10, nh_private.param("map_cloud_latch", false));
            pub_timer_ = nh_private.createTimer(ros::Rate(pub_rate_), &UFOMapManager::timerCallback, this);
        }
    }


    void UFOMapManager::setParametersFromROS() {
        std::string ns = ros::this_node::getName();
        frame_id_ = "world";
        if (!ros::param::get(ns + "/frame_id", frame_id_)) {
            ROS_WARN("No frame_id specified. Looking for %s. Default is 'world'.",
                     (ns + "/frame_id").c_str());
        }

        max_range_ = 12;
        if (!ros::param::get(ns + "/max_range", max_range_)) {
            ROS_WARN("No max_range specified. Looking for %s. Default is '12'.",
                     (ns + "/max_range").c_str());
        }

        insert_discrete_ = true;
        if (!ros::param::get(ns + "/insert_discrete", insert_discrete_)) {
            ROS_WARN("No insert_discrete specified. Looking for %s. Default is 'true'.",
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

        clear_robot_ = false;
        if (!ros::param::get(ns + "/clear_robot_enabled", clear_robot_)) {
            ROS_WARN("No clear_robot_enabled_ specified. Looking for %s. Default is 'false'.",
                     (ns + "/clear_robot_enabled_").c_str());
        }

        robot_height_ = 0.2;
        if (!ros::param::get(ns + "/robot_height", robot_height_)) {
            ROS_WARN("No robot_height specified. Looking for %s. Default is '0.2m'.",
                     (ns + "/robot_height").c_str());
        }

        robot_radius_ = 0.5;
        if (!ros::param::get(ns + "/robot_radius", robot_radius_)) {
            ROS_WARN("No robot_radius specified. Looking for %s. Default is '0.5m'.",
                     (ns + "/sensor_height_").c_str());
        }

        pub_rate_ = 10;
        if (!ros::param::get(ns + "/pub_rate", pub_rate_)) {
            ROS_WARN("No pub_rate specified. Looking for %s. Default is '10hz'.",
                     (ns + "/pub_rate").c_str());
        }

    }


    void UFOMapManager::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        ufo::math::Pose6 transform;
        try {
            transform = ufomap_ros::rosToUfo(
                    tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp,
                                               ros::Duration(1.0)).transform);
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1, "%s", ex.what());
            return;
        }

        auto start = std::chrono::steady_clock::now();
        auto start_time = ros::Time::now();
        // Update map
        ufo::map::PointCloudColor cloud;
        ufomap_ros::rosToUfo(*msg, cloud);
        cloud.transform(transform, true);

        if (insert_discrete_) {
            map_.insertPointCloudDiscrete(
                    transform.translation(), cloud, max_range_, insert_depth_,
                    simple_ray_casting_, early_stopping_, false);
        } else {
            map_.insertPointCloud(
                    transform.translation(), cloud, max_range_, insert_depth_,
                    simple_ray_casting_, early_stopping_, false);
        }

        double integration_time = (ros::Time::now() - start_time).toSec();
        ROS_INFO("insert cloud spent %f s", integration_time);

        if (0 == num_integrations_ || integration_time < min_integration_time_) {
            min_integration_time_ = integration_time;
        }
        if (integration_time > max_integration_time_) {
            max_integration_time_ = integration_time;
        }
        accumulated_integration_time_ += integration_time;
        ++num_integrations_;

        // Clear robot
        if (clear_robot_) {
            start = std::chrono::steady_clock::now();

            try {
                transform = ufomap_ros::rosToUfo(
                        tf_buffer_.lookupTransform(frame_id_, robot_frame_id_, msg->header.stamp,
                                                   ros::Duration(1.0)).transform);
            } catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1, "%s", ex.what());
                return;
            }

            ufo::map::Point3 r(robot_radius_, robot_radius_, robot_height_ / 2.0);
            ufo::geometry::AABB aabb(transform.translation() - r,
                                     transform.translation() + r);
            map_.setValueVolume(aabb, map_.getClampingThresMin(), clearing_depth_);

            double clear_time =
                    std::chrono::duration<float, std::chrono::seconds::period>(
                            std::chrono::steady_clock::now() - start)
                            .count();
            if (0 == num_clears_ || clear_time < min_clear_time_) {
                min_clear_time_ = clear_time;
            }
            if (clear_time > max_clear_time_) {
                max_clear_time_ = clear_time;
            }
            accumulated_clear_time_ += clear_time;
            ++num_clears_;
        }

    }

    void UFOMapManager::timerCallback(const ros::TimerEvent &event) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = frame_id_;

        ufomap_msgs::UFOMapStamped msg;
        ufomap_msgs::ufoToMsg(map_, msg.map, false);
        msg.header = header;
        map_pub_.publish(msg);

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

    }
}