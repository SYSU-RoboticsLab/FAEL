//
// Created by hjl on 2020/6/15.
//

#ifndef UFOMAP_WS_UFOMAP_MANAGER_H
#define UFOMAP_WS_UFOMAP_MANAGER_H

#include <ros/ros.h>
#include <ufo/map/occupancy_map.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <future>

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

namespace ufomap_manager
{
    class UFOMapManager
    {
    public:

        UFOMapManager(ros::NodeHandle& nh,ros::NodeHandle& nh_private);

        void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

        void timerCallback(const ros::TimerEvent& event);

        void setParametersFromROS();


        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Subscriber cloud_sub_;
        unsigned int cloud_in_queue_size_;

        // Publishers
        ros::Publisher map_pub_;
        ros::Publisher cloud_pub_;
        unsigned int map_queue_size_;
        ros::Timer pub_timer_;
        double pub_rate_;
        ros::Duration update_rate_;
        ros::Time last_update_time_;
        ros::Publisher info_pub_;

        // TF2
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        ros::Duration transform_timeout_;

        // Map
        ufo::map::OccupancyMap map_;
        std::string frame_id_;

        // Integration
        double max_range_;
        bool insert_discrete_;  
        int insert_depth_;
        bool simple_ray_casting_;   
        int early_stopping_;
        bool async_;

        // Clear robot
        bool clear_robot_;
        std::string robot_frame_id_;
        double robot_height_;
        double robot_radius_;
        int clearing_depth_;

        // Publishing
        bool compress_;
        bool update_part_of_map_;
        ufo::map::DepthType publish_depth_;
        std::future<void> update_async_handler_;

        // Integration
        double min_integration_time_;
        double max_integration_time_ = 0.0;
        double accumulated_integration_time_ = 0.0;
        int num_integrations_ = 0;

        // Clear robot
        double min_clear_time_;
        double max_clear_time_ = 0.0;
        double accumulated_clear_time_ = 0.0;
        int num_clears_ = 0;

        // Publish update
        double min_update_time_;
        double max_update_time_ = 0.0;
        double accumulated_update_time_ = 0.0;
        int num_updates_ = 0;

        // Publish whole
        double min_whole_time_;
        double max_whole_time_ = 0.0;
        double accumulated_whole_time_ = 0.0;
        int num_wholes_ = 0;

        // Verbose
        bool verbose_;

    };
}


#endif //UFOMAP_WS_UFOMAP_MANAGER_H
