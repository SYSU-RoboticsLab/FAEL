//
// Created by hjl on 2021/9/1.
//

#include <ufomap_manager/frontier_manager.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <ufo/map/code.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "read_ufomap_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ufo::map::OccupancyMap map_(0.3);

    std::string ns = ros::this_node::getName();
    std::string txt_name = "ufomap.txt";
    if (!ros::param::get(ns + "/txt_name", txt_name)) {
        ROS_WARN(
                "No txt_name specified. Looking for %s. Default is 'ufomap.txt'.",
                (ns + "/txt_name").c_str());
    }

    if(map_.read(txt_name)){

        ROS_INFO("resolution is %f", map_.getResolution());
        ROS_INFO("size is % f", map_.getNodeSize(0));
        ROS_INFO("leaf node num is %zu", map_.getNumLeafNodes());

        ros::Publisher pub = nh_private.advertise<ufomap_msgs::UFOMapStamped>("ufomap", 1);
        ros::Publisher cloud_pub_ = nh_private.advertise<sensor_msgs::PointCloud2>("ufomap_cloud", 1);

        while (ros::ok()){
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = "world";

            ufomap_msgs::UFOMapStamped msg;
            ufomap_msgs::ufoToMsg(map_, msg.map, false);
            msg.header = header;
            pub.publish(msg);//publish map

            if (0 < cloud_pub_.getNumSubscribers() || cloud_pub_.isLatched()) {
                ufo::map::PointCloud cloud;
                for (auto it = map_.beginLeaves(true, false, false, false, 0),
                             it_end = map_.endLeaves();
                     it != it_end; ++it) {
                    cloud.push_back(it.getCenter());
                }
                ROS_INFO("the cloud size is %zu", cloud.size());

                sensor_msgs::PointCloud2 cloud_msg;
                ufomap_ros::ufoToRos(cloud, cloud_msg);
                cloud_msg.header = header;
                cloud_pub_.publish(cloud_msg);
            }


            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
    }else{
        ROS_WARN("read ufomap failed..");
    }


    return 0;
}
