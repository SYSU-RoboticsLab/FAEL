//
// Created by hjl on 2021/9/4.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

class sensor_odom_to_world {
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Time ros_time;
    tf::TransformListener tf_listener_1;
    tf::TransformListener tf_listener_2;
    ros::Publisher sensor_odom_pub;
    ros::Publisher base_odom_pub;
    ros::Subscriber odom_sub;

    ros::Rate rate;

    std::string sensor_init_frame;
    std::string world_frame;  

    std::string target_frame;
    std::string sensor_frame;
    std::string source_frame;  

    bool is_get_transform;
    tf::Transform T_B_S;
    tf::Transform T_W_S0;

    sensor_odom_to_world(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void odomCallback(const nav_msgs::OdometryConstPtr &input);
};

sensor_odom_to_world::sensor_odom_to_world(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
        nh_(nh), nh_private_(nh_private), rate(100), is_get_transform(false) {

    std::string ns = ros::this_node::getName();
    world_frame = "world";
    if (!ros::param::get(ns + "/world_frame", world_frame)) {
        ROS_WARN("No world_frame specified. Looking for %s. Default is 'world'.",
                 (ns + "/world_frame").c_str());
    }

    target_frame = "base_link";
    if (!ros::param::get(ns + "/target_frame", target_frame)) {
        ROS_WARN("No target_frame specified. Looking for %s. Default is 'base_link'.",
                 (ns + "/target_frame").c_str());
    }

    odom_sub = nh_.subscribe("sensor/sensor_init/odometry", 1, &sensor_odom_to_world::odomCallback, this);
    sensor_odom_pub = nh_.advertise<nav_msgs::Odometry>("sensor/world/odometry", 1);
    base_odom_pub = nh_.advertise<nav_msgs::Odometry>("base_link/world/odometry", 1);
}


void sensor_odom_to_world::odomCallback(const nav_msgs::OdometryConstPtr &input) {

    sensor_frame = input->child_frame_id;
    sensor_init_frame = input->header.frame_id;
    if (!is_get_transform) {  
        bool get_1 = false;
        try {
            tf::StampedTransform transform;
            tf_listener_1.lookupTransform(world_frame, sensor_init_frame, ros::Time(0), transform);
            T_W_S0.setOrigin(transform.getOrigin());
            T_W_S0.setRotation(transform.getRotation());
            get_1 = true;
            ROS_INFO("tf 1 get");
        }
        catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(1, " get 1 acquire---        %s ", ex.what());
        }

        bool get_2 = false;
        try {
            tf::StampedTransform transform;
            tf_listener_2.lookupTransform(target_frame, sensor_frame, ros::Time(0), transform);
            T_B_S.setOrigin(transform.getOrigin());
            T_B_S.setRotation(transform.getRotation());
            get_2 = true;
            ROS_INFO("tf 2 get");
        }
        catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(1, " get 2 acquire---        %s ", ex.what());
        }

        if (get_1 && get_2) {
            is_get_transform = true;
            ROS_INFO("tf set finish, publishing odom ..");
        } else {
            ROS_INFO("TF NOT GET");
        }

    } else {
        ros_time = input->header.stamp;
        
        tf::Quaternion quaternion(input->pose.pose.orientation.x, input->pose.pose.orientation.y,
                                  input->pose.pose.orientation.z, input->pose.pose.orientation.w);
        tf::Vector3 vector3(input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z);
        tf::Transform T_S0_Si(quaternion, vector3);

        tf::Transform T_W_Si = T_W_S0 * T_S0_Si;
        tf::Transform T_W_Bi = T_W_Si * T_B_S.inverse();

        nav_msgs::Odometry odom_msg;
        odom_msg.child_frame_id = sensor_frame;
        odom_msg.header.frame_id = world_frame;
        odom_msg.header.stamp = ros_time;
        odom_msg.header.seq = input->header.seq;
        odom_msg.pose.pose.orientation.x = T_W_Si.getRotation().getX();
        odom_msg.pose.pose.orientation.y = T_W_Si.getRotation().getY();
        odom_msg.pose.pose.orientation.z = T_W_Si.getRotation().getZ();
        odom_msg.pose.pose.orientation.w = T_W_Si.getRotation().getW();
        odom_msg.pose.pose.position.x = T_W_Si.getOrigin().getX();
        odom_msg.pose.pose.position.y = T_W_Si.getOrigin().getY();
        odom_msg.pose.pose.position.z = T_W_Si.getOrigin().getZ();

        odom_msg.twist = input->twist;
        sensor_odom_pub.publish(odom_msg);

        nav_msgs::Odometry odom_msg_2;
        odom_msg_2.child_frame_id = target_frame;
        odom_msg_2.header.frame_id = world_frame;
        odom_msg_2.header.stamp = ros_time;
        odom_msg_2.header.seq = input->header.seq;
        odom_msg_2.pose.pose.orientation.x = T_W_Bi.getRotation().getX();
        odom_msg_2.pose.pose.orientation.y = T_W_Bi.getRotation().getY();
        odom_msg_2.pose.pose.orientation.z = T_W_Bi.getRotation().getZ();
        odom_msg_2.pose.pose.orientation.w = T_W_Bi.getRotation().getW();
        odom_msg_2.pose.pose.position.x = T_W_Bi.getOrigin().getX();
        odom_msg_2.pose.pose.position.y = T_W_Bi.getOrigin().getY();
        odom_msg_2.pose.pose.position.z = T_W_Bi.getOrigin().getZ();

        odom_msg_2.twist = input->twist;
        base_odom_pub.publish(odom_msg_2);

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_odom_to_world");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    sensor_odom_to_world sensorOdomToWorld(nh, nh_private);

    ros::spin();
    return 0;
}
