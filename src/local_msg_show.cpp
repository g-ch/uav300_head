#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include "map_recolor.h"

#define PI_2 1.57

ros::Publisher marker_pub, pcl_pub, target_pub;
Eigen::Vector3f p0;
Eigen::Quaternionf quad(1.0, 0.0, 0.0, 0.0);
double yaw0 = 0.0;
double motor_yaw = 0.0;
double motor_yaw_rate = 0.0;   
    
void timerCallback(const ros::TimerEvent&){
    static tf::TransformBroadcaster br_ros;
    static tf::Transform transform_ros;

    Eigen::Quaternionf q1(0, 0, 0, 1);
    Eigen::Quaternionf axis = quad * q1 * quad.inverse();
    axis.w() = cos(-PI_2/2.0);
    axis.x() = axis.x() * sin(-PI_2/2.0);
    axis.y() = axis.y() * sin(-PI_2/2.0);
    axis.z() = axis.z() * sin(-PI_2/2.0);
    quad = quad * axis;

    transform_ros.setOrigin( tf::Vector3(p0(0), p0(1), p0(2)));
    transform_ros.setRotation( tf::Quaternion(quad.x(), quad.y(), quad.z(), quad.w()) );
    br_ros.sendTransform(tf::StampedTransform(transform_ros, ros::Time::now(), "world", "uav_link"));

    /** Head **/
    static bool init_time = true;
    static double init_head_yaw = 0.0;

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())); //roll
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY())); //pitch
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(motor_yaw,Eigen::Vector3d::UnitZ())); //yaw

    Eigen::Quaterniond head_quaternion;
    head_quaternion=yawAngle*pitchAngle*rollAngle;

    transform_ros.setOrigin( tf::Vector3(0,0,0));
    transform_ros.setRotation( tf::Quaternion(head_quaternion.x(), head_quaternion.y(), head_quaternion.z(), head_quaternion.w()) );
    br_ros.sendTransform(tf::StampedTransform(transform_ros, ros::Time::now(), "uav_link", "head_link"));

    /** Path Markers **/
    static nav_msgs::Path uav_path;
    uav_path.header.stamp=ros::Time::now();
    uav_path.header.frame_id="world";

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = p0(0);
    this_pose_stamped.pose.position.y = p0(1);
    this_pose_stamped.pose.position.z = p0(2);
    this_pose_stamped.pose.orientation.x = head_quaternion.x();
    this_pose_stamped.pose.orientation.y = head_quaternion.y();
    this_pose_stamped.pose.orientation.z = head_quaternion.z();
    this_pose_stamped.pose.orientation.w = head_quaternion.w();

    this_pose_stamped.header.stamp=ros::Time::now();;
    this_pose_stamped.header.frame_id="world";

    uav_path.poses.push_back(this_pose_stamped);

    if(uav_path.poses.size() > 15)  //number of the markers to show is 15
    {
        uav_path.poses.erase(uav_path.poses.begin());
    }

    marker_pub.publish(uav_path);
}


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    // convert cloud to pcl form
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *cloud_in);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_recolored(new pcl::PointCloud<pcl::PointXYZRGB>());
    mapRecolor(cloud_in, cloud_recolored, 0.3, 3.0);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_recolored, output);
    output.header.frame_id = "world";
    pcl_pub.publish(output);
}


void odomCallback(const nav_msgs::Odometry &msg)
{
    /** UAV Pose**/
    p0(0) = msg.pose.pose.position.y;
    p0(1) = -msg.pose.pose.position.x;
    p0(2) = msg.pose.pose.position.z;

    quad.x() = msg.pose.pose.orientation.x;
    quad.y() = msg.pose.pose.orientation.y;
    quad.z() = msg.pose.pose.orientation.z;
    quad.w() = msg.pose.pose.orientation.w;
}


void headCallback(const geometry_msgs::Point32 &msg)
{
    motor_yaw = msg.x; // - 3.1415926 / 2;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle nh;

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);	
	
    ros::Subscriber cloud_sub = nh.subscribe("/ring_buffer/cloud_ob",1,cloudCallback);
    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom",1,odomCallback);
    ros::Subscriber head_sub = nh.subscribe("/place_velocity_info_corrected",1,headCallback);

    target_pub = nh.advertise<visualization_msgs::Marker>("/current_path", 1);
    marker_pub = nh.advertise<nav_msgs::Path>("/uav_path",1, true);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/recolored_map", 1, true);

    ros::Rate rate(20.0);
    ros::spin();

    return 0;
}
