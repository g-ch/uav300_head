#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>
#include <math.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle node;

    ros::Publisher view_field_pub = node.advertise<sensor_msgs::Range>("/view_field",1);

    tf::TransformBroadcaster br;
    tf::Transform transform;
	sensor_msgs::Range range;

    ros::Rate rate(20.0);
    while (node.ok()){
        ros::spinOnce();
    	transform.setOrigin( tf::Vector3(-0.05 + 1.6*sin(ros::Time::now().toSec()), 1.6*cos(ros::Time::now().toSec()), 1.5));
    	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "uav_link"));

        transform.setOrigin( tf::Vector3(-0.05 + 1.0*sin(ros::Time::now().toSec()), 1.2*cos(ros::Time::now().toSec()), 1.0));
    	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ardrone1_link"));

        transform.setOrigin( tf::Vector3(1.0, 1.2*cos(ros::Time::now().toSec()), 0.0));
    	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot1_link"));

        transform.setOrigin( tf::Vector3(0,0,0));
        transform.setRotation( tf::Quaternion(0, 0, sin(ros::Time::now().toSec()), 1) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "uav_link", "head_link"));

        // range.header.frame_id = "head_link";
        // range.radiation_type = 1;
        // range.field_of_view = 3.14 / 3.0;
        // range.min_range = 0.3;
        // range.max_range = 10.0;
        // range.range = 3.5;
        // view_field_pub.publish(range);

        rate.sleep();
    }


	return 0;
}