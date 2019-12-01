#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

tf::Transform transform;

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;


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

        rate.sleep();
    }


	return 0;
}