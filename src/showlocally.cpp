#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>
#include <wifi_transmitter/Display.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/PointCloud2.h>
#include "map_recolor.h"

std::map<std::string, int> rviz_objects_max_num;
ros::Publisher marker_pub, pcl_pub;

wifi_transmitter::Display display_msg;
bool updated_flag = false;

void rotateVector(cv::Point &center, cv::Point &start_point, float angle, cv::Point &end_point)
{
    cv::Point start_vector = start_point - center;
    cv::Point new_point_vector;
    new_point_vector.x = cos(angle)*start_vector.x + sin(angle)*start_vector.y;
    new_point_vector.y = -sin(angle)*start_vector.x + cos(angle)*start_vector.y;
    end_point = new_point_vector + center;
}


void objs_msg_cb(const wifi_transmitter::ObjectsInTracking::ConstPtr &msg){
    display_msg.objects = *msg;
    // display_msg.objects.header = msg->header;
    // display_msg.objects.result = msg->result;
}

void point32_msg_cb(const geometry_msgs::Point32::ConstPtr& msg){
    display_msg.vel_info = *msg;
}

void marker_msg_cb(const visualization_msgs::Marker::ConstPtr& msg){
    display_msg.makers = *msg;
}

void pose_msg_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    display_msg.local_pose = msg->pose;
}

void sim_pose_msg_cb(const geometry_msgs::Pose::ConstPtr& msg){
    display_msg.local_pose.position.x = -msg->position.y;
    display_msg.local_pose.position.y = msg->position.x;
    display_msg.local_pose.position.z = msg->position.z;

    display_msg.local_pose.orientation.x = -msg->orientation.y;
    display_msg.local_pose.orientation.y = msg->orientation.x;
    display_msg.local_pose.orientation.z = msg->orientation.z;
    display_msg.local_pose.orientation.w = msg->orientation.w;
}

void detectionTimeCallback(const std_msgs::Float64& msg){
    display_msg.detection_time = msg;
}

void costHeadVelocityCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_velocity = msg;
}

void costHeadDirectionCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_direction = msg;
}

void costHeadObjectsCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_objects = msg;
}

void costHeadFluctuationCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_fluctuation = msg;
}

void costHeadFinalCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_final = msg;
}

void costHeadUpdateCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_update = msg;
    updated_flag = true;
}


void show(const ros::TimerEvent& e){

    if(!updated_flag) return;

    Eigen::Quaternionf quad(1.0, 0.0, 0.0, 0.0);
    double yaw0 = 0.0;
    Eigen::Vector3f p0;
    double motor_yaw = 0.0;
    double motor_yaw_rate = 0.0;

    static tf::TransformBroadcaster br_ros;
    static tf::Transform transform_ros;

    if(display_msg.vel_info.x == 0.0)
        return;

    /** UAV Pose**/
    p0(0) = display_msg.local_pose.position.y;
    p0(1) = -display_msg.local_pose.position.x;
    p0(2) = display_msg.local_pose.position.z;

    quad.x() = display_msg.local_pose.orientation.y;
    quad.y() = -display_msg.local_pose.orientation.x;
    quad.z() = display_msg.local_pose.orientation.z;
    quad.w() = display_msg.local_pose.orientation.w;

    transform_ros.setOrigin( tf::Vector3(p0(0), p0(1), p0(2)));
    transform_ros.setRotation( tf::Quaternion(quad.x(), quad.y(), quad.z(), quad.w()) );
    br_ros.sendTransform(tf::StampedTransform(transform_ros, ros::Time::now(), "world", "uav_link"));

    /** Head **/
    static bool init_time = true;
    static double init_head_yaw = 0.0;

    motor_yaw = display_msg.vel_info.x; // - 3.1415926 / 2;

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())); //roll
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY())); //pitch
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(motor_yaw,Eigen::Vector3d::UnitZ())); //yaw

    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;

    transform_ros.setOrigin( tf::Vector3(0,0,0));
    transform_ros.setRotation( tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()) );
    br_ros.sendTransform(tf::StampedTransform(transform_ros, ros::Time::now(), "uav_link", "head_link"));


    /** Objects *
    * Publish tf to show in rviz.  Rviz_objects_max_num is the number of objects in urdf file for rviz **/
    std::map<std::string, int> rviz_object_counter;

    for(auto & object_i : display_msg.objects.result){

        if(rviz_objects_max_num.count(object_i.label) > 0){
            if(rviz_object_counter.count(object_i.label) > 0){
                if(rviz_object_counter[object_i.label] < rviz_objects_max_num[object_i.label]-1){
                    rviz_object_counter[object_i.label] ++;
                }else{
                    continue;
                }
            }else{
                rviz_object_counter[object_i.label] = 0;
            }

            transform_ros.setOrigin( tf::Vector3(object_i.position.x, object_i.position.y, 0.2));
            transform_ros.setRotation( tf::Quaternion(0, 0, 0, 1) );
            br_ros.sendTransform(tf::StampedTransform(transform_ros, ros::Time::now(), "world", object_i.label+std::to_string(rviz_object_counter[object_i.label])+"_link"));
        }
    }

    /** Let objects that is not updated disappear **/
    std::map<std::string, int>::iterator rviz_object_iter;
    for(rviz_object_iter = rviz_objects_max_num.begin(); rviz_object_iter != rviz_objects_max_num.end(); rviz_object_iter++){
        if(rviz_object_counter.count(rviz_object_iter->first) > 0){  //if updated in this callback
            for(int i=rviz_object_counter[rviz_object_iter->first] + 1; i<rviz_object_iter->second; i++){
                transform_ros.setOrigin( tf::Vector3(1000, 1000, 0));  //fly away
                transform_ros.setRotation( tf::Quaternion(0, 0, 0, 1) );
                br_ros.sendTransform(tf::StampedTransform(transform_ros, ros::Time::now(), "world", rviz_object_iter->first+std::to_string(i)+"_link"));
            }
        } else{
            for(int j=0; j<rviz_object_iter->second; j++){
                transform_ros.setOrigin( tf::Vector3(1000, 1000, 0));  //fly away
                transform_ros.setRotation( tf::Quaternion(0, 0, 0, 1) );
                br_ros.sendTransform(tf::StampedTransform(transform_ros, ros::Time::now(), "world", rviz_object_iter->first+std::to_string(j)+"_link"));
            }
        }
    }

    /*** Cost panels and detection time***/
    std::map<std::string, std::vector<double>> painting_data_map;
    painting_data_map["costHeadObjects"]=display_msg.cost_head_objects.data;
    painting_data_map["costHeadVelocity"]=display_msg.cost_head_velocity.data;
    painting_data_map["costHeadDirection"]=display_msg.cost_head_direction.data;
    painting_data_map["costHeadFluctuation"]=display_msg.cost_head_fluctuation.data;
    painting_data_map["costHeadFinal"]=display_msg.cost_head_final.data;
    painting_data_map["costHeadUpdate"]=display_msg.cost_head_update.data;

    int rows = 480;  // y
    int cols = 700;  // x
    int center_step = 200;
    int size_one_pannel = 60;
    cv::Mat background = cv::Mat::zeros(rows, cols, CV_8UC3);
    cv::Point center = cv::Point(center_step/2, center_step/2);

    for(auto & vector_i : painting_data_map){
        int num = vector_i.second.size();
        float angle_one_piece = 2*M_PI/num;

        /** Map color to 0, 255 **/
        double min_value = 1000000.f;
        double max_value = -1000000.f;
        for(auto & value_i : vector_i.second){

            if(value_i < min_value){
                min_value = value_i;
            }
            if(value_i > max_value){
                max_value = value_i;
            }
        }

        double delt_value = (max_value - min_value) / 250;
        cv::Point start_point = cv::Point(center.x, center.y + size_one_pannel);
        cv::Point text_place = cv::Point(start_point.x - 50, start_point.y + 20);
        cv::putText(background, vector_i.first, text_place, CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
        cv::putText(background, "Max:"+std::to_string(max_value), cv::Point(text_place.x, text_place.y+20), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
        cv::putText(background, " Min:"+std::to_string(min_value), cv::Point(text_place.x, text_place.y+40), CV_FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));

        /// Draw triangles
        for(int i=0; i<num; i++)
        {
            float delt_angle_rad = angle_one_piece * i; /// Note z axis is the opposite
            cv::Point middle_point, left_point, right_point;
            rotateVector(center, start_point, delt_angle_rad,middle_point);
            rotateVector(center, middle_point, angle_one_piece/2.f,left_point);
            rotateVector(center, middle_point, -angle_one_piece/2.f,right_point);

            std::vector<cv::Point> contour;
            contour.push_back(center);
            contour.push_back(left_point);
            contour.push_back(right_point);

            std::vector<std::vector<cv::Point >> contours;
            contours.push_back(contour);

            int color = (int)(250 - (vector_i.second[i] - min_value)/delt_value) + 5;
            cv::Scalar color_to_fill = cv::Scalar(0, 0, color);
            if(vector_i.second[i] == min_value) color_to_fill(0) = 150;

            cv::polylines(background, contours, true, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            cv::fillPoly(background, contours, color_to_fill);
        }
        center.x += center_step;
        if(center.x > cols - center_step/2){
            center.x = center_step/2;
            center.y += center_step;
        }
    }

    cv::Point detection_time_place = cv::Point(100, rows - 20);
    cv::Point detection_time_num_place = cv::Point(380, rows - 20);

    std::string detection_time = std::to_string(display_msg.detection_time.data) + " s";
    cv::putText(background, "detection time", detection_time_place, CV_FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255));
    cv::putText(background, detection_time, detection_time_num_place, CV_FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255));

    cv::imshow("costHeadObjects", background);
    cv::waitKey(1);
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


int main(int argc, char** argv){
    ros::init(argc, argv, "showlocally");
    ros::NodeHandle ros_nh;

    rviz_objects_max_num["robot"] = 3;
    rviz_objects_max_num["drone"] = 3;

    //通过自己合成消息，直接发送
    ros::Subscriber objs_sub = ros_nh.subscribe("/mot/objects_in_tracking_predicted",2,objs_msg_cb);
    ros::Subscriber point32_sub = ros_nh.subscribe("/place_velocity_info_corrected",2,point32_msg_cb);
    ros::Subscriber marker_sub = ros_nh.subscribe("/visualization_marker",2,marker_msg_cb);
    ros::Subscriber pose_sub = ros_nh.subscribe("/iris/ground_truth/pose",2,sim_pose_msg_cb); // For simulation
    ros::Subscriber detection_time_sub = ros_nh.subscribe("/yolo_ros_real_pose/detection_time", 2, detectionTimeCallback);

    ros::Subscriber cost_head_velocity_sub = ros_nh.subscribe("/head_cost/cost_head_velocity", 2, costHeadVelocityCallback);
    ros::Subscriber cost_head_direction_sub = ros_nh.subscribe("/head_cost/cost_head_direction", 2, costHeadDirectionCallback);
    ros::Subscriber cost_head_objects_sub = ros_nh.subscribe("/head_cost/cost_head_objects", 2, costHeadObjectsCallback);
    ros::Subscriber cost_head_fluctuation_sub = ros_nh.subscribe("/head_cost/cost_head_fluctuation", 2, costHeadFluctuationCallback);
    ros::Subscriber cost_head_final_sub = ros_nh.subscribe("/head_cost/cost_head_final", 2, costHeadFinalCallback);
    ros::Subscriber cost_head_update_sub = ros_nh.subscribe("/head_cost/cost_head_update", 2, costHeadUpdateCallback);

    ros::Subscriber cloud_sub = ros_nh.subscribe("/ring_buffer/cloud_ob",1,cloudCallback);
    pcl_pub = ros_nh.advertise<sensor_msgs::PointCloud2>("/recolored_map", 1, true);

    ros::Timer timer1 = ros_nh.createTimer(ros::Duration(0.2), show); // RATE 3 Hz to publish


    ros::spin();

	return 0;
}