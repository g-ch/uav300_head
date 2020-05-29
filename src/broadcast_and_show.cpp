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
#include <nav_msgs/Path.h>
#include <vector>

#define PI_2 1.57

std::map<std::string, int> rviz_objects_max_num;
ros::Publisher marker_pub;

void rotateVector(cv::Point &center, cv::Point &start_point, float angle, cv::Point &end_point)
{
    cv::Point start_vector = start_point - center;
    cv::Point new_point_vector;
    new_point_vector.x = cos(angle)*start_vector.x + sin(angle)*start_vector.y;
    new_point_vector.y = -sin(angle)*start_vector.x + cos(angle)*start_vector.y;
    end_point = new_point_vector + center;
}

void dataCallback(const wifi_transmitter::Display &msg){
    Eigen::Quaternionf quad(1.0, 0.0, 0.0, 0.0);
    double yaw0 = 0.0;
    Eigen::Vector3f p0;
    double motor_yaw = 0.0;
    double motor_yaw_rate = 0.0;

    static tf::TransformBroadcaster br_ros;
    static tf::Transform transform_ros;

    if(msg.vel_info.x == 0.0)
        return;

    /** UAV Pose**/
    p0(0) = msg.local_pose.position.y;
    p0(1) = -msg.local_pose.position.x;
    p0(2) = msg.local_pose.position.z;

    quad.x() = msg.local_pose.orientation.x;
    quad.y() = msg.local_pose.orientation.y;
    quad.z() = msg.local_pose.orientation.z;
    quad.w() = msg.local_pose.orientation.w;

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

    motor_yaw = msg.vel_info.x; // - 3.1415926 / 2;

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

    for(auto & object_i : msg.objects.result){

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

    /** Path Markers **/
//    visualization_msgs::Marker path_markers;
//    path_markers = msg.makers;
//    marker_pub.publish(path_markers);

    nav_msgs::Path uav_path;
    uav_path.header.stamp=ros::Time::now();
    uav_path.header.frame_id="world";

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = p0(0);
    this_pose_stamped.pose.position.y = p0(1);
    this_pose_stamped.pose.position.z = p0(2);
    this_pose_stamped.pose.orientation.x = quad.x();
    this_pose_stamped.pose.orientation.y = quad.y();
    this_pose_stamped.pose.orientation.z = quad.z();
    this_pose_stamped.pose.orientation.w = quad.w();

    this_pose_stamped.header.stamp=ros::Time::now();;
    this_pose_stamped.header.frame_id="world";

    uav_path.poses.push_back(this_pose_stamped);

    if(uav_path.poses.size() > 10)  //number of the markers to show is 10
    {
        uav_path.poses.erase(uav_path.poses.begin());
    }

    marker_pub.publish(uav_path);

    /*** Cost panels and detection time***/
    std::map<std::string, std::vector<double>> painting_data_map;
    painting_data_map["costHeadObjects"]=msg.cost_head_objects.data;
    painting_data_map["costHeadVelocity"]=msg.cost_head_velocity.data;
    painting_data_map["costHeadDirection"]=msg.cost_head_direction.data;
    painting_data_map["costHeadFluctuation"]=msg.cost_head_fluctuation.data;
    painting_data_map["costHeadFinal"]=msg.cost_head_final.data;
    painting_data_map["costHeadUpdate"]=msg.cost_head_update.data;

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

    std::string detection_time = std::to_string(msg.detection_time.data) + " s";
    cv::putText(background, "detection time", detection_time_place, CV_FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255));
    cv::putText(background, detection_time, detection_time_num_place, CV_FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255));

    cv::imshow("costHeadObjects", background);
    cv::waitKey(1);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle nh;

    rviz_objects_max_num["robot"] = 3;
    rviz_objects_max_num["drone"] = 3;

    ros::Subscriber data_sub = nh.subscribe("/Display/transfered",1,dataCallback);
//    marker_pub = nh.advertise<visualization_msgs::Marker>("/current_path", 1);
    marker_pub = nh.advertise<nav_msgs::Path>("/uav_path",1, true);

    ros::Rate rate(20.0);
    ros::spin();

	return 0;
}
