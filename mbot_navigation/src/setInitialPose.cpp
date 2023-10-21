//
// Created by yebin on 2021/11/18.
//
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

double x = 1.0;  // 设置初始的x,y,theta
double y = -1.0;
double theta = 0.3;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_initial_pose_node");
    ros::NodeHandle nh;
    ros::Publisher pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);

    std::string fixed_frame = "map";
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = fixed_frame;
    pose.header.stamp = ros::Time::now();

    // set x,y coord
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = 0.0;

    // set theta
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);
    quat.normalize();
    pose.pose.pose.orientation.x = quat[0];
    pose.pose.pose.orientation.y = quat[1];
    pose.pose.pose.orientation.z = quat[2];
    pose.pose.pose.orientation.w = quat[3];
    //tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
    pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

    // publish
    ROS_INFO("x: %f, y: %f, z: 0.0, theta: %f",x,y,theta);

    ros::Rate loop_rate(10);

    while (ros::ok()){
        pub_.publish(pose);
        loop_rate.sleep();
    }
    return 0;
}
