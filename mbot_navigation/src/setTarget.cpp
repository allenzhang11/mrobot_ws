//
// Created by yebin on 2021/11/17.
//
/*
 * This ROS node sends the robot goals to move to a particular location on
 * a map.
 * 输入q退出程序。
 * 或者在终端以下面的形式取消所有导航目标：
    rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}
 *
 * 为了确定目标点的坐标，可以首先使用rosrun map_server map_server xxx.yaml
 * 然后在rviz中使用Point Publish; 或者参看rostopic echo /clicked_point
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

// 定义导航目标点的结构体
struct Goal {
    double x;
    double y;
    double yaw;
};

const int NUM = 4; // 目标点的个数


// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

    // Connect to ROS
    ros::init(argc, argv, "simple_navigation_goals");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up so that we can begin processing goals.
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    struct Goal goalList[NUM]; // 定义各个目标点. 用rpy来定义，比用四元数定义更加直观。
    goalList[0].x = 3.0;
    goalList[0].y = 0.0;
    goalList[0].yaw = 0.0;

    goalList[1].x = 5.0;
    goalList[1].y = -2.0;
    goalList[1].yaw = -45.0/180.0*3.14;

    goalList[2].x = 5.0;
    goalList[2].y = -4.0;
    goalList[2].yaw = -90.0/180.0*3.14;

    goalList[3].x = 5.0;
    goalList[3].y = -5.0;
    goalList[3].yaw = -45.0/180.0*3.14;

    char choice_to_continue = 'c';

    do {
        // Create a new goal to send to move_base
        move_base_msgs::MoveBaseGoal goal;

        tf2::Quaternion q;

        // Send a goal to the robot. 去程
        for (int index = 0; index < NUM; index++) {
            q.setRPY(0.0, 0.0, goalList[index].yaw);
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = goalList[index].x;
            goal.target_pose.pose.position.y = goalList[index].y;
            goal.target_pose.pose.orientation.w = q[3];

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            // Wait until the robot reaches the goal
            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("The robot has arrived at the %d goal location", index);
            else
                ROS_INFO("The robot failed to reach the %d goal location for some reason",index);
        }

        // 回程
        for (int index = NUM-1; index >= 0; index--) {
            q.setRPY(0.0, 0.0, goalList[index].yaw);  // 将rpy弧度转化为四元数
            q.normalize();
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = goalList[index].x;
            goal.target_pose.pose.position.y = goalList[index].y;
            goal.target_pose.pose.orientation.w = q[3];

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            // Wait until the robot reaches the goal
            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("The robot has arrived at the %d goal location", index);
            else
                ROS_INFO("The robot failed to reach the %d goal location for some reason",index);
        }

        cin >> choice_to_continue;

    } while(choice_to_continue != 'q') ;

    return 0;
}
