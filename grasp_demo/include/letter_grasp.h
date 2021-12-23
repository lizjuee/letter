#ifndef LETTER_GRASP_H
#define LETTER_GRASP_H

#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include "grasp_demo/objtobaselink.h"
#include "grasp_demo/cam2real.h"
#include "grasp_demo/get_obj.h"
#include <xarm_gripper/MoveAction.h>
#include "std_msgs/Float32MultiArray.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "moveit_msgs/CollisionObject.h"

using namespace std;

class grasp
{
private:
	moveit::planning_interface::MoveGroupInterface armgroup;
	string end_effector_link, reference_frame, id;
	ros::NodeHandle nh_;
	vector<double> joint_group_positions;
	ros::Subscriber pose_sub, object_sub;
	geometry_msgs::PoseStamped Obj_pose;
	double grasp_angle;
	bool find_enable,  yolo_enable;
	geometry_msgs::Pose target_pose;
	Eigen::Quaterniond quaternion;
	string pause_;
	float pixel_x, pixel_y ;

public:
	bool move_finish;
	grasp();
	~grasp();
	void gripper_open();  //夹爪开
	void gripper_close(); //夹爪关
	void goSP(); //去到初始点
        void getPose(string object); //这个函数得到物体的像素坐标 
	void getObjPose(); // 这个函数得到物体和相机的相对位置，并通过坐标i转化，得到物体在机械臂基坐标系下的位置
	void goObj(); // 去到物体所在的位置
	void graspObj(); // 抓取物体
	void moveObj(int i); // 转移物体到相应的位置
	void goHome(); // 机械臂复位
	void initMove(); // 运动的实现（总）
};

#endif
