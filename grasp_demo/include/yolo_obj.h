#ifndef YOLO_OBJ_H
#define YOLO_OBJ_H

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
	ros::NodeHandle nh_;
	ros::Subscriber object_sub;
	geometry_msgs::PoseStamped Obj_pose;
	geometry_msgs::Pose target_pose;
	float pixel_x, pixel_y;
	ros::ServiceServer get_object;
	string Object_class;

public:
	bool move_finish;
	grasp();
	~grasp();
	void yoloCallback(const darknet_ros_msgs::BoundingBoxes &yolo_tmp);
	void initMove();
	bool getObjectCallback(grasp_demo::get_obj::Request &req,
						   grasp_demo::get_obj::Response &res);
};

#endif
