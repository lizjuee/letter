#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
from grasp_demo.srv import *


def tf_transform(req):
    tool_h_cam = np.array([[-0.0130, -0.9998, 0.0178, 0.0682],
                           [0.9999, -0.0132, -0.0081, -0.0340],
                           [0.0084, 0.0177, 0.9998, -0.1470],
                           [0.0, 0.0, 0.0, 1.0]
                           ])

    cam_h_obj = np.array([[req.marker_x], [req.marker_y], [req.marker_z], [1]])

    r_x = np.array([[1, 0, 0],
                    [0, math.cos(req.robot_roll), -math.sin(req.robot_roll)],
                    [0, math.sin(req.robot_roll), math.cos(req.robot_roll)]
                    ])

    r_y = np.array([[math.cos(req.robot_pitch), 0, math.sin(req.robot_pitch)],
                    [0, 1, 0],
                    [-math.sin(req.robot_pitch), 0, math.cos(req.robot_pitch)]
                    ])

    r_z = np.array([[math.cos(req.robot_yaw), -math.sin(req.robot_yaw), 0],
                    [math.sin(req.robot_yaw), math.cos(req.robot_yaw), 0],
                    [0, 0, 1]
                    ])

    r = np.dot(r_z, np.dot(r_y, r_x))

    base_h_tool = np.array([[r[0, 0], r[0, 1], r[0, 2], req.robot_x],
                            [r[1, 0], r[1, 1], r[1, 2], req.robot_y],
                            [r[2, 0], r[2, 1], r[2, 2], req.robot_z],
                            [0, 0, 0, 1]
                            ])

    base_h_obj = np.dot(np.dot(base_h_tool, tool_h_cam), cam_h_obj)
    print(base_h_obj)

    return objtobaselinkResponse(base_h_obj[0, 0], base_h_obj[1, 0], base_h_obj[2, 0])


def obj_to_base_server():
    rospy.init_node('objtoBaseServer')

    s = rospy.Service('objtobaselink', objtobaselink, tf_transform)

    rospy.spin()


if __name__ == "__main__":
    obj_to_base_server()
