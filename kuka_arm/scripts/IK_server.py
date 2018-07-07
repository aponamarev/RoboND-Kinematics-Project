#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import numpy as np
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose

def rot_x(r):
    return np.matrix([
        [1, 0, 0],
        [0, np.cos(r), -np.sin(r)],
        [0, np.sin(r), np.cos(r)]
    ])

def rot_y(p):
    return np.matrix([
        [np.cos(p), 0, np.sin(p)],
        [0, 1, 0],
        [-np.sin(p), 0, np.cos(p)]
    ])

def rot_z(y):
    return np.matrix([
        [np.cos(y), -np.sin(y), 0],
        [np.sin(y),  np.cos(y), 0],
        [0, 0, 1]
    ])

class IK(object):
    dh_table = None
    q1, q2, q3, q4, q5, q6, q7 = None, None, None, None, None, None, None
    r0_3 = None
    t0_ee = None
    t01 = None
    t12 = None
    t23 = None

    def __init__(self):
        # Create DH parameter symbols
        # d parameter is a link offset along z axis (between x0 and x1 axis)
        # a parameter is a link length along x axis (distance between z0 and z1)
        # alpha parameter is a twist angle (rotation angle of z axis along x axis) according to
        # right hand rule (counter-clock wise)
        # q (theta) parameter is joint angle (rotation angle of x axis along z axis) according to
        # the right hand rule (counter-clock wise
        # Create a dictionary containing df table
        self.dh_table = {
            'alpha0':            0.0, 'a0': 0.00, 'd1': 0.33,
            'alpha1':   -np.pi / 2.0, 'a1': 0.35, 'd2': 0.42,
            'alpha2':            0.0, 'a2': 1.25, 'd3': 0.0,
            'alpha3':   -np.pi / 2.0, 'a3': 0.0536, 'd4': 1.5014,
            'alpha4':   -np.pi / 2.0, 'a4': 0.0, 'd5': 0.00,
            'alpha5':   -np.pi / 2.0, 'a5': 0.0, 'd6': 0.00,
            'alpha6':            0.0, 'a6': 0.0, 'd7': 0.303
        }

    def _tf_matrix(self, alpha, a, d, q):
        # Creates a transformation matrix based yaw, pitch, roll rotations
        # for around x, y, z rotations
        tf = np.matrix([
            [np.cos(q)             ,             -np.sin(q),              0,              a],
            [np.sin(q)*np.cos(alpha),   np.cos(q)*np.cos(alpha),    -np.sin(alpha),  -np.sin(alpha)*d],
            [np.sin(q)*np.sin(alpha),   np.cos(q)*np.sin(alpha),      np.cos(alpha),  np.cos(alpha)*d],
            [0                  ,                   0,               0,             1]
        ])
        return tf

    def _eval_WC(self, px, py, pz, rot_ee):
        EE = np.matrix([
            [px],
            [py],
            [pz]
        ])
        # Calculate wrist center (wc)
        WC = EE - 0.303 * rot_ee[:, 2]
        return WC

    def _rot_matrix(self, r, p, y):
        _rot_x = rot_x(r)
        _rot_y = rot_y(p)
        _rot_z = rot_z(y)
        rot_ee = np.matmul(np.matmul(_rot_z, _rot_y), _rot_x)

        rot_z_adj = rot_z(np.radians(180))
        rot_y_adj = rot_y(np.radians(-90))

        rot_error = np.matmul(rot_z_adj, rot_y_adj)

        rot_ee = np.matmul(rot_ee, rot_error)

        return rot_ee


    def eval(self, px, py, pz, roll, pitch, yaw):
        # create end-effector matrix
        rot_ee = self._rot_matrix(roll, pitch, yaw)
        WC = self._eval_WC(px, py, pz, rot_ee)

        # Calculate joint angles using inverse kinematics
        # **Theta 1**
        # KR210 is fixed at the position of the base (xyz=0,0,0). The axis of the first joint reference frame are aligned with the
        # base axis, where x points in the direction of the arm movement, y is perpendicular ot x, and x points upwards. Therefore,
        # joint 1 is responsible for setting rotation of the main body of the arm along z axis, which allows to evaluate theta1 angle,
        # based on the x and y position of the wrist center (WC): theta1 = atan(wc.y, wc.x).
        theta1 = np.arctan2(WC[1,0], WC[0,0])
        # SSS traingle for theta2 and theta3
        s_a = 1.501
        s_b = np.sqrt(
            pow(np.sqrt(WC[0,0]**2 + WC[1,0]**2) - 0.35, 2)
            + pow(WC[2,0] - 0.75, 2)
        )
        s_c = 1.25

        aa = np.arccos( (s_b**2 + s_c**2 - s_a**2) / (2 * s_b * s_c) )
        ab = np.arccos((s_a**2 + s_c**2 - s_b**2) / (2 * s_a * s_c))
        #ac = np.arccos((s_a**2 + s_b[0,0]**2 - s_c**2) / (2 * s_a * s_b[0,0]))

        theta2 = np.pi / 2 - aa - np.arctan2(WC[2,0] - 0.75, np.sqrt(WC[0,0]**2 + WC[1,0]**2) - 0.35)
        theta3 = np.pi / 2 - (ab + 0.036)

        # Create transformation matrices
        self.q1, self.q2, self.q3 = theta1, -np.pi / 2.0 + theta2, theta3
        self.t01 = self._tf_matrix(self.dh_table['alpha0'], self.dh_table['a0'], self.dh_table['d1'],
                                   self.q1)
        self.t12 = self._tf_matrix(self.dh_table['alpha1'], self.dh_table['a1'], self.dh_table['d2'],
                                   self.q2)
        self.t23 = self._tf_matrix(self.dh_table['alpha2'], self.dh_table['a2'], self.dh_table['d3'],
                                   self.q3)
        self.r0_3 = self.t01[0:3, 0:3] * self.t12[0:3, 0:3] * self.t23[0:3, 0:3]

        r3_6 = np.linalg.inv(self.r0_3) * rot_ee
        theta4 = np.arctan2(r3_6[2, 2], -r3_6[0, 2])
        theta5 = np.arctan2(np.sqrt(r3_6[0, 2] * r3_6[0, 2] + r3_6[2, 2] * r3_6[2, 2]), r3_6[1, 2])
        theta6 = np.arctan2(-r3_6[1, 1], r3_6[1, 0])

        return (theta1, theta2, theta3, theta4, theta5, theta6), WC

    def eval_fwd_kinematics(self, q4, q5, q6, q7=0.0):
        self.q4, self.q5, self.q6, self.q7 = q4, q5, q6, q7

        t34 = self._tf_matrix(self.dh_table['alpha3'], self.dh_table['a3'], self.dh_table['d4'], self.q4)
        t45 = self._tf_matrix(self.dh_table['alpha4'], self.dh_table['a4'], self.dh_table['d5'], self.q5)
        t56 = self._tf_matrix(self.dh_table['alpha5'], self.dh_table['a5'], self.dh_table['d6'], self.q6)
        t6_ee = self._tf_matrix(self.dh_table['alpha6'], self.dh_table['a6'], self.dh_table['d7'], self.q7)

        t0_ee = self.t01 * self.t12 * self.t23 * t34 * t45 * t56 * t6_ee
        return t0_ee

ik = IK()
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            thetas, _ = ik.eval(px, py, py, roll, pitch, yaw)
            theta1, theta2, theta3, theta4, theta5, theta6 = thetas

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
