#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def tf_matrix(alpha, a, d, q):
    # Creates a transformation matrix based yaw, pitch, roll rotations
    # for around x, y, z rotations
    tf = Matrix([
        [cos(q)             ,             -sin(q),              0,              a],
        [sin(q)*cos(alpha)  ,   cos(q)*cos(alpha),    -sin(alpha),  -sin(alpha)*d],
        [sin(q)*sin(alpha)  ,   cos(q)*sin(alpha),      cos(alpha),  cos(alpha)*d],
        [0                  ,                   0,               0,             1]
    ])
    return tf

def eval_WC(px, py, pz, rot_ee):
    EE = Matrix([
        [px],
        [py],
        [pz]
    ])
    # Calculate wrist center (wc)
    WC = EE - 0.303 * rot_ee[:, 2]
    return WC

def rot_matrix():
    r, p, y = symbols('r p y')
    rot_x = Matrix([
        [1, 0, 0],
        [0, cos(r), -sin(r)],
        [0, sin(r), cos(r)]
    ])
    rot_y = Matrix([
        [cos(p), 0, sin(0)],
        [0, 1, 0],
        [-sin(p), 0, cos(p)]
    ])

    rot_z = Matrix([
        [cos(y), -sin(y), 0],
        [sin(y),  cos(y), 0],
        [0, 0, 1]
    ])

    rot_ee = rot_z * rot_y * rot_x

    rot_z_adj = rot_z.subs(y, radians(180))
    rot_y_adj = rot_y.subs(p, radians(-90))

    rot_error = rot_z_adj * rot_y_adj

    rot_ee = rot_ee * rot_error

    return rot_ee

def IK_thetas(px, py, pz, roll, pitch, yaw):
    # Create DH parameter symbols
    # d parameter is a link offset along z axis (between x0 and x1 axis)
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    # a parameter is a link length along x axis (distance between z0 and z1)
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    # alpha parameter is a twist angle (rotation angle of z axis along x axis) according to
    # right hand rule (counter-clock wise)
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    # q (theta) parameter is joint angle (rotation angle of x axis along z axis) according to
    # the right hand rule (counter-clock wise)
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    # Create a dictionary containing df table
    dh_table = {
        alpha0: 0.0, a0: 0.0, d1: 0.75, q1: q1,
        alpha1: -pi / 2.0, a1: 0.35, d2: 0.0, q2: -pi / 2.0 + q2,
        alpha2: 0.0, a2: 1.25, d3: 0.0, q3: q3,
        alpha3: -pi / 2.0, a3: -0.054, d4: 1.5, q4: q4,
        alpha4: -pi / 2.0, a4: 0.0, d5: 0.0, q5: q5,
        alpha5: -pi / 2.0, a5: 0.0, d6: 0.0, q6: q6,
        alpha6: 0.0, a6: 0.0, d7: 0.303, q7: q7
    }

    # Create transformation matrices
    t01 = tf_matrix(alpha0, a0, d1, q1).subs(dh_table)
    t12 = tf_matrix(alpha1, a1, d2, q2).subs(dh_table)
    t23 = tf_matrix(alpha2, a2, d3, q3).subs(dh_table)
    t34 = tf_matrix(alpha3, a3, d4, q4).subs(dh_table)
    t45 = tf_matrix(alpha4, a4, d5, q5).subs(dh_table)
    t56 = tf_matrix(alpha5, a5, d6, q6).subs(dh_table)
    t6_ee = tf_matrix(alpha6, a6, d7, q7).subs(dh_table)

    t0_ee = t01 * t12 * t23 * t34 * t45 * t56 * t6_ee

    # create end-effector matrix
    rot_ee = rot_matrix()
    rot_ee = rot_ee.subs({'r': roll, 'p': pitch, 'y': yaw})
    WC = eval_WC(px, py, pz, rot_ee)

    # Calculate joint angles using inverse kinematics
    theta1 = atan2(WC[1], WC[0])
    # SSS traingle for theta2 and theta3
    s_a = 1.501
    s_b = sqrt(pow(sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35, 2)
               + pow(WC[2] - 0.75, 2))
    s_c = 1.25

    # aa = acos( (s_b * s_b + s_c * s_c - s_a * s_a) / (2 * s_b * s_c) )
    ab = acos((s_a * s_a + s_c * s_c - s_b * s_b) / (2 * s_a * s_c))
    aa = acos((s_a * s_a + s_b * s_b - s_c * s_c) / (2 * s_a * s_b))

    theta2 = pi / 2 - aa - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
    theta3 = pi / 2 - (ab + 0.036)

    r0_3 = t01[0:3, 0:3] * t12[0:3, 0:3] * t23[0:3, 0:3]
    r0_3 = r0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    r3_6 = r0_3.inv("LU") * rot_ee
    theta4 = atan2(r3_6[2, 2], -r3_6[0, 2])
    theta5 = atan2(sqrt(r3_6[0, 2] * r3_6[0, 2] + r3_6[2, 2] * r3_6[2, 2]), r3_6[1, 2])
    theta6 = atan2(-r3_6[1, 1], r3_6[1, 0])

    return (theta1, theta2, theta3, theta4, theta5, theta6), t0_ee, WC

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
            thetas, t0_ee, WC = IK_thetas(px, py, pz, roll, pitch, yaw)
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
