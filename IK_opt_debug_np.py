import numpy as np
from time import time
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''
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
            'alpha0': 0.0, 'a0': 0.00, 'd1': 0.75,
            'alpha1': -np.pi / 2.0, 'a1': 0.35, 'd2': 0.00,
            'alpha2': 0.0, 'a2': 1.25, 'd3': 0.00,
            'alpha3': -np.pi / 2.0, 'a3': -0.054, 'd4': 1.50,
            'alpha4': -np.pi / 2.0, 'a4': 0.00, 'd5': 0.00,
            'alpha5': -np.pi / 2.0, 'a5': 0.00, 'd6': 0.00,
            'alpha6': 0.0, 'a6': 0.00, 'd7': 0.303
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

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


class Position:
    def __init__(self, EE_pos):
        self.x = EE_pos[0]
        self.y = EE_pos[1]
        self.z = EE_pos[2]


class Orientation:
    def __init__(self, EE_ori):
        self.x = EE_ori[0]
        self.y = EE_ori[1]
        self.z = EE_ori[2]
        self.w = EE_ori[3]


class Combine:
    def __init__(self,position,orientation):
        self.position = position
        self.orientation = orientation

class Pose:
    def __init__(self,comb):
        self.poses = [comb]


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    comb = Combine(position, orientation)
    req = Pose(comb)

    ik = IK()
    start_time = time()
    
    ########################################################################################
    # Extract end-effector position and orientation
    # position px, py, pz
    px, py, pz = req.poses[x].position.x, req.poses[x].position.y, req.poses[x].position.z
    # orientation roll, pitch, yaw
    x, y, z, w = req.poses[x].orientation.x, req.poses[x].orientation.y, \
                 req.poses[x].orientation.z, req.poses[x].orientation.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([x, y, z, w])
    ########################################################################################
    ## Insert IK code here!
    thetas, WC = ik.eval(px, py, pz, roll, pitch, yaw)
    theta1, theta2, theta3, theta4, theta5, theta6 = thetas
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    fk = ik.eval_fwd_kinematics(theta4, theta5, theta6)

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0], WC[1], WC[2]] # <--- Load your calculated WC values in this array
    your_ee = [fk[0, 3], fk[1, 3], fk[2, 3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = np.sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = np.sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
