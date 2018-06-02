from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])

    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]])

    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1]])

    return R_z


def trans_matrix(alpha, a, d, q):
    T = Matrix([[cos(q), -sin(q), 0, a],
                [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                [0, 0, 0, 1]])
    return T

def calculate_123(R_EE, px, py, pz, roll, pitch, yaw):
    # Compensate for rotation discrepancy between DH parameters and Gazebo
    Rot_err = rot_z(rad(180)) * rot_y(rad(-90))

    # print Rot_err
    # Matrix([[0,  0, 1],
    #         [0, -1, 0],
    #         [1,  0, 0]])

    R_EE = R_EE * Rot_err
    # print R_EE
    # Matrix([[r13, -r12, r11],
    #         [r23, -r22, r21],
    #         [r33, -r32, r31])

    R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
    # Find original wrist position with formula described in
    # https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/a1abb738-84ee-48b1-82d7-ace881b5aec0
    G = Matrix([[px], [py], [pz]])
    WC = G - (0.303) * R_EE[:, 2]

    # Uncomment to see how end effector will be when wrist center
    # is correct. Turns out when WC is correct theta and EE
    # position errors get larger.
    # WC = test_case[1]

    # Calculate joint angles using Geometric IK method

    # Relevant lesson:
    # https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/8d553d46-d5f3-4f71-9783-427d4dbffa3a
    theta1 = atan2(WC[1], WC[0])

    a = 1.501 # Found by using "measure" tool in RViz.
    b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + \
        pow((WC[2] - 0.75), 2))
    c = 1.25 # Length of joint 1 to 2.

    alpha = acos((b*b + c*c - a*a) / (2*b*c))
    beta = acos((a*a + c*c - b*b) / (2*a*c))
    # gamma = acos((a*a + b*b - c*c) / (2*a*b))
    # print("alpha: {} deg / {} rad".format(deg(alpha).evalf(), alpha))
    # print("beta: {} deg / {} rad".format(deg(beta).evalf(), beta))
    # print("gamma: {} deg / {} rad".format(deg(gamma).evalf(), gamma))

    delta = atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)
    theta2 = pi/2 - alpha - delta

    # Look at Z position of -0.054 in link 4 and use it to calculate epsilon
    # epsilon = math.atan2(d,math.sqrt(a**2-d**2))
    # epsilon = math.atan2(0.054,math.sqrt(1.501**2-0.054**2))
    epsilon = 0.036
    theta3 = pi/2 - (beta + epsilon)
    return (R_EE, WC, theta1, theta2, theta3)

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


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ##
    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
         req.poses[x].orientation.z, req.poses[x].orientation.w])

    ## Insert IK code here!

    r, p, y = symbols('r p y')
    # Define a rotation matrix of the end effector (ee) given roll, pitch, yaw
    R_x = rot_x(r)
    R_y = rot_y(p)
    R_z = rot_z(y)

    R_EE = R_z * R_y * R_x

    R_EE, WC, theta1, theta2, theta3 = calculate_123(R_EE, px, py, pz, roll, pitch, yaw)

    R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    R3_6 = R0_3.inv("LU") * R_EE

    theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])  # +0.45370228
    sq5 = -R3_6[1, 1] / sin(theta6)
    cq5 = R3_6[1, 2]
    theta5 = atan2(sq5, cq5)
    sq4 = R3_6[2, 2] / sin(theta5)
    cq4 = -R3_6[0, 2] / sin(theta5)
    theta4 = atan2(sq4, cq4)

    ## From walkthrough

    theta6_1 = atan2(-R3_6[1, 1], R3_6[1, 0])  # +0.45370228
    theta4_1 = atan2(R3_6[2, 2], -R3_6[0, 2])
    theta5_1 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])

    EE = T0_EE.evalf(subs={"q1": theta1, "q2": theta2, "q3": theta3,
                           "q4": theta4, "q5": theta5, "q6": theta6})

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0], WC[1], WC[2]] # <--- Load your calculated WC values in this array
    your_ee = EE[:3, 3] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
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
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
