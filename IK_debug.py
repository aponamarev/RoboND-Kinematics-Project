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
        [cos(p), 0, sin(p)],
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
        alpha0:         0.0, a0:  0.00, d1: 0.33, q1: q1,
        alpha1:   -pi / 2.0, a1:  0.35, d2: 0.42, q2: -pi / 2.0 + q2,
        alpha2:         0.0, a2:  1.25, d3: 0.00, q3: q3,
        alpha3:   -pi / 2.0, a3:0.0536, d4: 1.5014, q4: q4,
        alpha4:   -pi / 2.0, a4:  0.00, d5: 0.00, q5: q5,
        alpha5:   -pi / 2.0, a5:  0.00, d6: 0.00, q6: q6,
        alpha6:         0.0, a6:  0.00, d7:0.303, q7: 0.0
    }

    # Create transformation matrices
    t01     = tf_matrix(alpha0, a0, d1, q1).subs(dh_table)
    t12     = tf_matrix(alpha1, a1, d2, q2).subs(dh_table)
    t23     = tf_matrix(alpha2, a2, d3, q3).subs(dh_table)
    t34     = tf_matrix(alpha3, a3, d4, q4).subs(dh_table)
    t45     = tf_matrix(alpha4, a4, d5, q5).subs(dh_table)
    t56     = tf_matrix(alpha5, a5, d6, q6).subs(dh_table)
    t6_ee   = tf_matrix(alpha6, a6, d7, q7).subs(dh_table)

    t0_ee   = t01 * t12 * t23 * t34 * t45 * t56 * t6_ee

    # create end-effector matrix
    rot_ee = rot_matrix()
    rot_ee = rot_ee.subs({'r': roll, 'p': pitch, 'y': yaw})
    WC = eval_WC(px, py, pz, rot_ee)

    # Calculate joint angles using inverse kinematics
    # **Theta 1**
    # KR210 is fixed at the position of the base (xyz=0,0,0). The axis of the first joint reference frame are aligned with the
    # base axis, where x points in the direction of the arm movement, y is perpendicular ot x, and x points upwards. Therefore,
    # joint 1 is responsible for setting rotation of the main body of the arm along z axis, which allows to evaluate theta1 angle,
    # based on the x and y position of the wrist center (WC): theta1 = atan(wc.y, wc.x).
    theta1 = atan2(WC[1], WC[0])
    # SSS traingle for theta2 and theta3
    s_a = 1.501
    s_b = sqrt(pow(sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35, 2)
               + pow(WC[2] - 0.75, 2))
    s_c = 1.25

    aa = acos( (s_b * s_b + s_c * s_c - s_a * s_a) / (2 * s_b * s_c) )
    ab = acos((s_a * s_a + s_c * s_c - s_b * s_b) / (2 * s_a * s_c))
    #ac = acos((s_a * s_a + s_b * s_b - s_c * s_c) / (2 * s_a * s_b))

    theta2 = pi / 2 - aa - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
    theta3 = pi / 2 - (ab + 0.036)

    r0_3 = t01[0:3, 0:3] * t12[0:3, 0:3] * t23[0:3, 0:3]
    r0_3 = r0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    r3_6 = r0_3.inv("LU") * rot_ee
    theta4 = atan2(r3_6[2, 2], -r3_6[0, 2])
    theta5 = atan2(sqrt(r3_6[0, 2] * r3_6[0, 2] + r3_6[2, 2] * r3_6[2, 2]), r3_6[1, 2])
    theta6 = atan2(-r3_6[1, 1], r3_6[1, 0])

    return (theta1, theta2, theta3, theta4, theta5, theta6), t0_ee, WC

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

    comb = Combine(position,orientation)

    req = Pose(comb)
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
    thetas, t0_ee, WC = IK_thetas(px, py, pz, roll, pitch, yaw)
    theta1, theta2, theta3, theta4, theta5, theta6 = thetas
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    fk = t0_ee.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3, 'q4': theta4, 'q5': theta5, 'q6': theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0], WC[1], WC[2]] # <--- Load your calculated WC values in this array
    your_ee = [fk[0,3], fk[1,3], fk[2,3]] # <--- Load your calculated end effector value from your forward kinematics
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
