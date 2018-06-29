from sympy import init_session, symbols, sin, cos, Matrix, pi
init_session()

# 2. Denavit-Hartenberg (DH) Parameters Derivation
### Create mathematical symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

### Create Generalized form of DH homogenious transform matrix
def tf_matrix(alpha, a, d, q):
  # Creates a transformation matrix based yaw, pitch, roll rotations
  # for around x, y, z rotations
  tf = Matrix([
    [cos(q), -sin(q), 0, a],
    [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
    [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
    [0, 0, 0, 1]
  ])
  return tf

### Create table of DH parameters
dh_table = {
  alpha0:     0,  a0:        0, d1:  0.75, q1: q1,
  alpha1: -pi/2,  a1:     0.35, d2:     0, q2: q2-pi/2,
  alpha2:     0,  a2:     1.25, d3:     0, q3: q3,
  alpha3: -pi/2,  a3:   -0.054, d4:   1.5, q4: q4,
  alpha4:  pi/2,  a4:        0, d5:     0, q5: q5,
  alpha5: -pi/2,  a5:        0, d6:     0, q6: q6,
  alpha6:     0,  a6:        0, d7: 0.303, q7:   0
}

### Create transformation matrices for each joint
t01     = tf_matrix(alpha0, a0, d1, q1).subs(dh_table)
t12     = tf_matrix(alpha1, a1, d2, q2).subs(dh_table)
t23     = tf_matrix(alpha2, a2, d3, q3).subs(dh_table)
t34     = tf_matrix(alpha3, a3, d4, q4).subs(dh_table)
t45     = tf_matrix(alpha4, a4, d5, q5).subs(dh_table)
t56     = tf_matrix(alpha5, a5, d6, q6).subs(dh_table)
t6_ee   = tf_matrix(alpha6, a6, d7, q7).subs(dh_table)

t0_ee   = t01 * t12 * t23 * t34 * t45 * t56 * t6_ee
t0_ee


