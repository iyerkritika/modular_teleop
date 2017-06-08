#!/usr/bin/env python
from sympy import *
import numpy as np
import time
import mpmath as mp
import rospy

def pseudo_inv(full_jaco,var,var_num):
    # pseudoinv = full_jaco.transpose() *(full_jaco*full_jaco.transpose())
    # print "transpose Full 6DOF Jacobian\n",full_jaco.transpose()
    # print "Time taken T:", -(start - time.time())
    print "jacobian\n"
    # start= time.time()
    for i in range(len(var)):
        full_jaco=full_jaco.subs(var[i],var_num[i])
    # print full_jaco.singular_values()
    print simplify(full_jaco)
    # U,S,V = np.linalg.svd(np.array(full_jaco))
    # print np.array(full_jaco)
    U,s,v = np.linalg.svd(np.array(full_jaco))
    # print U
    S=np.diag(s)
    print np.matrix.round(np.dot(U,np.dot(S,v)))
    # print v
    # print type(S)
    # pseudo_jaco = V.transpose()*S.inv()*U
    # return pseudo_jaco
    # # start= time.time()
    # # print "Inv Full 6DOF Jacobian\n",full_jaco.inv()
    # print "Time taken inv:", -(start - time.time())

if __name__ == '__main__':
    # rospy.init_node('velocity_control_node', anonymous=True)
    n = 6
    # rospy.set_param('test',q[0])
    theta = Symbol('theta')
    alpha = Symbol('alpha')
    a = Symbol('a')
    d = Symbol('d')
    q1 = Symbol('q1')
    q2 = Symbol('q2')
    q3 = Symbol('q3')
    q4 = Symbol('q4')
    q5 = Symbol('q5')
    q6 = Symbol('q6')
    q = symbols('q1:7')   #1:q1:(n+1) -> q1,q2,q3...,qn

    #### Depends on manipulator
    theta_val = [q[0],q[1]+(pi/2),q[2],q[3],q[4],q[5]]
    d_val = [290,0,0,302,0,72]

    full_jaco = Matrix([[270*sin(q1)*sin(q2) + 72*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) + 70*sin(q1)*sin(q2 + q3) - 72*sin(q1)*cos(q5)*cos(q2 + q3) - 302*sin(q1)*cos(q2 + q3) + 72*sin(q4)*sin(q5)*cos(q1), -2*(36*sin(q5)*cos(q4)*cos(q2 + q3) + 36*sin(q2 + q3)*cos(q5) + 151*sin(q2 + q3) + 135*cos(q2) + 35*cos(q2 + q3))*cos(q1), -2*(36*sin(q5)*cos(q4)*cos(q2 + q3) + 36*sin(q2 + q3)*cos(q5) + 151*sin(q2 + q3) + 35*cos(q2 + q3))*cos(q1), 72*(sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q5),  72*sin(q1)*sin(q4)*cos(q5) - 72*sin(q5)*cos(q1)*cos(q2 + q3) - 72*sin(q2 + q3)*cos(q1)*cos(q4)*cos(q5),                                                                                        0],[72*sin(q1)*sin(q4)*sin(q5) - 270*sin(q2)*cos(q1) - 72*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 70*sin(q2 + q3)*cos(q1) + 72*cos(q1)*cos(q5)*cos(q2 + q3) + 302*cos(q1)*cos(q2 + q3), -2*(36*sin(q5)*cos(q4)*cos(q2 + q3) + 36*sin(q2 + q3)*cos(q5) + 151*sin(q2 + q3) + 135*cos(q2) + 35*cos(q2 +q3))*sin(q1),-2*(36*sin(q5)*cos(q4)*cos(q2 + q3) + 36*sin(q2 + q3)*cos(q5) + 151*sin(q2 + q3) + 35*cos(q2 + q3))*sin(q1), 72*(sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4))*sin(q5), -72*sin(q1)*sin(q5)*cos(q2 + q3) - 72*sin(q1)*sin(q2 + q3)*cos(q4)*cos(q5) - 72*sin(q4)*cos(q1)*cos(q5),                                                                                        0],[                                                                                                                                                                                0,             -270*sin(q2) - 72*sin(q5)*sin(q2 + q3)*cos(q4) - 70*sin(q2 + q3) + 72*cos(q5)*cos(q2 + q3) + 302*cos(q2 + q3),             -72*sin(q5)*sin(q2 + q3)*cos(q4) - 70*sin(q2 + q3) + 72*cos(q5)*cos(q2 + q3) + 302*cos(q2 + q3),                            -72*sin(q4)*sin(q5)*cos(q2 +q3),                                              -72*sin(q5)*sin(q2 + q3) + 72*cos(q4)*cos(q5)*cos(q2 + q3),                                                                                        0],[                                                                                                                                                                          sin(q1),                                                                                                                  sin(q1),                                                                                        cos(q1)*cos(q2 + q3),              sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1),                 (sin(q1)*sin(q4) - sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3),  (sin(q1)*sin(q4) - sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3)],[                                                                                                                                                                         -cos(q1),                                                                                                                  -cos(q1),                                                                                        sin(q1)*cos(q2 + q3),              sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4),               -(sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), -(sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3)],[                                                                                                                                                                                0,                                                                                                                        0,                                                                                                sin(q2 + q3),                                       -sin(q4)*cos(q2 + q3),                                                     sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5),                                      sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5)]])

    pseudo_inv(full_jaco,q,[0,1,0,0,0,0])

    # rospy.spin()
