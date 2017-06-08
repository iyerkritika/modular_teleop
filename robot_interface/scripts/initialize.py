#!/usr/bin/env python
from sympy import *
import numpy as np
import time
import mpmath as mp
import rospy

def dh2mat(theta,d,alpha,a):
    T = Matrix([[cos(theta), -sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)],
                [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta) ],
                [0         ,        sin(alpha)   ,       cos(alpha)     ,      d      ],
                [0         ,           0         ,          0           ,      1      ]])
    return T

def full_jacobian(trans,var,rot):
    start= time.time()
    full_jaco = Matrix(6,len(var),zeros(6,len(var)))
    position_vec = trans[0:3,3]
    full_jaco[0:3,:] = simplify(position_vec.jacobian(var))
    for i in range(len(var)):
        full_jaco[3:6,i] = rot[0:3,i]
    full_jaco =  simplify(full_jaco)
    print "Time taken jaco:", -(start - time.time())
    # return np.array(full_jaco)
    return full_jaco

def simplifyH(H,q):
    q_val = Matrix([0,0,0,0,0,0])
    q_val = (pi/180)*q_val
    for i in range(6):
        H = H.subs(q[i],q_val[i])
    H =  simplify(H)
    H = trigsimp(H)
    H = cancel(H)
    H =  simplify(H)
    print "Simplified H\n\n",H

if __name__ == '__main__':
    rospy.init_node('init_node', anonymous=True)
    n = 6
    q = symbols('q1:7')   #1:q1:(n+1) -> q1,q2,q3...,qn
    # rospy.set_param('test',q[0])
    theta = Symbol('theta')
    alpha = Symbol('alpha')
    d = Symbol('d')
    a = Symbol('a')
    #### Depends on manipulator
    #### unit mm
    theta_val = [q[0],q[1]-(pi/2),q[2],q[3],q[4],q[5]+pi]
    d_val = [290,0,0,302,0,72]
    alpha_val = [-pi/2,0,-pi/2,pi/2,-pi/2,0]
    a_val = [0,270,70,0,0,0]
    jointType = [1,1,1,1,1,1]  # revolute -1 and prisimatic -0

    #### Depends on Haptics
    #### unit mm
    theta_val = [q[0]+pi,q[1],q[2]+(pi/2),q[3],q[4]+(pi/2),q[5]]
    d_val = [-132.1,0,0,132.1,0,30.0]
    alpha_val = [-pi/2,0,pi/2,pi/2,pi/2,0]
    a_val = [0,132.1,0,0,0,0]
    jointType = [1,1,1,1,1,1]  # revolute -1 and prisimatic -0

    # n=3
    # q = symbols('q1:4')   #1:q1:(n+1) -> q1,q2,q3...,qn
    # theta_val = [q[0],q[1],q[2]]
    # d_val = [0,0,0]
    # alpha_val = [0,0,0]
    # a_val = [80,40,20]
    # jointType = [1,1,1]  # revolute -1 and prisimatic -0

    Rz = Matrix(3,n,[0,0,0]*n)
    for i in range(n):
        if i==0:
            H = dh2mat(theta,d,alpha,a)
        else:
            H = H*dh2mat(theta,d,alpha,a)
        H = H.subs(theta,theta_val[i])
        H = H.subs(alpha,alpha_val[i])
        H = H.subs(d,d_val[i])
        H = H.subs(a,a_val[i])
        Rz[0:3,i] = jointType[i]*H[0:3,2]
    H =  simplify(H)
    print "\nHomogeneous Transformation matrix: \n",H
    # simplifyH(H,q)
    print "\nFull-Jacobian\n"
    full_jacobi = full_jacobian(H,q,Rz)
    print full_jacobi
    # print "\nFull jacobian shape: ",full_jacobi.shape
    # simplifyH(full_jacobi,q)
    # rospy.spin()
