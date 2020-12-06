#!/usr/bin/env python3
#
#   finger_counting.py
#
#
#   Parameters: /robot_description      URDF
#
import rospy

import numpy as np
from numpy.linalg import inv

from hw6code.kinematics import Kinematics #Check if this library actually exists or if you need to make a setup.py for it

from sensor_msgs.msg   import JointState

#
#  Basic Rotation Matrices
#
def Rx(theta):
    return np.array([[ 1, 0            , 0            ],
                     [ 0, np.cos(theta),-np.sin(theta)],
                     [ 0, np.sin(theta), np.cos(theta)]])

def Ry(theta):
    return np.array([[ np.cos(theta), 0, np.sin(theta)],
                     [ 0            , 1, 0            ],
                     [-np.sin(theta), 0, np.cos(theta)]])

def Rz(theta):
    return np.array([[ np.cos(theta), -np.sin(theta), 0 ],
                     [ np.sin(theta), np.cos(theta) , 0 ],
                     [ 0            , 0             , 1 ]])

#
#  Simple Vector
#
#  Just collect a 3x1 column vector
#
def vec(x,y,z):
    return np.array([[x], [y], [z]])


#  6x1 Error Computation
#
#  Note the 3x1 translation is on TOP of the 3x1 rotation error!
#
#  Also note, the cross product does not return a column vector but
#  just a one dimensional array.  So we need to convert into a 2
#  dimensional matrix, and transpose into the column form.  And then
#  we use vstack to stack vertically...
#
def etip(p, pd, R, Rd):
    ep  = pd - p
    eR1 = 0.5 * (np.cross(R[:,0], Rd[:,0]) +
                 np.cross(R[:,1], Rd[:,1]) +
                 np.cross(R[:,2], Rd[:,2]))
    eR  = np.matrix.transpose(np.atleast_2d(eR1))
    return np.vstack((ep,eR))


# Cubic Function Coefficient Calculator
#
# Takes in position and velocity values p0 and v0 at time t0
# and position and velocity values pf and vf at time tf and
# returns the four coefficients for the function.

def cubic_coeff(t0, tf, p0, pf, v0, vf):
	Dt = tf - t0
	Y  = np.array([[1, 0 , 0    , 0     ], 
				   [0, 1 , 0    , 0     ],
				   [1, Dt, Dt**2, Dt**3 ], 
				   [0, 1 , 2*Dt , 2*Dt**2]] )
	Yinv = np.linalg.inv(Y)
	coeff = Yinv @ np.array([[p0], [pf], [v0], [vf]])
	return coeff









