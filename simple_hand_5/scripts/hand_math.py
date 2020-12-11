#!/usr/bin/env python3
#
#   Hand Math Helper Functions
#
#   Collection of all the extra functions needed to calculate the motion
#   for the fingers. 
#

import numpy as np
from numpy.linalg import inv

import math as m

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


#
# Path Calculation
#

def desired(t, tf, p0, pgoal):
    pd_x = gen_path(t, tf, p0[0], pgoal[0])
    pd_y = gen_path(t, tf, p0[1], pgoal[1])
    pd_z = gen_path(t, tf, p0[2], pgoal[2])

    pd = vec(pd_x, pd_y, pd_z)

    vd_x = gen_vel(t, tf, p0[0], pgoal[0])
    vd_y = gen_vel(t, tf, p0[1], pgoal[1])
    vd_z = gen_vel(t, tf, p0[2], pgoal[2])

    vd = vec(vd_x, vd_y,vd_z)
    combined = np.array((pd,vd)).reshape((2,3,1))

    return combined

def gen_path(t, tf, p0, pgoal):
    path = p0 + (2*(p0 - pgoal)/tf**3)*t**3 + (3*(pgoal - p0)/tf**2)*t**2
    return path

def gen_vel(t, tf, p0, pgoal):
    vel = 6*((p0 - pgoal)*t*(t - tf))/tf**3
    return vel

def rot_path(t, tf, desiredNum, finger, R0):
    Rd = R0
    wd = np.zeros((3,1))
    if desiredNum == 9 and finger == 'index':
        alpha = 2.09
        Rd = R0 @ Rx(alpha * t/tf)
        wd = vec(alpha/tf, 0, 0)
    elif desiredNum == 8 and finger == 'middle':
        alpha = 0.81
        Rd = R0 @ Rx(alpha * t/tf)
        wd = vec(alpha/tf, 0, 0)

    #TO DO: FIND ROTATIONS FOR RING AND PINKY
    elif desiredNum == 7 and finger == 'ring':
        Rd = R0
    elif desiredNum == 6 and finger == 'pinky':
        Rd = R0
    elif desiredNum not in [6, 7, 8, 9]:
        print("Invalid Number Entered")


    return (Rd, wd)




#
#   Plane Calculation
#
def plane_proximity(pos, plane):
    # this takes in a position and plane vector and returns the distance between
    d = np.abs(plane[0]*pos[0] + plane[1]*pos[1] + \
        plane[2]*pos[2] + plane[3])/np.sqrt(plane[0]**2 + \
        plane[1]**2 + plane[2]**2)
    return(d)

def plane_vec(point1, point2, point3):
    # this takes in 3 points and returns the plane vector (of coefficients)
    vec1 = point1 - point2
    vec2 = point1 - point3
    nVec = np.zeros((4,1))
    nVec[0:3,:] = np.cross(vec1.T, vec2.T).T
    nVec[3,:] = -(nVec[0]*point1[0] + nVec[1]*point1[1] + nVec[2]*point1[2])
    return(nVec)

    
def closest_plane(pos):
    # this takes in the thumb tip position and returns which number it should go for
    finger_list = np.array([[9], [8], [7], [6]])
    dist_index = plane_proximity(pos, index_plane)
    dist_middle = plane_proximity(pos, middle_plane)
    dist_ring = plane_proximity(pos, ring_plane)
    dist_pinky = plane_proximity(pos, pinky_plane)
    return finger_list[np.argmin(np.array([[dist_index], [dist_middle], [dist_ring], \
        [dist_pinky]]))]