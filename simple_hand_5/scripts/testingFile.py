import numpy as np

def vec(x,y,z):
    return np.array([[x], [y], [z]])

def cubic_coeff(dt, p0, pf, v0, vf):
    Y  = np.array([[1, 0 , 0    , 0     ], 
                   [0, 1 , 0    , 0     ],
                   [1, dt, dt**2, dt**3 ], 
                   [0, 1 , 2*dt , 3*dt**2]] )
    Yinv = np.linalg.pinv(Y)
    coeff = Yinv @ np.array([p0, pf, v0, vf])
    return coeff

def desired_path(t, dt, p0, pgoal):
    c_t_x = cubic_coeff(dt, p0[0], pgoal[0], np.array([0]), np.array([0]))
    c_t_y = cubic_coeff(dt, p0[1], pgoal[1], np.array([0]), np.array([0]))
    c_t_z = cubic_coeff(dt, p0[2], pgoal[2], np.array([0]), np.array([0]))

    coeffMat = np.array([c_t_x, c_t_y, c_t_z])

    pd = np.array([1, t, t**2, t**3]) @ coeffMat
    vd = np.array([0, 1, 2*t, 3*t**2]) @ coeffMat

    return (pd, vd)

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


def rotation(t, tf, R0, Rf):
	R = np.zeros((3,3))
	for i in range(len(R[1])):
		for j in range(len(R[1])):
			R[i,j] =(gen_path(t,tf,R0[i,j],Rf[i,j]))
	return R
    

def rot_path(t, tf, desiredNum, finger, R0, handState=1):
    (ax, ay, az) = (0, 0, 0)
    if desiredNum == 9 and finger == 'index':
        ax = 2.09
        
    elif desiredNum == 8 and finger == 'middle':
        ax = 2.27

    #TO DO: FIND ROTATIONS FOR RING AND PINKY
    elif desiredNum == 7 and finger == 'ring':
        ax = 2.64

    elif desiredNum == 6 and finger == 'pinky':
        ax = 2.64

    elif finger == 'thumb':
        if desiredNum == 9:
            (ax, ay, az) = (0.3, 0.4, 1.3)

        elif desiredNum == 8:
            #TODO
            (ax, ay, az) = (0.85, 0.6, 0.75)

        elif desiredNum == 7:
            #TODO
            (ax, ay, az) = (1.4, 0.5, 0.5)

        elif desiredNum == 6:
            #TOOD
            (ax, ay, az) = (1.45, 0.55, 0.6)


    elif desiredNum not in [6, 7, 8, 9]:
        print("Invalid Number Entered")

    if handState == 0:
        (ax, ay, az) = (-ax, -ay, -az)
        Rd = R0 @ Rz(az * t/tf) @ Ry(ay * t/tf) @ Rx(ax *t/tf)

    else:
        Rd = R0 @ Rx(ax * t/tf) @ Ry(ay * t/tf) @ Rz(az *t/tf)
    wd = vec(ax, ay, az)/tf

    return (Rd, wd)

testArray1 = np.array([[[0],[1],[0.7]],[[0.85],[0.6],[-0.5]],[[0.5],[-0.3],[0]]])
testArray2 = np.array([[[0.8],[1],[-0.2]],[[0.05],[0.56],[-0.25]],[[0.57],[-0.03],[0.1]]])

print(rotation(0,2,testArray1,testArray2))

#print(desired(0.017,2.22,vec(-2,7.3,0.001),vec(2,-0.77,51)))

#print(desired_path(0.017,2.22,vec(-2,7.3,0.001),vec(2,-0.77,51)))



# def vec(x,y,z):
#     return np.array([[x], [y], [z]])

# def plane_vec(point1, point2, point3):
# 	vec1 = point1 - point2
# 	vec2 = point1 - point3
# 	nVec = np.zeros((4,1))
# 	nVec[0:3,:] = np.cross(vec1.T, vec2.T).T
# 	nVec[3,:] = -(nVec[0]*point1[0] + nVec[1]*point1[1] + nVec[2]*point1[2])
# 	return(nVec)

# def plane_proximity(pos, plane):
# 	d = np.abs(plane[0]*pos[0] + plane[1]*pos[1] + \
# 		plane[2]*pos[2] + plane[3])/np.sqrt(plane[0]**2 + \
# 		plane[1]**2 + plane[2]**2)
# 	return(d)



# def closest_plane(pos):
#     finger_list = np.array([[9], [8], [7], [6]])
#     dist_index = plane_proximity(pos, index_plane)
#     dist_middle = plane_proximity(pos, middle_plane)
#     dist_ring = plane_proximity(pos, ring_plane)
#     dist_pinky = plane_proximity(pos, pinky_plane)
#     return finger_list[np.argmin(np.array([[dist_index], [dist_middle], [dist_ring], \
#     	[dist_pinky]]))]

# index_plane = plane_vec(vec(-0.0295, -0.000655, 0.13), vec(-0.0328, -0.0531, 0.1), \
# 	vec(-0.0299, -0.0556, 0.0844))
# middle_plane = plane_vec(vec(-0.0102, 0.000551, 0.13), vec(-0.009, -0.07, 0.101), \
# 	vec(-0.011, -0.0745, 0.0692))
# ring_plane = plane_vec(vec(0.0403, -0.000453, 0.124), vec(0.0253, -0.0795, 0.0694), \
# 	vec(0.0145, -0.0804, 0.0247))
# pinky_plane = plane_vec(vec(0.0683, -0.000504, 0.106), vec(0.0441, -0.0613, 0.0542), \
# 	vec(0.0284, -0.0656, 0.0137))



# print(closest_plane(vec(0.0251,-0.0356,0.00437)))

# print(plane_proximity(vec(0,1,5),plane_vec(vec(1,2,3),vec(1,3,4),vec(0,0,2))))
# print(plane_vec(vec(1,2,3),vec(1,3,4),vec(0,0,2)))