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

	return pd

def gen_path(t, tf, p0, pgoal):
    path = p0 + (2*(p0 - pgoal)/tf**3)*t**3 + (3*(pgoal - p0)/tf**2)*t**2
    return path


print(desired(0.017,2.22,vec(-2,7.3,0.001),vec(2,-0.77,51)))

# print(desired_path(2,0.01,np.array([[0],[0],[0]]),np.array([[5],[5],[5]])))



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