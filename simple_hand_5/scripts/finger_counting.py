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
#  Joint States Publisher
#
#  Isolate the ROS message publisher to keep the main code simpler.
#
class JointStatePublisher:
    def __init__(self, names):
        # Save the dofs = number of names.
        self.n = len(names)

        # Create a publisher to send the joint values (joint_states).
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=100)

        # Wait until connected.  You don't have to wait, but the first
        # messages might go out before the connection and hence be lost.
        rospy.sleep(0.25)

        # Create a joint state message.
        self.msg = JointState()

        # You have to explicitly name each joint: Keep appending to
        # create a list.
        for i in range(self.n):
            self.msg.name.append(names[i])

        # We should also prepare the position list, initialize to zero.
        for i in range(self.n):
            self.msg.position.append(0.0)

        # Report.
        rospy.loginfo("Ready to publish /joint_states with %d DOFs", self.n)

    def dofs(self):
        # Return the number of DOFs.
        return self.n

    def send(self, q):
        # Set the positions.
        for i in range(self.n):
            self.msg.position[i] = q[i]

        # Send the command (with specified time).
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)

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
    # return np.vstack((ep,eR))  Add this back in if doing rotation orientations as well
    return ep

# Cubic Function Coefficient Calculator
#
# Takes in position and velocity values p0 and v0 at time t0
# and position and velocity values pf and vf at time tf and
# returns the four coefficients for the function.

def cubic_coeff(dt, p0, pf, v0, vf):
	Y  = np.array([[1, 0 , 0    , 0     ], 
				   [0, 1 , 0    , 0     ],
				   [1, dt, dt**2, dt**3 ], 
				   [0, 1 , 2*dt , 2*dt**2]] )
	Yinv = np.linalg.inv(Y)
	coeff = Yinv @ np.array([[p0], [pf], [v0], [vf]])
	return coeff

def desired_path(t, dt, p0, pgoal):
	c_t_x = cubic_coeff(dt, p0[0], pgoal[0], 0, 0)
	c_t_y = cubic_coeff(dt, p0[1], pgoal[1], 0, 0)
	c_t_z = cubic_coeff(dt, p0[2], pgoal[2], 0, 0)

	coeffMat = np.array([c_t_x, c_t_y, c_t_z])

	pd = np.array([1, t, t**2, t**3]) @ coeffMat
	vd = np.array([0, 1, 2*t, 3*t**2]) @ coeffMat

	return (pd, vd)

#
#  Main Code
#
if __name__ == "__main__":

    #  LOGISTICAL SETUP
    #
    # Prepare the node.
    rospy.init_node('finger_counting')
    rospy.loginfo("Starting the code for finger counting...")

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running with a loop dt of %f seconds (%fHz)" %
                  (dt, rate))

    # Set up the kinematics, from world to tip.
    urdf = rospy.get_param('/robot_description')
    kin_thumb  = Kinematics(urdf, 'base_link', 'thumb_3')
    N_thumb    = kin_thumb.dofs()

    kin_index = Kinematics(urdf, 'base_link', 'index_3')
    N_index = kin_index.dofs()

    kin_middle = Kinematics(urdf, 'base_link', 'middle_3')
    N_middle = kin_middle.dofs()

    kin_ring = Kinematics(urdf, 'base_link', 'ring_3')
    N_ring = kin_ring.dofs()

    kin_pinky = Kinematics(urdf, 'base_link', 'pinky_3')
    N_pinky = kin_pinky.dofs()

    # Allocate the memory for the numpy variables for tip position,
    # tip orientation, and Jacobian.  The kinematics code changes the
    # data in place, so these need to be allocated!  But the content
    # will be overwritten so the initial values are irrelevant.
    p_thumb = np.zeros((3,1))
    R_thumb = np.identity(3)
    J_thumb = np.zeros((6,N_thumb))

    p_index = np.zeros((3,1))
    R_index = np.identity(3)
    J_index = np.zeros((6,N_index))

    p_middle = np.zeros((3,1))
    R_middle = np.identity(3)
    J_middle = np.zeros((6,N_middle))

    p_ring = np.zeros((3,1))
    R_ring = np.identity(3)
    J_ring = np.zeros((6,N_ring))

    p_pinky = np.zeros((3,1))
    R_pinky = np.identity(3)
    J_pinky = np.zeros((6,N_pinky))

    # Set up the publisher, naming the joints!
    pub = JointStatePublisher(('thumb_palm', 'thumb_palm_updown', 'thumb_12', 'thumb_23',
                               'index_palm', 'index_12', 'index_23', 
                               'middle_palm', 'middle_12', 'middle_23', 
                               'rp_palm', 'ring_rp', 'ring_12', 'ring_23',
                               'pinky_rp', 'pinky_12', 'pinky_23'))

    # Make sure the URDF and publisher agree in dimensions.
    if not pub.dofs() == N_thumb + N_index + N_ring + N_pinky - 1:
        rospy.logerr("FIX Publisher to agree with URDF!")

    # Set the numpy printing options (as not to be overly confusing).
    # This is entirely to make it look pretty (in my opinion).
    np.set_printoptions(suppress = True, precision = 6)


# Close finger positions
# Used fkin to establish the closed positions to calculate errors/know where you are aiming for in the task space


theta_ti = np.array([[1.51], [0], [0.11], [0.18],
					 [0], [0.87], [1.42], 
					 [0], [0], [0], 
					 [0], [0], [0], [0],
					 [0], [0], [0]])

theta_tm = np.array([[1.12], [-0.38], [0.27], [0.31]
					 [0], [0], [0],
					 [0], [1.31], [1.16],
					 [0], [0], [0], [0],
					 [0], [0], [0]])

theta_tr = np.array([[1.19], [-0.91], [0.20], [0.30]
					 [0], [0], [0],
					 [0], [0], [0],
					 [0], [0.42], [1.33], [1.09],
					 [0], [0], [0]])

theta_tp = np.array([[1.41], [-1.07], [0.14], [0.13],
					 [0], [0], [0],
					 [0], [0], [0],
					 [0], [0], [0], [0],
					 [0.69], [1.17], [1.03]])


theta_start = np.zeros(pub.dofs(), 1)

# Find all the positions for an open hand









#
# Pseudo Code for testing
#



	while not rospy.is_shutdown():
		th_thumb = theta[0:3, :]
		th_index = theta[4:6, :]
		th_middle = theta[7:9, :]
		th_ring = theta[10:13, :]
		th_pinky = np.array([[theta[10], theta[14:16]]])

		# Update the locations of each tip
		kin_thumb.fkin(th_thumb, p_thumb, R_thumb)
		kin_index.fkin(th_index, p_index, R_index)
		kin_middle.fkin(th_middle, p_middle, R_middle)
		kin_ring.fkin(th_ring, p_ring, R_ring)
		kin_pinky.fkin(th_pinky, p_pinky, R_pinky)

		# Calculate all the errors from the previous step:

		e_thumb = etip(p_thumb, pd_thumb, R_thumb, Rd_thumb)
		e_index = etip(p_index, pd_index, R_index, Rd_index)
		e_middle = etip(p_middle, pd_middle, R_middle, Rd_middle)
		e_ring = etip(p_ring, pd_ring, R_ring, Rd_ring)
		e_pinky = etip(p_pinky, pd_pinky, R_pinky, Rd_pinky)

		# Read some input, either of thumb position + closed state, or just what number to be achieved

		# TODO: Set up subscriber to a gui? 
			# Would want the gui to not constantly send messages, but only send a message when 
			# a button is pressed like enter to send message
			# Inputs from gui:
				# Current position of thumb? (let person move thumb in designated x, y, z way)
				# Whether open or close finger count
				# 
				# OR
				#
				# Current position of thumb?
				# What number to go to?

		# From message, determine which finger to close (if closed == true)
			# If 1st option, determine what the closest finger is to close with thumb
			
		#	if condition open vs. closed changed:
				p_t0 = p_thumb
				p_i0 = p_index
				p_m0 = p_middle
				p_r0 = p_ring
				p_p0 = p_pinky

				#TBH all the index through pinky fingers are likely going to have p_x0 be open or closed
 				
 				p_thumb_goal = p_thumb_open
				p_index_goal = p_index_open
				p_middle_goal = p_middle_open
				p_ring_goal = p_ring_open
				p_pinky_goal = p_pinky_open


			# if closed && 9
				p_thumb_goal = p_thumb_ti
				p_index_goal = p_index_ti

			# else if closed && 8
				p_thumb_goal = p_thumb_tmn
				p_middle_goal = p_middle_tm

			# else if closed && 7
				p_thumb_goal = p_thumb_tr
				p_ring_goal = p_ring_tr

			# else if closed && 6
				p_thumb_goal = p_thumb_tp
				p_pinky_goal = p_pinky_tp

		# Now that we have all the goals, we can calculate the desired positions

		(pd_thumb, vd_thumb) = desired(t, total_t, p_t0, p_thumb_goal)
		(pd_index, vd_index) = desired(t, total_t, p_i0, p_index_goal)
		(pd_middle, vd_middle) = desired(t, total_t, p_m0, p_middle_goal)
		(pd_ring, vd_ring) = desired(t, total_t, p_r0, p_ring_goal)
		(pd_pinky, vd_pinky) = desired(t, total_t, p_r0, p_ring_goal)

		# From these desired positions and velocities, we can probably get:

		# Calculate thetas for the thumb position
		vr_thumb = vd_thumb + lam * e_thumb   # 3 x 1 column vector
		Jv_thumb = J_thumb[0:2, :]      # 3 x dofs matrix
		Jvinv_thumb = np.linalg.pinv(Jv_thumb) # dofs x 3 matrix
		theta_dot_thumb = Jvinv_thumb @ vr_thumb     # dofs x 1 column vector
		theta_thumb = theta_dot_thumb * dt     # theta_palm, theta_palm_updown, theta_12, theta_23

		# Calculate thetas for index position
		vr_index = vd_index + lam * e_index   # 3 x 1 column vector
		Jv_index = J_index[0:2, :]      # 3 x dofs matrix
		Jvinv_index = np.linalg.pinv(Jv_index) # dofs x 3 matrix
		theta_dot_index = Jvinv_index @ vr_index     # dofs x 1 column vector
		theta_index = theta_dot_index * dt     # index_palm, index_12, index_23

		# Calculate thetas for middle position
		vr_middle = vd_middle + lam * e_middle   # 3 x 1 column vector
		Jv_middle = J_middle[0:2, :]      # 3 x dofs matrix
		Jvinv_middle = np.linalg.pinv(Jv_middle) # dofs x 3 matrix
		theta_dot_middle = Jvinv_middle @ vr_middle     # dofs x 1 column vector
		theta_middle = theta_dot_middle * dt     # middle_palm, middle_12, middle_23

		# Calculate thetas for ring position
		vr_ring = vd_ring + lam * e_ring   # 3 x 1 column vector
		Jv_ring = J_ring[0:2, :]      # 3 x dofs matrix
		Jvinv_ring = np.linalg.pinv(Jv_ring) # dofs x 3 matrix
		theta_dot_ring = Jvinv_ring @ vr_ring     # dofs x 1 column vector
		theta_ring = theta_dot_ring * dt     # rp_palm, ring_rp, ring_12, ring_23

		# Calculate thetas for pinky position
		vr_pinky = vd_pinky + lam * e_pinky   # 3 x 1 column vector
		Jv_pinky = J_pinky[0:2, :]      # 3 x dofs matrix
		Jvinv_pinky = np.linalg.pinv(Jv_pinky) # dofs x 3 matrix
		theta_dot_pinky = Jvinv_pinky @ vr_pinky     # dofs x 1 column vector
		theta_pinky = theta_dot_pinky * dt     # rp_palm, pinky_rp, pinky_12, pinky_23


		# NOTE: RIGHT NOW RP_PALM IS IN BOTH THE PINKY BEND AND THE RING FINGER BEND.
		# THIS MAY NOT BE AN ISSUE ARE RIGHT NOW RP_PALM IS BEING TREATED AS FIXED, BUT IF 
		# WE ACTUALLY FIXED THIS JOINT THEN WE MAY AVOID ERRORS. FOR NOW, WE'RE GOING TO ASSUME IT 
		# AS BEING FIXED, ST. theta_pinky[0] == theta_ring[0]

		theta = np.vstack((theta_thumb, theta_index, theta_middle, theta_ring, theta_pinky[1:N_pinky-1]))

