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
    return np.vstack((ep,eR))


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



