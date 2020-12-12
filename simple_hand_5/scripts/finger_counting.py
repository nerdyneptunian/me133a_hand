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

import math as m

from hw6code.kinematics import Kinematics #Check if this library actually exists or if you need to make a setup.py for it

from sensor_msgs.msg   import JointState

from hand_math import *

def kin_var_allocate(N):
    p = np.zeros((3,1))
    R = np.identity(3)
    J = np.zeros((6,N))
    Jv = J[0:3, :]
    p_open = np.zeros((3,1))
    R_open = np.identity(3)
    theta_open = np.zeros((N,1))
    theta_open[N-2:N] = 0.1*np.ones((2, 1))

    return (p, R, J, p_open, R_open, theta_open)

def theta_finger(th_0, vd_fing, wd_fing, J_fing, e_fing, lam, right_pseudo=False):
        # From these desired positions and velocities, we can probably get:
        # Calculate thetas for index position
        vr_fing = np.vstack((vd_fing, wd_fing)) + lam * e_fing   # 3 x 1 column vector
        if right_pseudo:
            gamma = 0.1
            Jt_fing = np.transpose(J_fing)
            # J_Jt = J_fing @ Jt_fing
            # J_pinv = Jt_fing @ np.linalg.inv(J_Jt)
            theta_dot_fing = np.linalg.pinv(Jt_fing @ J_fing + gamma**2 * np.identity(4)) @ Jt_fing @ vr_fing
        else:
            J_pinv = np.linalg.pinv(J_fing) # dofs x 3 matrix
            theta_dot_fing = J_pinv @ vr_fing     # dofs x 1 column vector

        th_fing = theta_dot_fing * dt + th_0 

        return th_fing


#  6x1 Error Computation
#
def etip(p, pd, R, Rd):
    # add back in R, Rd to above if rotation
    ep  = pd - p
    eR1 = 0.5 * (np.cross(R[:,0], Rd[:,0]) +
                np.cross(R[:,1], Rd[:,1]) +
                np.cross(R[:,2], Rd[:,2]))
    eR  = np.matrix.transpose(np.atleast_2d(eR1))
    return np.vstack((ep,eR))  
    # return ep



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

    # Set up the kinematics, from world to filange tips.
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
    (p_thumb, R_thumb, J_thumb, p_thumb_open, R_thumb_open, theta_thumb_open) = kin_var_allocate(N_thumb)

    (p_index, R_index, J_index, p_index_open, R_index_open, theta_index_open) = kin_var_allocate(N_index)

    (p_middle, R_middle, J_middle, p_middle_open, R_middle_open, theta_middle_open) = kin_var_allocate(N_middle)

    (p_ring, R_ring, J_ring, p_ring_open, R_ring_open, theta_ring_open) = kin_var_allocate(N_ring)
    
    (p_pinky, R_pinky, J_pinky, p_pinky_open, R_pinky_open, theta_pinky_open) = kin_var_allocate(N_pinky)
    
    

    # Set up the publisher, naming the joints!
    pub = JointStatePublisher(('thumb_palm', 'thumb_palm_updown', 'thumb_12', 'thumb_23',
                               'index_palm', 'index_12', 'index_23', 
                               'middle_palm', 'middle_12', 'middle_23', 
                               'ring_rp', 'ring_12', 'ring_23',
                               'pinky_rp', 'pinky_12', 'pinky_23'))

    # Make sure the URDF and publisher agree in dimensions.
    if not pub.dofs() == N_thumb + N_index + N_ring + N_pinky + N_middle:
        rospy.logerr("FIX Publisher to agree with URDF!")


    fj_thumb = 0
    fj_index = N_thumb
    fj_middle = fj_index + N_index 
    fj_ring = fj_middle + N_middle
    fj_pinky = fj_ring + N_ring
    tot_joints = pub.dofs()

    theta = np.zeros((tot_joints,1))
    theta_thumb_open[1, :] = -0.1

    # Set the numpy printing options (as not to be overly confusing).
    # This is entirely to make it look pretty (in my opinion).
    np.set_printoptions(suppress = True, precision = 6)


    # 
    #   Close finger positions
    #

    #   Used fkin to establish the closed positions to calculate errors
    #   /know where you are aiming for in the task space
    
    # for number 9
    theta_ti = np.array([[1.51], [0], [0.11], [0.18],
                         [0],[0.87],[1.42],
                         [0],[0],[0], 
                         [0], [0], [0],
                         [0], [0], [0]])

    p_index_ti = np.zeros((3,1))
    p_thumb_ti = np.zeros((3,1))
    R_index_ti = np.identity(3)
    R_thumb_ti = np.identity(3)
    kin_index.fkin(theta_ti[fj_index:fj_middle], p_index_ti, R_index_ti)
    kin_thumb.fkin(theta_ti[fj_thumb:fj_index], p_thumb_ti, R_thumb_ti)

    # for number 8
    theta_tm = np.array([[1.12], [-0.38], [0.27], [0.31],\
                         [0], [0], [0],
                         [0], [1.31], [1.16],
                         [0], [0], [0],
                         [0], [0], [0]])

    p_middle_tm = np.zeros((3,1))
    p_thumb_tm = np.zeros((3,1))    
    R_middle_tm = np.identity(3)
    R_thumb_tm = np.identity(3)
    kin_middle.fkin(theta_tm[fj_middle:fj_ring], p_middle_tm, R_middle_tm)
    kin_thumb.fkin(theta_tm[fj_thumb:fj_index], p_thumb_tm, R_thumb_tm)


    # for number 7
    theta_tr = np.array([[1.19], [-0.91], [0.20], [0.30], \
                         [0], [0], [0],
                         [0], [0], [0],
                         [0.42], [1.33], [1.09],
                         [0], [0], [0]])

    p_ring_tr = np.zeros((3,1))
    p_thumb_tr = np.zeros((3,1))
    R_ring_tr = np.identity(3)
    R_thumb_tr = np.identity(3)
    kin_ring.fkin(theta_tr[fj_ring:fj_pinky], p_ring_tr, R_ring_tr)
    kin_thumb.fkin(theta_tr[fj_thumb:fj_index], p_thumb_tr, R_thumb_tr)



    # for number 6
    theta_tp = np.array([[1.41], [-1.07], [0.14], [0.13], \
                         [0], [0], [0],
                         [0], [0], [0],
                         [0], [0], [0],
                         [0.69], [1.17], [1.03]])

    p_pinky_tp = np.zeros((3,1))
    p_thumb_tp = np.zeros((3,1))
    R_pinky_tp = np.identity(3)
    R_thumb_tp = np.identity(3)
    kin_pinky.fkin(theta_tp[fj_pinky:tot_joints], p_pinky_tp, R_pinky_tp)
    kin_thumb.fkin(theta_tp[fj_thumb:fj_index], p_thumb_tp, R_thumb_tp)


    # Open hand positions

    kin_thumb.fkin(theta_thumb_open, p_thumb_open, R_thumb_open)
    kin_index.fkin(theta_index_open, p_index_open, R_index_open)
    kin_middle.fkin(theta_middle_open, p_middle_open, R_middle_open)
    kin_ring.fkin(theta_ring_open, p_ring_open, R_ring_open)
    kin_pinky.fkin(theta_pinky_open, p_pinky_open, R_pinky_open)

    theta_open = np.vstack((theta_thumb_open, theta_index_open, theta_middle_open, theta_ring_open, theta_pinky_open))

    # finger planes
    # we can make these more robust later with actual positions, but I 
    # got most of these from the rviz

    # index_plane = plane_vec(p_index_open, vec(-0.0328, -0.0531, 0.1), \
    #     vec(-0.0299, -0.0556, 0.0844))
    # middle_plane = plane_vec(p_middle_open, vec(-0.009, -0.07, 0.101), \
    #     vec(-0.011, -0.0745, 0.0692))
    # ring_plane = plane_vec(p_ring_open, vec(0.0253, -0.0795, 0.0694), \
    #     vec(0.0145, -0.0804, 0.0247))
    # pinky_plane = plane_vec(p_pinky_open, vec(0.0441, -0.0613, 0.0542), \
    #     vec(0.0284, -0.0656, 0.0137))


# Determining Start Theta (For Non-Gui Application)
    h_open = 0
    h_close = 1
    h_free = 2        

    desiredNum = 6 #we can change this later to incorporate the thumb position
    handState = 0

    if handState == 1:
        if desiredNum == 9:
            theta = theta_ti
        elif desiredNum == 8:
            theta = theta_tm
        elif desiredNum == 7:
            theta = theta_tr
        elif desiredNum == 6:
            theta = theta_tp

    else:
        theta = theta_open

# Initializing the desired velocities and positions
    t = 0.0
    t_sub = t
    lam = 0.1/dt
    total_t = 1
    tf = 1.0
    step_no = 1

    (pd_thumb, vd_thumb) = desired(0, total_t, p_thumb_open, p_thumb_open)
    (pd_index, vd_index) = desired(0, total_t, p_index_open, p_index_open)
    (pd_middle, vd_middle) = desired(0, total_t, p_middle_open, p_middle_open)
    (pd_ring, vd_ring) = desired(0, total_t, p_ring_open, p_ring_open)
    (pd_pinky, vd_pinky) = desired(0, total_t, p_pinky_open, p_pinky_open)

    Rd_thumb = R_thumb
    Rd_index = R_index_open
    Rd_middle = R_middle_open
    Rd_ring = R_ring_open
    Rd_pinky = R_pinky_open

    p_t0 = np.zeros((3, 1))
    p_i0 = np.zeros((3, 1)) 
    p_m0 = np.zeros((3, 1))
    p_r0 = np.zeros((3, 1))
    p_p0 = np.zeros((3, 1))

    R_t0 = np.identity(3)
    R_i0 = np.identity(3)
    R_m0 = np.identity(3)
    R_r0 = np.identity(3)
    R_p0 = np.identity(3)


    num_changed = True

    #
    #   Main Time Loop
    #

    while not rospy.is_shutdown():
        print(t)

        th_thumb = theta[fj_thumb:fj_index, :]
        th_index = theta[fj_index:fj_middle, :]
        th_middle = theta[fj_middle:fj_ring, :]
        th_ring = theta[fj_ring:fj_pinky, :]
        th_pinky = theta[fj_pinky:tot_joints]

        # Update the locations of each tip
        kin_thumb.fkin(th_thumb, p_thumb, R_thumb)
        kin_index.fkin(th_index, p_index, R_index)
        kin_middle.fkin(th_middle, p_middle, R_middle)
        kin_ring.fkin(th_ring, p_ring, R_ring)
        kin_pinky.fkin(th_pinky, p_pinky, R_pinky)

        # print("th middle: ",th_middle)
        # print("p goal: ", p_pinky_open)

        kin_thumb.Jac(th_thumb, J_thumb)
        kin_index.Jac(th_index, J_index) 
        kin_middle.Jac(th_middle, J_middle)
        kin_ring.Jac(th_ring, J_ring)
        kin_pinky.Jac(th_pinky, J_pinky) 

        # Calculate all the errors from the previous step:

        e_thumb = etip(p_thumb, pd_thumb, R_thumb, Rd_thumb)
        e_index = etip(p_index, pd_index, R_index, Rd_index)
        e_middle = etip(p_middle, pd_middle, R_middle, Rd_middle)
        e_ring = etip(p_ring, pd_ring, R_ring, Rd_ring)
        e_pinky = etip(p_pinky, pd_pinky, R_pinky, Rd_pinky)

        # print("Error Index: ", e_index)

        t+= dt
        t_sub+= dt

        desiredNumNew = 6 + m.floor((step_no - 1)/2)
        handStateNew = (step_no) % 2
       
            
        if desiredNum != desiredNumNew or handState != handStateNew:
            kin_thumb.fkin(th_thumb, p_t0, R_t0)
            kin_index.fkin(th_index, p_i0, R_i0)
            kin_middle.fkin(th_middle, p_m0, R_m0)
            kin_ring.fkin(th_ring, p_r0, R_r0)
            kin_pinky.fkin(th_pinky, p_p0, R_p0)
            print("Initial Conditions logged! Number: ", desiredNumNew, " Hand State: ", handStateNew)
            desiredNum = desiredNumNew
            handState = handStateNew
        
        if handState == h_free:

            (p_index_goal, R_index_goal) = (p_index_open, R_index_open)
            (p_middle_goal, R_middle_goal) = (p_middle_open, R_middle_open)
            (p_ring_goal, R_ring_goal) = (p_ring_open, R_ring_open)
            (p_pinky_goal, R_pinky_goal) = (p_pinky_open, R_pinky_open)

            # placeholder vector rn until we have a gui/something to give us the joint states
            thumbInput = vec(1,-1,1,1) #some vector in the right tange 
            theta_thumb = thumbInput


        else:
            
            #TBH all the index through pinky fingers are likely going to have p_x0 be open or closed
                # make the goal to be open positions (need to define these)
            (p_thumb_goal, R_thumb_goal) = (p_thumb_open, R_thumb_open)
            (p_index_goal, R_index_goal) = (p_index_open, R_index_open)
            (p_middle_goal, R_middle_goal) = (p_middle_open, R_middle_open)
            (p_ring_goal, R_ring_goal) = (p_ring_open, R_ring_open)
            (p_pinky_goal, R_pinky_goal) = (p_pinky_open, R_pinky_open)

            if handState == h_close:
                if desiredNum == 9:
                # if closed && 9
                    (p_thumb_goal, R_thumb_goal) = (p_thumb_ti, R_thumb_ti)
                    (p_index_goal, R_index_goal) = (p_index_ti, R_index_ti)
        
                elif desiredNum == 8:
                # else if closed && 8
                    (p_thumb_goal, R_thumb_goal) = (p_thumb_tm, R_thumb_tm)
                    (p_middle_goal, R_middle_goal) = (p_middle_tm, R_middle_tm)
                elif desiredNum == 7:
                # else if closed && 7
                    (p_thumb_goal, R_thumb_goal) = (p_thumb_tr, R_thumb_tr)
                    (p_ring_goal, R_ring_goal) = (p_ring_tr, R_ring_tr)
                elif desiredNum == 6:
                # else if closed && 6
                    (p_thumb_goal, R_thumb_goal) = (p_thumb_tp, R_thumb_tp)
                    (p_pinky_goal, R_pinky_goal) = (p_pinky_tp, R_pinky_tp)
            # because the free doesn't touch the thumb at all, we're going to put it here


            
        # Now that we have all the goals, we can calculate the desired positions (besides the thumb)
        (pd_thumb, vd_thumb) = desired(t_sub, total_t, p_t0, p_thumb_goal)
        (pd_index, vd_index) = desired(t_sub, total_t, p_i0, p_index_goal) #test with open instead
        (pd_middle, vd_middle) = desired(t_sub, total_t, p_m0, p_middle_goal)
        (pd_ring, vd_ring) = desired(t_sub, total_t, p_r0, p_ring_goal)
        (pd_pinky, vd_pinky) = desired(t_sub, total_t, p_p0, p_pinky_goal)

        (Rd_thumb, wd_thumb) = rot_path(t_sub, total_t, desiredNum, 'thumb', R_t0, handState)
        (Rd_index, wd_index) =  rot_path(t_sub, total_t, desiredNum, 'index', R_i0, handState)
        (Rd_middle, wd_middle) = rot_path(t_sub, total_t, desiredNum, 'middle', R_m0, handState)
        (Rd_ring, wd_ring) = rot_path(t_sub, total_t, desiredNum, 'ring', R_r0, handState)
        (Rd_pinky, wd_pinky) = rot_path(t_sub, total_t, desiredNum, 'pinky', R_p0, handState)

        # print('R_thumb_open: ', R_t0)
        # print('Actual R_thumb: ',  R_thumb)

        th_thumb = theta_finger(th_thumb, vd_thumb, wd_thumb, J_thumb, e_thumb, lam, True)
        th_index = theta_finger(th_index, vd_index, wd_index, J_index, e_index, lam)
        th_middle = theta_finger(th_middle, vd_middle, wd_middle, J_middle, e_middle, lam)
        th_ring = theta_finger(th_ring, vd_ring, wd_ring, J_ring, e_ring, lam)
        th_pinky = theta_finger(th_pinky, vd_pinky, wd_pinky, J_pinky, e_pinky, lam)



        theta = np.vstack((th_thumb, th_index, th_middle, th_ring, th_pinky))

       # print(theta)
        pub.send(theta)

        servo.sleep()
        

        if (t_sub >= tf):
            step_no+= 1
            t_sub = 0
        if (step_no > 8):
            break
