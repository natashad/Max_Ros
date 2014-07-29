#!/usr/bin/env python

# PURPOSE
# This ROS node takes in information from the WaveDNA software and converts it into trajectories for the drone to follow to complete the conductor motion.

# SUBSCRIBED TOPICS
# /terpsichore/ableton_time
# /terpsichore/conductor_time
# /waypoint_request

# PUBLISHED TOPICS
# /path_coordinates

# VERSION HISTORY
# Jul 11, 2014 - initialy created (Tristan Laidlow)


####################
# IMPORT LIBRARIES #
####################

# Import ROS libraries, rospy, and load manifest file for access to project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import math for trigonometric functions
import math

# Import time for synchronization
from time import time


###################
# IMPORT MESSAGES #
###################

# StateData is used to send desired coordinates to the drone
from ardrone_tutorials.msg import StateData

# Bool is used for waypoint requests and Float64 for time
from std_msgs.msg import Bool
from std_msgs.msg import Float64

# Pair is used to send time and signature from WaveDNA
from terpsichore.msg import pair


##################
# SET PARAMETERS #
##################

scale = 0.333


#########################
# MAIN CLASS DEFINITION #
#########################

class Conductor(object):
    def __init__(self):
        super(Conductor,self).__init__()
        print "hello"
        # Initialize state variables for the drone
        self.x = 0.0
        self.y = 0.0
        self.z = 1.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Manage time for synchronization
        self.cur_time = 0.0

        # Subscribe to the waypoint_request topic
        self.sub_request = rospy.Subscriber('/waypoint_request', Bool, self.process_request)

        # Subscribe to the WaveDNA data
        self.sub_wavedna = rospy.Subscriber('/terpsichore/conductor_time', pair, self.determine_state)
        self.sub_time = rospy.Subscriber('/terpsichore/ableton_time', Float64, self.update_time)

        # Publish to the desired_coordinates topic
        self.pub_desired = rospy.Publisher('/path_coordinates', StateData)

    # Keep time in sync with WaveDNA
    def update_time(self, new_time):
        self.cur_time = new_time.data

    # Process a request for a waypoint
    def process_request(self, unneeded):

        # Publish state
        state = StateData()

        state.x = self.x
        state.y = self.y
        state.z = self.z
        state.vx = self.vx
        state.vy = self.vy
        state.vz = self.vz
        state.ax = self.ax
        state.ay = self.ay
        state.az = self.az
        state.roll = self.roll
        state.pitch = self.pitch
        state.yaw = self.yaw

        self.pub_desired.publish(state)

    # Calculate the desired state based on the required trajectory
    def determine_state(self, cond):

        t = cond.t

        # Determine state for 4/4 time
        if cond.data[0] == 4.0: # or the tempo is too fast

            # Determine z
            A = 0.35
            B = 0.40
            C = 0.15
            D = 1.00

            if t < 2.0:
                self.z = (A / 2.0) * (1.0 - math.cos(2.0*math.pi*t))
                self.vz = A*math.pi*math.sin(2.0*math.pi*t)
                self.az = 2.0*A*math.pi*math.pi*math.cos(2.0*math.pi*t)
            elif t < 2.5:
                self.z = -16.0*B*math.pow(t-2.0,3) + 12.0*B*math.pow(t-2.0,2)
                self.vz = -48.0*B*math.pow(t-2.0,2) + 24.0*B*(t-2.0)
                self.az = -96.0*B*(t-2.0) + 24.0*B
            elif t < 3.0:
                self.z = 16.0*(B-C)*math.pow(t-2.5,3)-12.0*(B-C)*math.pow(t-2.5,2)+B
                self.vz = 48.0*(B-C)*math.pow(t-2.5,2)-24.0*(B-C)*(t-2.5)
                self.az = 96.0*(B-C)*(t-2.5) - 24.0*(B-C)
            elif t < 3.5:
                self.z = 16.0*(C-D)*math.pow(t-3.0,3)-12.0*(C-D)*math.pow(t-3.0,2)+C
                self.vz = 48.0*(C-D)*math.pow(t-3.0,2)-24.0*(C-D)*(t-3.0)
                self.az = 96.0*(C-D)*(t-3.0) - 24.0*(C-D)
            else:
                self.z = 16.0*D*math.pow(t-3.5,3)-12.0*D*math.pow(t-3.5,2)+D
                self.vz = 48.0*D*math.pow(t-3.5,2)-24.0*D*(t-3.5)
                self.az = 96.0*D*(t-3.5) - 24.0*D

            # Determine y
            L = -0.7
            R = 0.7

            if t < 0.3:
                self.y = 0.0
                self.vy = 0.0
                self.ay = 0.0
            elif t < 1.3:
                self.y = -2.0*L*math.pow(t-0.3,3) + 3.0*L*math.pow(t-0.3,2)
                self.vy = -6.0*L*math.pow(t-0.3,2) + 6.0*L*(t-0.3)
                self.ay = -12.0*L*(t-0.3) + 6.0*L
            elif t < 2.3:
                self.y = 2.0*(L-R)*math.pow(t-1.3,3)-3.0*(L-R)*math.pow(t-1.3,2)+L
                self.vy = 6.0*(L-R)*math.pow(t-1.3,2)-6.0*(L-R)*(t-1.3)
                self.ay = 12.0*(L-R)*(t-1.3)-6.0*(L-R)
            elif t < 3.5:
                self.y = (125.0/108.0)*R*math.pow(t-2.3,3)-(25.0/12.0)*R*math.pow(t-2.3,2)+R
                self.vy = (125.0/36.0)*R*math.pow(t-2.3,2)-(25.0/6.0)*R*(t-2.3)
                self.ay = (125.0/18.0)*R*(t-2.3)-(25.0/6.0)*R
            else:
                self.y = 0.0
                self.vy = 0.0
                self.ay = 0.0
 
        # Determine state for 3/4 time
        if cond.data[0] == 3.0:

            # Determine z
            A = 0.25
            B = 0.40
            C = 0.20
            D = 1.00

            if t < 1.0:
                self.z = (A/2.0)*(1.0 - math.cos(2.0*math.pi*t))
                self.vz = A*math.pi*math.sin(2.0*math.pi*t)
                self.az = 2.0*A*math.pi*math.pi*math.cos(2.0*math.pi*t)
            elif t < 1.5:
                self.z = -16.0*B*math.pow(t-1.0,3)+12.0*B*math.pow(t-1.0,2)
                self.vz = -48.0*B*math.pow(t-1.0,2)+24.0*B*(t-1.0)
                self.az = -96.0*B*(t-1.0)+24.0*B
            elif t < 2.0:
                self.z = 16.0*(B-C)*math.pow(t-1.5,3)-12.0*(B-C)*math.pow(t-1.5,2)+B
                self.vz = 48.0*(B-C)*math.pow(t-1.5,2)-24.0*(B-C)*(t-1.5)
                self.az = 96.0*(B-C)*(t-1.5)-24.0*(C-D)
            elif t < 2.5:
                self.z = 16.0*(C-D)*math.pow(t-2.0,3)-12.0*(C-D)*math.pow(t-2.0,2)+C
                self.vz = 48.0*(C-D)*math.pow(t-2.0,2)-24.0*(C-D)*(t-2.0)
                self.az = 96.0*(C-D)*(t-2.0)-24.0*(C-D)
            elif t < 3.0:
                self.z = 16.0*D*math.pow(t-2.5,3)-12.0*D*math.pow(t-2.5,2)+D
                self.vz = 48.0*D*math.pow(t-2.5,2)-24.0*D*(t-2.5)
                self.az = 96.0*D*(t-2.5)-24.0*D

            # Determine y
            L = -0.3
            R = 0.8

            if t < 0.5:
                self.y = -16.0*L*math.pow(t,3)+12.0*L*math.pow(t,2)
                self.vy = -48.0*L*math.pow(t,2)+23.0*L*t
                self.ay = -96.0*L*t+24.0*L
            elif t < 1.5:
                self.y = 2.0*(L-R)*math.pow(t-0.5,3)-3.0*(L-R)*math.pow(t-0.5,2)+L
                self.vy = 6.0*(L-R)*math.pow(t-0.5,2)-6.0*(L-R)*(t-0.5)
                self.ay = 12.0*(L-R)*(t-0.5)-6.0*(L-R)
            elif t < 2.5:
                self.y = 2.0*R*math.pow(t-1.5,3)-3.0*R*math.pow(t-1.5,2)+R
                self.vy = 6.0*R*math.pow(t-1.5,2)-6.0*R*(t-1.5)
                self.ay = 12.0*R*(t-1.5)-6.0*R
            else:
                self.y = 0.0
                self.vy = 0.0
                self.ay = 0.0

        # Determine state for 2/4 time
        if cond.data[0] == 2.0:

            # Determine z
            A = 0.4
            B = 1.0

            if t < 1.0:
                self.z = (A/2.0)*(1.0 - math.cos(2.0*math.pi*t))
                self.vz = A*math.pi*math.sin(2.0*math.pi*t)
                self.az = 2.0*A*math.pi*math.pi*math.cos(2.0*math.pi*t)
            else:
                self.z = (B/2.0)*(1.0 - math.cos(2.0*math.pi*t))
                self.vz = B*math.pi*math.sin(2.0*math.pi*t)
                self.az = 2.0*B*math.pi*math.pi*math.cos(2.0*math.pi*t)

            # Determine y
            R = 0.4

            if t < 1.5:
                self.y = (R/2.0)*(1.0 - math.cos((4.0/3.0)*math.pi*t))
                self.vy = (2.0/3.0)*R*math.pi*math.sin((4.0/3.0)*math.pi*t)
                self.ay = (8.0/9.0)*R*math.pi*math.pi*math.cos((4.0/3.0)*math.pi*t)
            else:
                self.y = 0.0
                self.vy = 0.0
                self.ay = 0.0

        # x is always zero
        self.x = 0.0
        self.vx = 0.0
        self.ax = 0.0

        # scale to make feasible
        self.x = self.x * scale
        self.vx = self.vx * scale
        self.ax = self.ax * scale
        self.y = self.y * scale
        self.vy = self.vy * scale
        self.ay = self.ay * scale
        self.z = self.z * scale + 1.0
        self.vz = self.vz * scale
        self.az = self.az * scale



# Setup the ROS node
if __name__=='__main__':

    # Initialize the ROS node
    rospy.init_node('conductor')

    # Create an instance of ViconCoordinates
    current_state = Conductor()

    # Do not exit until shutdown
    rospy.spin()