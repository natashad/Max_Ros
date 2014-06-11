#!/usr/bin/env python

# PURPOSE
# This ROS node takes in information from the WaveDNA software and converts it into trajectories for the drone to follow.

# SUBSCRIBED TOPICS
# /terpsichore/ableton_time
# /terpsichore/music_data
# /waypoint_request

# PUBLISHED TOPICS
# /desired_coordinates

# VERSION HISTORY
# Jun 09, 2014 - initialy created (Tristan Laidlow)

# NOTES
# This is based on the work initially done by Federico Augugliaro.


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

# bardata is used to communicate with the WaveDNA software
from terpischore.msg import bardata

# StateData is used to send desired coordinates to the drone
from ardrone_tutorials.msg import StateData

# Bool is used for waypoint requests
from std_msgs.msg import Bool


##################
# SET PARAMETERS #
##################

k = 0.5
scale = 1.0


#########################
# MAIN CLASS DEFINITION #
#########################

class CreateTrajectory(object):
    def __init__(self):
        super(CreateTrajectory,self).__init__()

        # Initialize state variables for the drone
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Initialize buffers for storing WaveDNA data
        self.time_buffer = []
        self.point_buffer = []

        # Manage time for synchronization
        #self.last_time_check = 0.0
        self.cur_time = 0.0

        # Subscribe to the waypoint_request topic
        self.sub_request = rospy.Subscriber('/waypoint_request', Bool, self.process_request)

        # Subscribe to the WaveDNA data
        self.sub_wavedna = rospy.Subscriber('/terpsichore/music_data', bardata, self.buffer_data)
        self.sub_time = rospy.Subscriber('/terpsichore/ableton_time', float64, self.update_time)

        # Publish to the desired_coordinates topic
        self.pub_desired = rospy.Publisher('/desired_coordinates', StateData)

    # Add new WaveDNA data to the buffers
    def buffer_data(self, new_wavedna):

        # Update the time for synchronization
        #self.last_time_check = new_wavedna.time
        #self.cur_time = time()

        # Remove any unnecessary data points (those already used)
        self.clean_buffers()

        # Add new data to the buffers
        self.time_buffer.append(new_wavedna.events(0).t)
        self.point_buffer.append(new_wavedna.events(0).data(0))
    
    # Keep time in sync with WaveDNA
    def update_time(self, new_time):
        self.cur_time = new_time

    # Remove any unnecessary points from buffers
    def clean_buffers(self):

        # Estimate the current time
        #est_time = time() - self.cur_time + self.last_time_check

        # If the current time is past the 2nd item in the buffer, get rid
        # of the first. Repeat until time is between first and second items.
        #while (len(self.time_buffer) > 1) and (est_time > self.time_buffer(1)):
        while (len(self.time_buffer) > 1) and (self.cur_time > self.time_buffer(1)):
            self.time_buffer.pop(0)
            self.point_buffer.pop(0)

    # Process a request for a waypoint
    def process_request(self):

        # Remove any unnecessary data points
        self.clean_buffers()

        # If buffers are empty, go to the default position
        if len(self.time_buffer) == 0:
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

        # Else if buffers only have one data point, hold steady
        elif len(self.time_buffer) == 1:
            self.x = 0.0
            self.y = 0.0
            self.z = 1.0 + self.point_buffer(0)*scale
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0
            self.ax = 0.0
            self.ay = 0.0
            self.az = 0.0
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0

        # Else create the trajectory to move to the next point
        else:
            total_time = self.time_buffer(1) - self.time_buffer(0)
            step_time = 0.75 * total_time * (1.0 - 2.0*math.fabs(0.5 - k))
            p_initial = 1.0 + self.point_buffer(0)*scale
            p_final = 1.0 + self.point_buffer(1)*scale
            v_initial = 0.0
            v_final = 0.0
            est_time = time() - self.cur_time + self.last_time_check
            calc_time = est_time - self.time_buffer(0)
            self.determine_state(total_time, step_time, p_initial, p_final, v_initial, v_final, calc_time)

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
    def determine_state(self, total_time, step_time, p0, pT, v0, vT, time):

        # Calculate time parameters
        t1 = k*total_time - 0.5*step_time
        t2 = k*total_time + 0.5*step_time
        w = math.pi / step_time

        # Calculate intermediate variables
        A = (2.0 * math.pow(w,2.0) * (total_time*(v0 + vT) + 2.0*(p0 - pT))) / (-8.0 + math.pow(math.pi,2.0) + 4.0*(-1.0 + k)*k*math.pow(total_time,2.0)*math.pow(w,2.0))
        B = (A*total_time - 2.0*A*k*total_time - v0 + vT) / total_time
        c1 = v0
        c2 = A*k*total_time + v0 - ((A*math.pi) / (2.0*w))
        c3 = A*total_time - B*total_time + vT
        c4 = p0
        c5 = (-A*(-8.0 + math.pow(math.pi - 2.0*k*total_time*w,2.0)) + 8.0*math.pow(w,2.0)*p0) / (8.0*math.pow(w,2.0))
        c6 = 0.5*(-A + B)*math.pow(total_time,2.0) - total_time*vT + pT

        # Calculate the desired state
        if time < t1:
            self.z = 0.5*(A + B)*math.pow(time,2.0) + c1*time + c4
            self.vz = (A + B)*time + c1
            self.az = A + B
        elif time < t2:
            self.z = (-A / math.pow(w,2.0))*math.cos(w*(time - t1)) + 0.5*B*math.pow(time,2.0) + c2*time + c5
            self.vz = (A / w)*math.sin(w*(time - t1)) + B*time + c2
            self.az = A*math.cos(w*(time - t1)) + B
        else:
            self.z = 0.5*(-A + B)*math.pow(time,2.0) + c3*time + c6
            self.vz = (-A + B)*time + c3
            self.az = -A + B


# Setup the ROS node
if __name__=='__main__':

    # Initialize the ROS node
    rospy.init_node('wavedna')

    # Create an instance of ViconCoordinates
    current_state = CreateTrajectory()

    # Do not exit until shutdown
    rospy.spin()


