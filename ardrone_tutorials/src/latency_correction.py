#!/usr/bin/env python

# PURPOSE
# This ROS node receives state information about the vehicle from VICON and
# corrects for the latency in the system by projecting forward the state by
# the amount of latency in the system. This allows the controller to determine
# commands for this projected state, so that by the time the vehicle actually
# receives and acts on the commands, the vehicle should (in reality) be closer
# to the projected state than the one reported by VICON.

# SUBSCRIBED TOPICS
# /current_coordinates_uncorrected
# /cmd_vel

# PUBLISHED TOPICS
# /current_coordinates

# VERSION HISTORY
# July 30, 2014 - initially created (Tristan Laidlow)


####################
# IMPORT LIBRARIES #
####################

# Import ROS libraries, rospy, and load manifest file for access to project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import math


###################
# IMPORT MESSAGES #
###################

from ardrone_tutorials.msg import StateData
from geometry_msgs.msg import Twist


##################
# SET PARAMETERS #
##################

# Delay in the system (determined by measure_latency.py)
LATENCY = 0.06 #0.024 # in seconds

# Design Parameters (should be the same as in dsl_controller_parrot.py)
tau_x =  0.6
tau_y =  0.6
tau_z =  0.6
tau_w = 1.5

grav = 9.81


#######################
# CORRECT FOR LATENCY #
#######################

class CorrectForLatency(object):

    def __init__(self):
        super(CorrectForLatency,self).__init__()

        # Initialize the command queue and state variable
        self.cmd_q = []
        self.cmd_q_time = []
        self.cur_state = StateData()
        self.cur_time = 0.0

        # Subscribe to the VICON data
        self.sub_vicon = rospy.Subscriber('/current_coordinates_uncorrected', StateData, self.estimate)

        # Subscribe to /cmd_vel in order to buffer the command queue
        self.sub_cmd = rospy.Subscriber('/cmd_vel', Twist, self.update_cmd_q)

        # Publish so the controller can receive the corrected state
        self.pub_state = rospy.Publisher('/current_coordinates', StateData)

    def estimate(self, uncorrected):

        if len(self.cmd_q_time) > 0:

            # Store the old state information
            old_state = uncorrected

            # Clean the command queue
            self.clean_cmd_q()

            # Get the current time
            now = rospy.get_rostime()
            self.cur_time = now.secs + (now.nsecs / (10.0**9.0))

            # Move up to the first command in the command queue
            time_to_first_cmd = self.cmd_q_time[0] - self.cur_time + LATENCY
            if (time_to_first_cmd > 0.0):

                # Estimate thrust
                thrust = (old_state.az + grav) / (math.cos(old_state.roll)*math.cos(old_state.pitch))

                # Estimate velocities (assumes yaw held close to zero)
                self.cur_state.vx = old_state.vx + (time_to_first_cmd * thrust * math.sin(old_state.pitch) * math.cos(old_state.roll))
                self.cur_state.vy = old_state.vy - (time_to_first_cmd * thrust * math.sin(old_state.roll))
                self.cur_state.vz = old_state.vz + (time_to_first_cmd * thrust * math.cos(old_state.pitch) * math.cos(old_state.roll))

                # Estimate position
                self.cur_state.x = old_state.x + (time_to_first_cmd * old_state.vx)
                self.cur_state.y = old_state.y + (time_to_first_cmd * old_state.vy)
                self.cur_state.z = old_state.z + (time_to_first_cmd * old_state.vz)

                # Estimate vertical acceleration
                self.cur_state.az = (self.cur_state.vz - old_state.vz) / time_to_first_cmd

                # Keep roll, pitch and yaw constant
                self.cur_state.roll = old_state.roll
                self.cur_state.pitch = old_state.pitch
                self.cur_state.yaw = old_state.yaw


            # For each command in the command queue
            for i in range(0, len(self.cmd_q)):

                # Store the previous state
                old_state = self.cur_state

                # Determine the amount of time before the next command
                if i == len(self.cmd_q)-1:
                    dt = self.cur_time - self.cmd_q_time[i]
                else:
                    dt = self.cmd_q_time[i+1] - self.cmd_q_time[i]

                # Estimate the thrust
                thrust = (old_state.az + grav) / (math.cos(old_state.roll)*math.cos(old_state.pitch))

                # Estimate the z velocity
                C_z = math.exp(-dt * tau_z)
                self.cur_state.vz = (1.0 - C_z)*self.cmd_q[i].linear.z + C_z*old_state.vz

                # Estimate the x and y velocities
                self.cur_state.vx = old_state.vx + (dt * thrust * math.sin(old_state.pitch) * math.cos(old_state.roll))
                self.cur_state.vy = old_state.vy - (dt * thrust * math.sin(old_state.roll))

                # Estimate the current position
                self.cur_state.x = old_state.x + (dt * old_state.vx)
                self.cur_state.y = old_state.y + (dt * old_state.vy)
                self.cur_state.z = old_state.z + (dt * old_state.vz)

                # Estimate the z acceleration
                self.cur_state.az = (self.cur_state.vz - old_state.vz) / dt

                # Estimate the roll
                C_r = math.exp(-dt * tau_y)
                self.cur_state.roll = (1.0 - C_r)*self.cmd_q[i].linear.y + C_r*old_state.roll

                # Estimate the pitch
                C_p = math.exp(-dt * tau_x)
                self.cur_state.pitch = (1.0 - C_p)*self.cmd_q[i].linear.x + C_p*old_state.pitch

                # Estimate the yaw
                C_w = math.exp(-dt * tau_w)
                yaw_velocity = (1.0 - C_w)*self.cmd_q[i].angular.z + C_w*old_state.yaw
                self.cur_state.yaw = old_state.yaw + dt*yaw_velocity

        # Publish the corrected state
        self.pub_state.publish(self.cur_state)

    def update_cmd_q(self, cmd):

        # Update the command queue
        now = rospy.get_rostime()
        self.cmd_q_time.append(now.secs + (now.nsecs / (10.0**9.0)))
        self.cmd_q.append(cmd)

    def clean_cmd_q(self):

        if len(self.cmd_q_time) > 0:

            # Remove commands from the buffer if further back than the latency time
            now = rospy.get_rostime()
            self.cur_time = now.secs + (now.nsecs / (10.0**9.0))
            while (len(self.cmd_q) > 0) and ((self.cur_time - self.cmd_q_time[0]) > LATENCY):
                self.cmd_q.pop(0)
                self.cmd_q_time.pop(0)

        
# Setup the ROS node
if __name__=='__main__':

    # Initialize the ROS node
    rospy.init_node('latency_correction')

    # Create an instance of CorrectForLatency
    correction = CorrectForLatency()

    # Do not exit until shutdown
    rospy.spin()