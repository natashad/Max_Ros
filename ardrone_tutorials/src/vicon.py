#!/usr/bin/env python

# PURPOSE
# This ROS node takes in information from VICON, transforms it, and sends it to the drone over the /current_coordinates topic.

# SUBSCRIBED TOPICS
# /vicon/ARDroneChris/ARDroneChris

# PUBLISHED TOPICS
# /current_coordinates

# VERSION HISTORY
# Jun 09, 2014 - initialy created (Tristan Laidlow)

# NOTES
# This is based on the work initially done by Chris McKinnon.


####################
# IMPORT LIBRARIES #
####################

# Import ROS libraries, rospy, and load manifest file for access to project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

import math             # for trigonometric functions
import numpy            # to handle arrays for buffering
import tf               # to convert between quaternion and eulerian


###################
# IMPORT MESSAGES #
###################

# TransformStamped is used to receive VICON data
from geometry_msgs.msg import TransformStamped

# StateData is used to send current coordinates to the drone
from ardrone_tutorials.msg import StateData


##################
# SET PARAMETERS #
##################

# Set the number of points to use for numerical differentiation
numberOfBufferPoints = 3

# Choose the VICON model to be used
model = '/vicon/ARDroneShare/ARDroneShare'


#########################
# MAIN CLASS DEFINITION #
#########################

class ViconCoordinates(object):

    def __init__(self):
        super(ViconCoordinates,self).__init__()

        # Initialize the state variables (default is to hover at 1.0m)
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

        # Initialize time variables for use in numerical differentiation
        self.time = 0.0
        self.v_time = 0.0

        # Initialize buffers for calculating velocities and accelerations
        self.vx_buffer = []
        self.vy_buffer = []
        self.vz_buffer = []
        self.v_time_buffer = []

        self.ax_buffer = []
        self.ay_buffer = []
        self.az_buffer = []
        self.a_time_buffer = []

        # Subscribe to the VICON information
        self.sub_vicon = rospy.Subscriber(model,TransformStamped,self.update)

        # Publish to the /current_coordinates topic
        self.pub_state = rospy.Publisher('/current_coordinates',StateData)

    # Calculate the current state based on the VICON information
    def update(self,vicon):

        # Record the time at which the state was determined
        self.time = (vicon.header.stamp.secs)
        self.time = self.time + float(vicon.header.stamp.nsecs)/(10.0**(9))

        # Get the translational position directly from VICON
        self.x = vicon.transform.translation.x
        self.y = vicon.transform.translation.y
        self.z = vicon.transform.translation.z

        # Get the rotational position in the form of a quaternion
        quat_x = vicon.transform.rotation.x
        quat_y = vicon.transform.rotation.y
        quat_z = vicon.transform.rotation.z
        quat_w = vicon.transform.rotation.w
        quat = numpy.array([quat_x, quat_y, quat_z, quat_w])

        # Convert the quaternion in eulerian coordinates
        euler = tf.transformations.euler_from_quaternion(quat)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

        # Calculate the velocities
        self.addCoordinatesToVelocityBuffer()
        self.calculateVelocity()

        # Calculate the accelerations
        self.addCoordinatesToAccelerationBuffer()
        self.calculateAcceleration()

        # Publish the current state data
        self.publishVicon()


    # Add the current translational values to the velocity buffer
    def addCoordinatesToVelocityBuffer(self):
            
        # If the buffer is full, remove the first element
        if len(self.vx_buffer) == numberOfBufferPoints:
            self.vx_buffer.pop(0)
            self.vy_buffer.pop(0)
            self.vz_buffer.pop(0)
            self.v_time_buffer.pop(0)

        self.vx_buffer.append(self.x)
        self.vy_buffer.append(self.y)
        self.vz_buffer.append(self.z)
        self.v_time_buffer.append(self.time)


    # Determine the velocity through numerical differentiation
    def calculateVelocity(self):

        # Determine the number of points in the buffer
        num_pts = len(self.vx_buffer)

        # If there is not at least two points in the buffer, assume an
        # initial velocity of zero.
        if num_pts < 2:
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0

        # Otherwise, find the average speed over the points in the buffer
        else:
            time_diff = numpy.diff(numpy.asarray(self.v_time_buffer))
            avg_time_diff = numpy.average(time_diff)

            self.vx = sum(numpy.diff(self.vx_buffer))
            self.vx = self.vx / ((num_pts - 1) * avg_time_diff)

            self.vy = sum(numpy.diff(self.vy_buffer))
            self.vy = self.vy / ((num_pts - 1) * avg_time_diff)

            self.vz = sum(numpy.diff(self.vz_buffer))
            self.vz = self.vz / ((num_pts - 1) * avg_time_diff)

            # calculate the average time to use for determining acceleration
            self.v_time = numpy.average(numpy.asarray(self.v_time_buffer))


    # Add the current velocity values to the acceleration buffer
    def addCoordinatesToAccelerationBuffer(self):

        # If the buffer is full, remove the first element
        if len(self.ax_buffer) == numberOfBufferPoints:
            self.ax_buffer.pop(0)
            self.ay_buffer.pop(0)
            self.az_buffer.pop(0)
            self.a_time_buffer.pop(0)

        self.ax_buffer.append(self.vx)
        self.ay_buffer.append(self.vy)
        self.az_buffer.append(self.vz)
        self.a_time_buffer.append(self.v_time)


    # Determine the acceleration through numerical differentiation
    def calculateAcceleration(self):

        # Determine the number of points in the buffer
        num_pts = len(self.ax_buffer)

        # If there is not at least two points in the buffer, assume an
        # initial acceleration of zero.
        if num_pts < 2:
            self.ax = 0.0
            self.ay = 0.0
            self.az = 0.0

        # Otherwise, find the average acceleration over the points
        else:
            time_diff = numpy.diff(numpy.asarray(self.a_time_buffer))
            avg_time_diff = numpy.average(time_diff)

            self.ax = sum(numpy.diff(self.ax_buffer))
            self.ax = self.ax / ((num_pts - 1) * avg_time_diff)

            self.ay = sum(numpy.diff(self.ay_buffer))
            self.ay = self.ay / ((num_pts - 1) * avg_time_diff)

            self.az = sum(numpy.diff(self.az_buffer))
            self.az = self.az / ((num_pts - 1) * avg_time_diff)


    # Publish the current state information
    def publishVicon(self):

        # Get the most recent VICON data
        vicon_data = StateData()

        vicon_data.x = self.x
        vicon_data.y = self.y
        vicon_data.z = self.z

        vicon_data.vx = self.vx
        vicon_data.vy = self.vy
        vicon_data.vz = self.vz

        vicon_data.ax = self.ax
        vicon_data.ay = self.ay
        vicon_data.az = self.az

        vicon_data.roll = self.roll
        vicon_data.pitch = self.pitch
        vicon_data.yaw = self.yaw

        # Publish the most recent data
        self.pub_state.publish(vicon_data)


# Setup the ROS node
if __name__=='__main__':

    # Initialize the ROS node
    rospy.init_node('vicon_coordinates')

    # Create an instance of ViconCoordinates
    current_state = ViconCoordinates()

    # Do not exit until shutdown
    rospy.spin()
