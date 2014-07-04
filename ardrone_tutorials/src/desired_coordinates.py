#!/usr/bin/env python

# Created by Tristan Laidlow on July 24, 2013

# This ROS node suscribes to data from the /ardrone/navdata topic, converts that information into a relative x-y-z coordinate relative to the tag, and publishes that data to the /ardrone/tag_coordinates topic.

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
from time import time
import math

# Import messages for sending out positional information
from ardrone_tutorials.msg import StateData

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# Constants
setup_time = 0.0 # number of seconds to wait before starting movement

# Our controller definition, note that we extend the DroneVideoDisplay class
class DesiredCoordinates(object):
    def __init__(self):
		super(DesiredCoordinates,self).__init__()
	
		# set the base time and setup flag
		self.start_time = time()
		self.finished_setup = 0

 		# values to publish
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.vz = 0.0
		self.ax = 0.0
		self.ay = 0.0
		self.az = 0.0


    # Call a function that will report a position as a function of time
    def updatePosition(self):

		self.keepConstant()
		#self.moveFlatCircle()
		#self.moveCircle()


    # Keep a constant position in space
    def keepConstant(self):

		# Keep a constant position
		self.x = 0.0
		self.y = 0.0
		self.z = 1.0


    # Move between two positions on the z-axis
    def moveZ(self):

		# Keep a constant position until setup is finished
		if(self.finished_setup == 0):
			self.x = 2.0
			self.y = 0.0
			self.z = 1.5
			if(time() - self.start_time >= setup_time):
				self.finished_setup = 1
				self.start_time = time()

		# Move back and forth between the two positions every 10s
		else:
			self.x = 2.0
			self.y = 0.0
			if(time() - self.start_time < 10):
				self.z = 0.5
			elif(time() - self.start_time < 20):
				self.z = 1.5
			else:
				self.start_time = time()


    # Move between two positions on the x_axis
    def moveX(self):

		# Keep a constant position until setup is finished
		if(self.finished_setup == 0):
			self.x = 2.0
			self.y = 0.0
			self.z = 1.0
			if(time() - self.start_time >= setup_time):
				self.finished_setup = 1
				self.start_time = time()

		# Move back and forth between the two positions every 10s
		else:
			self.y = 0.0
			self.z = 1.0
			if(time() - self.start_time < 10):
				self.x = 1.0
			elif(time() - self.start_time < 20):
				self.x = 2.0
			else:
				self.start_time = time()


    # Move between two positions on the y-axis
    def moveY(self):

		# Keep a constant position until setup is finished
		if(self.finished_setup == 0):
			self.x = 2.0
			self.y = 0.0
			self.z = 1.0
			if(time() - self.start_time >= setup_time):
				self.finished_setup = 1
				self.start_time = time()

		# Move back and forth between the two positions every 10s
		else:
			self.x = 2.0
			self.z = 1.0
			if(time() - self.start_time < 10):
				self.y = 0.5
			elif(time() - self.start_time < 20):
				self.y = -0.5
			else:
				self.start_time = time()


    # Move in a circle
    def moveCircle(self):

		# Define the circular constants
		radius = 0.25 # radius of the circle
		height = 1.765 # height of the centre of the circle
		t_max = 4.0 # number of seconds to complete one revolution
		distance = 2.5 # distance from the tag
		direction = -1 # +1 for counter-clockwise, -1 for clockwise

		# Keep a constant position until setup is finished
		if(self.finished_setup == 0):
			self.x = distance
			self.y = radius
			self.z = height
			if(time() - self.start_time >= setup_time):
				self.finished_setup = 1
				self.start_time = time()

		# Follow the ciruclar path
		else:
			theta = (time() - self.start_time) * direction * 2 * math.pi / t_max
			self.x = distance
			self.y = radius * math.cos(theta)
			self.z = radius * math.sin(theta) + height
			self.vx = 0.0
			self.vy = 2 * math.pi * radius * math.sin(theta) / t_max


    # Move in a flat circle
    def moveFlatCircle(self):

		# Define the cirle constants
		radius = 0.50 # radius of the circle
		height = 1.765 # height of the centre of the circle
		distance = 2.0 # distance to the centre of the circle from the tag
		t_max = 2.5 # number of seconds to complete the trajectory
		direction = -1 # +1 for counter-clockwise, -1 for clockwise

		# Keep at a constant position until setup is finished
		if(self.finished_setup == 0):
			self.x = distance
			self.y = radius
			self.z = height
			if(time() - self.start_time >= setup_time):
				self.finished_setup = 1
				self.start_time = time()

		# Follow the circular path
		else:
			theta = (time() - self.start_time) * direction * 2 * math.pi / t_max
			self.x = radius * math.sin(theta) + distance
			self.y = radius * math.cos(theta)
			self.z = height
			self.vx = 2 * math.pi * radius * math.cos(theta) / t_max
			self.vy = 2 * math.pi * radius * math.sin(theta) / t_max


    # Move in a flat circle
    def moveInclinedCircle(self):

		# Define the cirle constants
		radius = 0.50 # radius of the circle
		vertical_diff = 0.25 # how far out of the plane the drone moves up and down
		height = 1.765 # height of the centre of the circle
		distance = 2.5 # distance to the centre of the circle from the tag
		t_max = 15.0 # number of seconds to complete the trajectory
		direction = -1 # +1 for counter-clockwise, -1 for clockwise

		# Keep at a constant position until setup is finished
		if(self.finished_setup == 0):
			self.x = distance
			self.y = radius
			self.z = height
			if(time() - self.start_time >= setup_time):
				self.finished_setup = 1
				self.start_time = time()

		# Follow the circular path
		else:
			theta = (time() - self.start_time) * direction * 2 * math.pi / t_max
			self.x = radius * math.sin(theta) + distance
			self.y = radius * math.cos(theta)
			self.z = height + vertical_diff * math.sin(theta)
			self.vx = 2 * math.pi * radius * math.cos(theta) / t_max
			self.vy = 2 * math.pi * radius * math.sin(theta) / t_max


    # Move in a square
    def moveSquare(self):

		# Define the square constants
		side = 0.75 # length of one side in meters
		height = 1.765 # height of the centre of the square
		t_max = 30.0 # number of seconds to complete the path
		distance = 2.5 # distance from the tag in meters

		# Keep a constant position at top right corner until setup is finished
		if(self.finished_setup == 0):
			self.x = distance
			self.y = -(side / 2.0)
			self.z = height + (side / 2.0)
			if(time() - self.start_time >= setup_time):
				self.finished_setup = 1
				self.start_time = time()

		# Follow the path
		else:
			if(time() - self.start_time < t_max * 0.25):
				self.x = distance
				self.y = -(side / 2.0) + ((time() - self.start_time) / (t_max * 0.25)) * side
				self.z = height + (side / 2.0)
				self.vx = 0.0
				self.vy = side / (t_max * 0.25)
			elif(time() - self.start_time < t_max * 0.5):
				self.x = distance
				self.y = (side / 2.0)
				self.z = height + (side / 2.0) - ((time() - self.start_time - (t_max * 0.25)) / (t_max * 0.25)) * side
				self.vx = 0.0
				self.vy = 0.0
			elif(time() - self.start_time < t_max * 0.75):
				self.x = distance
				self.y = (side / 2.0) - ((time() - self.start_time - (t_max * 0.5)) / (t_max * 0.25)) * side
				self.z = height - (side / 2.0)
				self.vx = 0.0
				self.vy = -side / (t_max * 0.25)
			elif(time() - self.start_time < t_max):
				self.x = distance
				self.y = -(side / 2.0)
				self.z = height - (side / 2.0) + ((time() - self.start_time - (t_max * 0.75)) / (t_max * 0.25)) * side
				self.vx = 0.0
				self.vy = 0.0
			else:
				self.start_time = time()


# Publishes the coordinates at a frequency of 100Hz
def publish_coordinates():

    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('ardrone_desired_coordinates')

    # Initialize a class to hold the coordinate information
    coordinates = DesiredCoordinates()

    # Publish the coordinates to the desired_coordinates topic
    pub = rospy.Publisher('path_coordinates',StateData)
    pub_data = StateData()

    # Continue to publish until shutdown
    while not rospy.is_shutdown():
	
	# Get current values
	coordinates.updatePosition()

	# Set values for publishing
	#pub_data.header.stamp = rospy.get_rostime()
	pub_data.x = coordinates.x
	pub_data.y = coordinates.y
	pub_data.z = coordinates.z
	pub_data.roll = coordinates.roll
	pub_data.pitch = coordinates.pitch
	pub_data.yaw = coordinates.yaw
	pub_data.vx = coordinates.vx
	pub_data.vy = coordinates.vy
	pub_data.vz = coordinates.vz
	pub_data.ax = coordinates.ax
	pub_data.ay = coordinates.ay
	pub_data.az = coordinates.az

	# Publish the data
	pub.publish(pub_data)

	# Pause
	rospy.sleep(1.0/70.0) # publish at 70Hz


if __name__=='__main__':
    try:
        publish_coordinates()
    except rospy.ROSInterruptException:
        pass
