#!/usr/bin/env python

# Created by Tristan Laidlow on July 30, 2013
# Modified by Dorian Tsai on Dec 16, 2013 to incorporate VICON input, rather than tag input
# Modified by Kaizad Raimalwala on Dec 27, 2013, to remove subscription to tag_coordinates, add subscription to navdata, and modify VICON input

# This is an adapted version of Mike Hamer's keyboard_controller.py, found at:
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone for landing, take-off, etc.
# This controller suscribes to the 'navdata' and 'vicon' topics for information on its current position and; after computing outputs in sends the commands to the drone through BasicDroneController

# POSITIVE ROLL: when the drone banks right
# POSITIVE PITCH: when the nose points down
# POSITIVE YAW: when the drone twists left


################################################################################
# Chris's Stuff
# cmd_rate is a rosparam that defaults to 70 Hz
# Commands to be sent are updated when vicon position is recieved
# Commands are published to /cmd_vel at cmd_rate on an independant timer
# Requests are sent using a Timer on the topic /waypoint_request at cmd_rate

################################################################################
# TODO: get acceleration from vicon
################################################################################

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
import math
import numpy
from time import time
import tf # added

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_video_display import DroneVideoDisplay

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui

# Import message for receiving state information
from ardrone_tutorials.msg import StateData
from ardrone_autonomy.msg import Navdata 
from std_msgs.msg import Empty        

# For Vicon
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

# Some Constants
max_euler = rospy.get_param('euler_angle_max', 0.3) #rads
max_vz = rospy.get_param('control_vz_max', 700.0) / 1000.0 #m/s
max_yaw = rospy.get_param('control_yaw', 1.75) #rads/s

# Design Parameters
tau_x =  0.3 # 0.45# 0.2 # 0.7 # 1.6
tau_y =  0.3 # 0.45# 0.2 # 0.7 # 1.6
tau_z =  0.3 # 0.45# 0.3 # 0.7 # 0.8
tau_w = 1.5 # 2.2
zeta = 0.8 # 0.575 # 0.8
grav = 9.81
euler_limit = 0.21
vz_limit = 0.7
yaw_limit = 1.75

# numerical differentiation window
numberOfBufferPoints = 3

# logfile1= '/home/dsl1/viconlog1.txt'
# logfile2 = '/home/dsl1/viconlog2.txt'

# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
  ForwardLeft      = QtCore.Qt.Key.Key_W
  Forward          = QtCore.Qt.Key.Key_E
  ForwardRight     = QtCore.Qt.Key.Key_R
  Right            = QtCore.Qt.Key.Key_F
  BackwardRight    = QtCore.Qt.Key.Key_V
  Backward         = QtCore.Qt.Key.Key_C
  BackwardLeft     = QtCore.Qt.Key.Key_X
  Left             = QtCore.Qt.Key.Key_S
  YawLeft          = QtCore.Qt.Key.Key_A
  YawRight         = QtCore.Qt.Key.Key_G
  IncreaseAltitude = QtCore.Qt.Key.Key_Q
  DecreaseAltitude = QtCore.Qt.Key.Key_Z
  Takeoff          = QtCore.Qt.Key.Key_Y
  Land             = QtCore.Qt.Key.Key_H
  Emergency        = QtCore.Qt.Key.Key_Space
  StartHover       = QtCore.Qt.Key.Key_I
  EndHover         = QtCore.Qt.Key.Key_K
  NextWaypoint     = QtCore.Qt.Key.Key_N


# Our controller definition, note that we extend the DroneVideoDisplay class
class DroneController(DroneVideoDisplay):
  def __init__(self):
    super(DroneController,self).__init__()
    
    # current state values
    self.az = 0.0

    self.x_v = 0.0; # TODO added for vicon input, terrible sneaky method
    self.y_v = 0.0;
    self.z_v = 1.0; # z_v is set to one so if no vicon messages are recieved, the drones position matches desired position so it should not move
    self.t_v = 0.0;
    self.roll_v = 0.0;
    self.pitch_v = 0.0;
    self.yaw_v = 0.0;
    self.yaw_adjustment = 0.0

    # desired state values
    self.x_d = 0.0
    self.y_d = 0.0
    self.z_d = 1.0
    self.yaw_d = 0.0
    self.vx_d = 0.0
    self.vy_d = 0.0
                
    # buffer for differentiating positions to velocity
    self.bufferX=[]
    self.bufferY=[]
    self.bufferZ=[]
    self.buffert=[]
    self.vx_v=0.0
    self.vy_v=0.0
    self.vz_v=0.0

    # values to send to the drone
    self.pitch_out = 0.0
    self.roll_out = 0.0
    self.yaw_velocity_out = 0.0 
    self.z_velocity_out = 0.0

    # Holds the current drone status
    self.status = -1

    self.request_msg = Bool(True)
    self.command = Twist()

    # Used to calculate the last time state information was received
    self.lastSeen = time()

    self.sub_vicon = rospy.Subscriber('/vicon/ARDroneChris/ARDroneChris',TransformStamped,self.updateCurrentViconState)
    self.sub_current = rospy.Subscriber('/ardrone/navdata',Navdata,self.updateCurrentState)
    self.sub_path = rospy.Subscriber('/path_coordinates',StateData,self.updatePathState)
    
    # from BasicDroneController
    self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
    self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
    self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
    self.pubCommand = rospy.Publisher('/cmd_vel',Twist)
    
    self.pub_update_waypoint_request = rospy.Publisher('/next_waypoint',Bool)
                
    # request path waypoints at a certain rate
    cmd_rate = rospy.get_param('/cmd_rate',70); # set frequency to command_rate = cycling rate of controller [hz]
    COMMAND_PERIOD = 1.0/cmd_rate
    self.pub_request  = rospy.Publisher('/waypoint_request', Bool)

    # request waypoints
    self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD),self.requestWaypoints)
    # send commands to the drone
    self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD),self.SendCommand)

    # create a keyboard override
    self.keyboard_override = 0
    self.hover = 0

  # sets the current command. Used by both keyboard functions and controller
  def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
    self.command.linear.x  = pitch
    self.command.linear.y  = roll
    self.command.linear.z  = z_velocity
    self.command.angular.z = yaw_velocity

  # determine commands using the most recent data and send to the drone
  def SendCommand(self,event):
    if(self.keyboard_override == 0): # and (self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering):
      self.determineCommands() 
      self.pubCommand.publish(self.command)
    elif (self.keyboard_override == 1):
      self.pubCommand.publish(self.command)

  def SendEmergency(self):
    # Send an emergency (or reset) message to the ardrone driver
    self.pubReset.publish(Empty())

  # Send a takeoff message to the ardrone driver
  def SendTakeoff(self):
    self.pubTakeoff.publish(Empty())
  
  # Send a landing message to the ardrone driver
  def SendLand(self):
    self.pubLand.publish(Empty())

  # publish requests for commands
  def requestWaypoints(self,end):
    self.pub_request.publish(self.request_msg)

  def clamp(self, num, upper=1.0, lower=None):
    if (lower is None):
      num = max(min(num,upper),-1.0*upper)
      return (num)
    else:
      num = max(min(num,upper,lower))
      return (num)

  # This function gets state information for the drone from Vicon (position and rotation) and Navdata (velocity and acceleration)
  def updateCurrentViconState(self,viconState):
    self.t_v = (viconState.header.stamp.secs) + float(viconState.header.stamp.nsecs)/(10.0**(9))

    self.x_v = viconState.transform.translation.x
    self.y_v = viconState.transform.translation.y
    self.z_v = viconState.transform.translation.z
      
    qx_v = viconState.transform.rotation.x
    qy_v = viconState.transform.rotation.y
    qz_v = viconState.transform.rotation.z
    qw_v = viconState.transform.rotation.w

    quat = numpy.array([qx_v, qy_v, qz_v, qw_v])
    euler = tf.transformations.euler_from_quaternion(quat) 
    # print "(rpy) = (", euler[0]*180/math.pi, ', ', euler[1]*180/math.pi, ', ',  euler[2]*180/math.pi,')'

    self.roll_v  = euler[0]
    self.pitch_v = euler[1]
    self.yaw_v   = euler[2]

    self.addLatestCoordinates(self.x_v,self.y_v,self.z_v,self.t_v)
    self.calculateVelocity()
                
  # Add most recent vicon state info to buffer
  def addLatestCoordinates(self,x,y,z,t):
    # if buffer is full (limited by numberOfBufferPoints), remove first element
    if len(self.bufferX)==numberOfBufferPoints:
      self.bufferX.pop(0)  
      self.bufferY.pop(0)
      self.bufferZ.pop(0)
      self.buffert.pop(0)

    self.bufferX.append(x)
    self.bufferY.append(y)
    self.bufferZ.append(z)
    self.buffert.append(t)

  # calculate velocity
  def calculateVelocity(self):
    totalPoints=len(self.bufferX)-1
    if totalPoints==0:
      # assume initial velocity zero
      self.vx_v = 0
      self.vy_v = 0
      self.vz_v = 0
    else:
      # calculate itme step
      tDif = numpy.diff(numpy.asarray(self.buffert))
      tDifAvg = numpy.average(tDif)

      self.vx_v=sum(numpy.diff(self.bufferX))/(totalPoints*tDifAvg)
      self.vy_v=sum(numpy.diff(self.bufferY))/(totalPoints*tDifAvg)
      self.vz_v=sum(numpy.diff(self.bufferZ))/(totalPoints*tDifAvg)      
  # TODO: need to apply this to accelerations as well!

  # This method updates the current state by using vicon coordinates and subscribed navdata (vx, vy, az)
  def updateCurrentState(self,navdata):
    self.lastSeen = time()
    self.az = navdata.az * grav
    self.status = navdata.state

  # This method is called to prescribe a path to follow for ARDrone
  def updatePathState(self,pathState):
    self.x_d   = pathState.x
    self.y_d   = pathState.y
    self.z_d   = pathState.z
    self.yaw_d = pathState.yaw % (2.0*math.pi)
    self.vx_d  = pathState.vx
    self.vy_d  = pathState.vy


  # This method calculates the correct output values and sends them to the drone.
  def determineCommands(self):

    # Save and work with current variables so they are not over-written mid-calculation
    x_d = self.x_d
    y_d = self.y_d
    z_d = self.z_d
    yaw_d = self.yaw_d

    vx_d = self.vx_d
    vy_d = self.vy_d
    # no desired z velocity since that controller is just first order?
    
    x = self.x_v
    y = self.y_v
    z = self.z_v

    vx = self.vx_v
    vy = self.vy_v
    vz = self.vz_v

    az = self.az

    roll  = self.roll_v
    pitch = self.pitch_v
    yaw = self.yaw_v

    # determine commands in global coordinates
    self.z_velocity_out = (1.0 / tau_z) * (z_d - z)
    self.z_velocity_out = self.clamp(self.z_velocity_out) 

    # determine the mass-normalized thrust
    thrust = (grav + az) / (math.cos(roll) * math.cos(pitch))

    # calculate the acceleration in x (global coordinates)
    ax = (2.0 * zeta / tau_x) * (vx_d - vx) + (1.0 / (tau_x * tau_x)) * (x_d - x)

    # calculate the acceleration in y (global coordinates)
    ay = (2.0 * zeta / tau_y) * (vy_d - vy) + (1.0 / (tau_y * tau_y)) * (y_d - y)

    # make sure desired accelerations are reasonable
    ay_t_clamped = self.clamp(ay/thrust,1.0)
    ax_t_clamped = self.clamp(ax / (thrust * math.cos(self.roll_out)), 1.0)

    # NOTE: do we calculate roll_out after using it to calculate ax_t_clamped?

    # calculate the desired roll
    self.roll_out = self.clamp(math.asin(ay_t_clamped),euler_limit)

    # calculate the desired pitch using the desired roll
    self.pitch_out = self.clamp(math.asin(ax_t_clamped),euler_limit)

    # calculate the desired yaw velocity and convert to a percentage of maximum yaw rate

    # make sure yaw angles are a range that makes sense for proportional control
    # use the most recent yaw
    yaw = (yaw + 2.0*math.pi) if (yaw < 0.0) else yaw

    yaw_d = (yaw_d + 2.0*math.pi) if ( yaw_d < 0.0)            else yaw_d
    yaw_d = (yaw_d + 2.0*math.pi) if ((yaw - yaw_d) > math.pi) else yaw_d
    yaw_d = (yaw_d - 2.0*math.pi) if ((yaw_d - yaw) > math.pi) else yaw_d 

    self.yaw_velocity_out = self.clamp((1.0 / tau_w) * (yaw_d - yaw), yaw_limit)

    # transform roll, pitch, and yaw to local (drone) coordinates). Small roll and pitch, so z not effected significantly
    self.pitch_out_global = self.pitch_out*math.cos(yaw) + self.roll_out*math.sin(yaw)
    self.roll_out_global = (self.roll_out*math.cos(yaw) - self.pitch_out*math.sin(yaw))

    # print 'cmd(rp): ' , self.roll_out_global, ', ', self.pitch_out_global 
    # send the commands to the drone if the keyboard is not currently being used
    self.SetCommand(self.roll_out_global, self.pitch_out_global, self.yaw_velocity_out, self.z_velocity_out)


  # This method is called when a key is pressed. It overrides the automated commands.
  def requestNextWaypoint(self):
    bool_msg = Bool(True)
    self.pub_update_waypoint_request.publish(bool_msg)

  def keyPressEvent(self, event):
    key = event.key()

    # If we have constructed the drone controller and the key is not generated from an auto-repeating key
    if (not event.isAutoRepeat() ) :
      # Turn on override
      self.keyboard_override = 1 # turn on override
      self.roll_out = 0.0
      self.pitch_out = 0.0
      self.yaw_velocity_out = 0.0
      self.z_velocity_out = 0.0

      # Handle the important cases first!
      if key == KeyMapping.Emergency:
        self.SendEmergency()
      elif key == KeyMapping.Takeoff:
        self.SendTakeoff()
      elif key == KeyMapping.Land:
        self.SendLand()
      elif key == KeyMapping.StartHover:
        self.hover = 1
      elif key == KeyMapping.EndHover:
        self.hover = 0
      elif key == KeyMapping.NextWaypoint:
        self.requestNextWaypoint() 
      else:
        # Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
        if key == KeyMapping.YawLeft:
          self.yaw_velocity_out += 1
        elif key == KeyMapping.YawRight:
          self.yaw_velocity_out += -1

        elif key == KeyMapping.ForwardLeft:
          self.pitch_out += 1
          self.roll_out += 1
        elif key == KeyMapping.Forward:
          self.pitch_out += 1
        elif key == KeyMapping.ForwardRight:
          self.pitch_out += 1
          self.roll_out += -1
        elif key == KeyMapping.Right:
          self.roll_out += -1
        elif key == KeyMapping.BackwardRight:
          self.pitch_out += -1
          self.roll_out += -1
        elif key == KeyMapping.Backward:
          self.pitch_out += -1
        elif key == KeyMapping.BackwardLeft:
          self.pitch_out += -1
          self.roll_out += 1
        elif key == KeyMapping.Left:
          self.roll_out += 1

        elif key == KeyMapping.IncreaseAltitude:
          self.z_velocity_out += 1
        elif key == KeyMapping.DecreaseAltitude:
          self.z_velocity_out += -1

      # finally we set the command to be sent. The controller handles sending this at regular intervals
      self.SetCommand(self.roll_out, self.pitch_out, self.yaw_velocity_out, self.z_velocity_out)


  def keyReleaseEvent(self,event):
    key = event.key()

    # If we have constructed the drone controller and the key is not generated from an auto-repeating key
    if (not event.isAutoRepeat() ):
      # Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
      # Now we handle moving, notice that this section is the opposite (-=) of the keypress section
      self.keyboard_override = 0 # turn off override
      if key == KeyMapping.YawLeft:
        self.yaw_velocity_out -= 1
      elif key == KeyMapping.YawRight:
        self.yaw_velocity_out -= -1

      elif key == KeyMapping.ForwardLeft:
        self.pitch_out -= 1
        self.roll_out -= 1
      elif key == KeyMapping.Forward:
        self.pitch_out -= 1
      elif key == KeyMapping.ForwardRight:
        self.pitch_out -= 1
        self.roll_out -= -1
      elif key == KeyMapping.Right:
        self.roll_out -= -1
      elif key == KeyMapping.BackwardRight:
        self.pitch_out -= -1
        self.roll_out -= -1
      elif key == KeyMapping.Backward:
        self.pitch_out -= -1
      elif key == KeyMapping.BackwardLeft:
        self.pitch_out -= -1
        self.roll_out -= 1
      elif key == KeyMapping.Left:
        self.roll_out -= 1

      elif key == KeyMapping.IncreaseAltitude:
        self.z_velocity_out -= 1
      elif key == KeyMapping.DecreaseAltitude:
        self.z_velocity_out -= -1

      # finally we set the command to be sent. The controller handles sending this at regular intervals
      self.SetCommand(self.roll_out, self.pitch_out, self.yaw_velocity_out, self.z_velocity_out)


# Setup the application
if __name__=='__main__':
  import sys
  # Firstly we setup a ros node, so that we can communicate with the other packages
  rospy.init_node('drone_controller')
  
  # Now we construct our Qt Application and associated controllers and windows
  app = QtGui.QApplication(sys.argv)
  display = DroneController()

  display.show()

  # executes the QT application
  status = app.exec_()

  # and only progresses to here once the application has been shutdown
  rospy.signal_shutdown('Great Flying!')
  sys.exit(status)
