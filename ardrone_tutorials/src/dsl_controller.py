#!/usr/bin/env python

# PURPOSE
# This controller takes in current state information through the /current_coordinates topic and requests waypoint information from the /path_follower node for the desired state. Based on the difference between these two states, the controller computes outputs and sends commands to the drone.
# This controller also has a keypress handler to allow for manual control of the vehicle.

# SUBSCRIBED TOPICS
# /current_coordinates
# /path_coordinates
# /ardrone/navdata

# PUBLISHED TOPICS
# /cmd_vel
# /ardrone/land
# /ardrone/takeoff
# /ardrone/reset
# /waypoint_request

# NOTE
# This is an adapted version of Mike Hamer's keyboard_controller.py, found at:
# https://github.com/mikehamer/ardrone_tutorials

# VERSION HISTORY
# Jul 30, 2013 - initialy created (Tristan Laidlow)
# Dec 16, 2013 - modified to include VICON input (Dorian Tsai)
# Dec 27, 2013 - subscriptions changed, VICON input changed (Kaizad Raimalwala)
# ??? ??, 2014 - dependence on BasicDroneController removed (Chris McKinnon)
# Jun 05, 2014 - VICON removed as a separate node (Tristan Laidlow)

# SIGN CONVENTIONS
# Positive Roll: when the drone banks right
# Positive Pitch: when the nose points down
# Positive Yaw: when the drone twists left


####################
# IMPORT LIBRARIES #
####################

# Import ROS libraries, rospy, and load manifest file for access to project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import math for trigonometric functions
import math

# Load the DroneVideoDisplay class, which handles video display
from drone_video_display import DroneVideoDisplay
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui


###################
# IMPORT MESSAGES #
###################

from ardrone_tutorials.msg import StateData
from std_msgs.msg import Empty  
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

##################
# SET PARAMETERS #
##################

max_euler = rospy.get_param('euler_angle_max', 0.26) # rads
max_vz = rospy.get_param('control_vz_max', 700.0) / 1000.0 # m/s
max_yaw = rospy.get_param('control_yaw', 1.75) # rads/s
grav = 9.81 # m/s/s
cmd_rate = rospy.get_param('/cmd_rate', 70); # command rate (Hz)
COMMAND_PERIOD = 1.0/cmd_rate

# Design Parameters
tau_x =  0.6
tau_y =  0.6
tau_z =  0.6
tau_w = 1.5
zeta = 0.707


####################
# KEYPRESS HANDLER #
####################

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
  NextWaypoint     = QtCore.Qt.Key.Key_N # not currently implemented


###################
# MAIN CONTROLLER #
###################

class DroneController(DroneVideoDisplay):
  def __init__(self):
    super(DroneController,self).__init__()
    
    # current state values (default is to hover at 1.0m)
    self.x_cur = 0.0
    self.y_cur = 0.0
    self.z_cur = 1.0

    self.vx_cur = 0.0
    self.vy_cur = 0.0
    self.vz_cur = 0.0

    self.ax_cur = 0.0
    self.ay_cur = 0.0
    self.az_cur = 0.0

    self.roll_cur = 0.0
    self.pitch_cur = 0.0
    self.yaw_cur = 0.0

    # desired state values (default is to hover at 1.0m)
    self.x_des = 0.0
    self.y_des = 0.0
    self.z_des = 1.0

    self.vx_des = 0.0
    self.vy_des = 0.0
    self.vz_des = 0.0

    self.ax_des = 0.0
    self.ay_des = 0.0
    self.az_des = 0.0

    self.roll_des = 0.0
    self.pitch_des = 0.0
    self.yaw_des = 0.0

    # values to send to the drone
    self.roll_out = 0.0
    self.pitch_out = 0.0
    self.yaw_velocity_out = 0.0 
    self.z_velocity_out = 0.0

    # Holds the current drone status
    self.status = -1

    # Create a keyboard override, so the drone will stop acting autonomously
    self.keyboard_override = 0
    self.hover = 0

    # Whether or not to request a waypoint from the path node
    self.request_waypoint = Bool(True)

    # Command values to send to the drone
    self.command = Twist()

    # Subscribe to the desired_coordinates topic
    self.sub_des = rospy.Subscriber('/path_coordinates', StateData, self.updateDesiredState)

    # Subscribe to the current_coordinates topic
    self.sub_cur = rospy.Subscriber('/current_coordinates', StateData, self.updateCurrentState)

    # Subscribe to the navdata topic
    self.sub_navdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.updateNavdata)

    # Publish to topics that control the drone
    self.pubLand    = rospy.Publisher('/ardrone/land', Empty)
    self.pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    self.pubReset   = rospy.Publisher('/ardrone/reset', Empty)
    self.pubCommand = rospy.Publisher('/cmd_vel', Twist)
    
    # Requests path waypoint
    self.pub_request  = rospy.Publisher('/waypoint_request', Bool)

    # Establish a timer to send commands to the drone at a given frequency
    self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD), self.SendCommand)

    # Establish a timer to request waypoints at a given frequency
    self.waypointTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD), self.requestWaypoint)


  # Sets the current command (used by both keyboard functions and controller)
  # If hover is turned on, will only set the hover command
  def SetCommand(self,roll=0.0,pitch=0.0,yaw_velocity=0.0,z_velocity=0.0):
    if self.hover == 0:
      self.command.linear.x = pitch
      self.command.linear.y = roll
      self.command.linear.z = z_velocity
      self.command.angular.z = yaw_velocity
    else:
      self.command.linear.x = 0.0
      self.command.linear.y = 0.0
      self.command.linear.z = 0.0
      self.command.angular.z = 0.0

  # determine commands using the most recent data and send to the drone
  def SendCommand(self,event):
    if (self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering):
      self.pubCommand.publish(self.command)

  # Send an emergency (or reset) message to the ardrone driver
  def SendEmergency(self):
    self.pubReset.publish(Empty())

  # Send a takeoff message to the ardrone driver
  def SendTakeoff(self):
    self.pubTakeoff.publish(Empty())
  
  # Send a landing message to the ardrone driver
  def SendLand(self):
    self.pubLand.publish(Empty())

  # Publish requests for commands
  def requestWaypoint(self,event):
    self.pub_request.publish(self.request_waypoint)

  # Make sure value is within specified limits
  def clamp(self, num, upper=1.0, lower=None):
    if (lower is None):
      num = max(min(num,upper),-1.0*upper)
    else:
      num = max(min(num,upper),lower)
    return (num)


  # This method updates the drone status from navdata
  def updateNavdata(self,navdata):

    # Update the drone's status
    self.status = navdata.state

    # Obtain the vertical acceleration from navdata
    # TODO: this should be obtained from VICON
    self.az_cur = (navdata.az - 1.0) * grav


  # This method updates the current state of the drone
  def updateCurrentState(self,cur_data):

    # update the state information
    self.x_cur = cur_data.x
    self.y_cur = cur_data.y
    self.z_cur = cur_data.z

    self.vx_cur = cur_data.vx
    self.vy_cur = cur_data.vy
    self.vz_cur = cur_data.vz

    self.ax_cur = cur_data.ax
    self.ay_cur = cur_data.ay
    #self.az_cur = cur_data.az
    # TODO: get az from VICON, not Navdata
    # az may need some filter or more points to numerically differentiate over

    self.roll_cur = cur_data.roll
    self.pitch_cur = cur_data.pitch
    self.yaw_cur = cur_data.yaw


  # This method is called to prescribe a path to follow for ARDrone
  def updateDesiredState(self,desiredState):

    # Update the desired state information
    self.x_des = desiredState.x
    self.y_des = desiredState.y
    self.z_des = desiredState.z

    self.vx_des = desiredState.vx
    self.vy_des = desiredState.vy
    self.vz_des = desiredState.vz

    self.ax_des = desiredState.ax
    self.ay_des = desiredState.ay
    self.az_des = desiredState.az

    self.roll_des = desiredState.roll
    self.pitch_des = desiredState.pitch
    self.yaw_des = desiredState.yaw % (2.0*math.pi)

    # Determine the commands to be sent to the drone
    self.determineCommands()


  # This method calculates the correct output values and sends them to the drone.
  def determineCommands(self):

    # Save variables so they are not over-written mid-calculation
    x_des = self.x_des
    y_des = self.y_des
    z_des = self.z_des
    vx_des = self.vx_des
    vy_des = self.vy_des
    vz_des = self.vz_des
    ax_des = self.ax_des
    ay_des = self.ay_des
    az_des = self.az_des
    roll_des = self.roll_des
    pitch_des = self.pitch_des
    yaw_des = self.yaw_des
    
    x_cur = self.x_cur
    y_cur = self.y_cur
    z_cur = self.z_cur
    vx_cur = self.vx_cur
    vy_cur = self.vy_cur
    vz_cur = self.vz_cur
    ax_cur = self.ax_cur
    ay_cur = self.ay_cur
    az_cur = self.az_cur
    roll_cur  = self.roll_cur
    pitch_cur = self.pitch_cur
    yaw_cur = self.yaw_cur

    # determine commands in global coordinates
    self.z_velocity_out = (1.0 / tau_z) * (z_des - z_cur)
    self.z_velocity_out = self.clamp(self.z_velocity_out, max_vz) 

    # determine the mass-normalized thrust
    thrust = (grav + az_cur) / (math.cos(roll_cur) * math.cos(pitch_cur))

    # calculate the acceleration in x (global coordinates)
    ax = (2.0 * zeta / tau_x) * (vx_des - vx_cur) + (1.0 / (tau_x * tau_x)) * (x_des - x_cur)

    # calculate the acceleration in y (global coordinates)
    ay = (2.0 * zeta / tau_y) * (vy_des - vy_cur) + (1.0 / (tau_y * tau_y)) * (y_des - y_cur)

    # clamp ay for use in arcsin
    if thrust == 0.0:
        ay_clamped = 1.0
    else:
        ay_clamped = self.clamp(ay / thrust, 1.0)

    # calculate the desired roll
    roll_out_global = self.clamp(math.asin(ay_clamped), max_euler)

    # clamp ax for use in arcsin
    if thrust == 0.0:
        ax_clamped = 1.0
    else:
        ax_clamped = self.clamp(ax / (thrust * math.cos(roll_out_global)),1.0)

    # calculate the desired pitch using the desired roll
    pitch_out_global = self.clamp(math.asin(ax_clamped), max_euler)

    # make sure yaw angles are a range that makes sense for proportional control
    yaw_cur = (yaw_cur + 2.0*math.pi) if (yaw_cur < 0.0) else yaw_cur
    yaw_des = (yaw_des + 2.0*math.pi) if (yaw_des < 0.0) else yaw_des
    yaw_des = (yaw_des + 2.0*math.pi) if ((yaw_cur - yaw_des) > math.pi) else yaw_des
    yaw_des = (yaw_des - 2.0*math.pi) if ((yaw_des - yaw_cur) > math.pi) else yaw_des

    # calculate the desired yaw velocity and convert to a percentage of maximum yaw rate
    self.yaw_velocity_out = self.clamp((1.0 / tau_w) * (yaw_des - yaw_cur), max_yaw)

    # Transform roll, pitch, and yaw to local (drone) coordinates.
    # Assume z not affected significantly due to small roll and pitch angles.
    self.pitch_out = pitch_out_global * math.cos(yaw_cur) + roll_out_global * math.sin(yaw_cur)
    self.roll_out = roll_out_global * math.cos(yaw_cur) - pitch_out_global * math.sin(yaw_cur)

    # send the commands to the drone if the keyboard is not currently being used
    if(self.keyboard_override == 0):
      self.SetCommand(self.roll_out, self.pitch_out, self.yaw_velocity_out, self.z_velocity_out)
      
  # This method is called when a key is pressed. It overrides the automated commands.
  def keyPressEvent(self, event):
    key = event.key()

    # If the key is not generated from an auto-repeating key
    if (not event.isAutoRepeat()):
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
      #elif key == KeyMapping.NextWaypoint:
        #self.requestWaypoint() 
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

      # finally we set the command to be sent
      self.SetCommand(self.roll_out, self.pitch_out, self.yaw_velocity_out, self.z_velocity_out)


  def keyReleaseEvent(self,event):
    key = event.key()

    # If the key is not generated from an auto-repeating key
    if (not event.isAutoRepeat()):
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

      # finally we set the command to be sent.
      self.SetCommand(self.roll_out, self.pitch_out, self.yaw_velocity_out, self.z_velocity_out)


# Setup the application
if __name__=='__main__':
  import sys
  # Firstly we setup a ros node, so that we can communicate with the other packages
  rospy.init_node('drone_controller')
  
  # Now we construct our Qt Application and associated windows
  app = QtGui.QApplication(sys.argv)
  display = DroneController()

  display.show()

  # executes the QT application
  status = app.exec_()

  # and only progresses to here once the application has been shutdown
  rospy.signal_shutdown('Great Flying!')
  sys.exit(status)
