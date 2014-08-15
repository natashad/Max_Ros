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
# Jul 31, 2014 - initialy created (Tristan Laidlow)


####################
# IMPORT LIBRARIES #
####################

# Import ROS libraries, rospy, and load manifest file for access to project dependencies
import roslib; roslib.load_manifest('dsl__wavedna')
import rospy

# Import math for trigonometric functions
import math

# Import time for synchronization
from time import time


###################
# IMPORT MESSAGES #
###################

# StateData is used to send desired coordinates to the drone
from dsl__msg.msg import StateData

# Bool is used for waypoint requests and Float64 for time
from std_msgs.msg import Bool
from std_msgs.msg import Float64

# Pair is used to send time and signature from WaveDNA
from dsl__msg.msg import pair


##################
# SET PARAMETERS #
##################

scale = 1.0
base_height = 1.0


#########################
# MAIN CLASS DEFINITION #
#########################

class Conductor(object):
    def __init__(self):
        super(Conductor,self).__init__()

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

        # Create lookup for spline coefficients: [beginning of spline, t^3, t^2, t^1, t^0]
        # Coefficients for 4/4 Time Vertical Position
        coeffv44 = []
        coeffv44.append([-0.1, -19.8520787581, 11.6459319338, -1.5351032364, 0.05])
        coeffv44.append([0.1, -1.0528758126, -0.265315321, 0.7410200861, 0.05])
        coeffv44.append([0.4, -1.3743428403, -1.2129035524, 0.2975544241, 0.22])
        coeffv44.append([0.6, 4.4123805333, -2.0375092565, -0.3525281377, 0.22])
        coeffv44.append([0.9, -0.0758924259, 1.9336332234, -0.3836909476, 0.05])
        coeffv44.append([1.1, -4.2268657144, 1.8880977678, 0.3806552506, 0.05])
        coeffv44.append([1.4, 0.2739026645, -1.9160813751, 0.3722601684, 0.22])
        coeffv44.append([1.6, 3.5573258678, -1.7517397764, -0.3613040619, 0.22])
        coeffv44.append([1.9, 4.0474810614, 1.4498535046, -0.4518699434, 0.05])
        coeffv44.append([2.1, -8.6363536475, 3.8783421415, 0.6137691858, 0.05])
        coeffv44.append([2.4, 4.247906059, -3.8943761413, 0.6089589859, 0.35])
        coeffv44.append([2.6, 1.9562869848, -1.3456325059, -0.4390427435, 0.35])
        coeffv44.append([2.9, 15.8804901271, 0.4150257804, -0.7182247612, 0.15])
        coeffv44.append([3.1, -20.4048924807, 9.9433198567, 1.3534443663, 0.15])
        coeffv44.append([3.4, -3.1474658825, -8.4210833759, 1.8101153105, 0.9])
        coeffv44.append([3.6, 24.3949942674, -10.3095629054, -1.9360139458, 0.9])
        coeffv44.append([3.9, -19.8520787561, 11.6459319353, -1.5351032368, 0.05])

        # Coefficients for 3/4 Time Vertical Position
        coeffv34 = []
        coeffv34.append([-0.1, -19.8327350366, 11.6438333433, -1.5354572672, 0.05])
        coeffv34.append([0.1, -1.0970988382, -0.2558076787, 0.7421478657, 0.05])
        coeffv34.append([0.4, -1.0951811386, -1.2431966331, 0.2924465722, 0.22])
        coeffv34.append([0.6, 3.7742082881, -1.9003053163, -0.3362538177, 0.22])
        coeffv34.append([0.9, 3.952608527, 1.496482143, -0.4574007697, 0.05])
        coeffv34.append([1.1, -8.6213254282, 3.8680472592, 0.6155051108, 0.05])
        coeffv34.append([1.4, 4.2413381131, -3.8911456261, 0.6085756007, 0.35])
        coeffv34.append([1.6, 1.9573137444, -1.3463427583, -0.4389220762, 0.35])
        coeffv34.append([1.9, 15.8801274457, 0.4152396117, -0.7182530202, 0.15])
        coeffv34.append([2.1, -20.4050326858, 9.9433160791, 1.353458118, 0.15])
        coeffv34.append([2.4, -3.1461568165, -8.4212133381, 1.8100889403, 0.9])
        coeffv34.append([2.6, 24.3919345336, -10.308907428, -1.9359352129, 0.9])
        coeffv34.append([2.9, -19.8327346184, 11.6438336522, -1.5354573457, 0.05])

        # Coefficients for 2/4 Time Vertical Position
        coeffv24 = []
        coeffv24.append([-0.1, -12.4137419581, 9.8007411485, -1.5635985514, 0.07])
        coeffv24.append([0.1, -6.3644185016, 2.3524959736, 0.8670488731, 0.05])
        coeffv24.append([0.4, 2.8735668441, -3.3754806778, 0.5601534618, 0.35])
        coeffv24.append([0.6, 3.0438476975, -1.6513405714, -0.445210788, 0.35])
        coeffv24.append([0.9, 12.4137945309, 1.0881223564, -0.6141762525, 0.15])
        coeffv24.append([1.1, -17.8331232175, 8.5363990749, 1.3107280338, 0.17])
        coeffv24.append([1.4, -2.8735461464, -7.5134118208, 1.61762421, 0.85])
        coeffv24.append([1.6, 21.1536434271, -9.2375395086, -1.7325660559, 0.85])
        coeffv24.append([1.9, -12.4135469867, 9.8007395758, -1.5636060357, 0.07])

        coeffv = [coeffv24, coeffv34, coeffv44]

        # Coefficients for 4/4 Time Horizontal Position
        coeffh44 = []
        coeffh44.append([0.0, 0.351381903892195, -0.987851793292893, -0.163530110599302, 0.0])
        coeffh44.append([1.0, 4.20776410526663, 0.0662939183836926, -1.0850879855085, -0.8])
        coeffh44.append([1.5, -8.50399619986791, 6.37794007628363, 2.13702901182516, -0.8])
        coeffh44.append([2.0, 4.208220694205, -6.37805422351823, 2.13697193820786, 0.8])
        coeffh44.append([2.5, 0.350639946867334, -0.0657231822107219, -1.08491676465661, 0.8])
        coeffh44.append([3.5, -1.31462016287835, 0.986196658391281, -0.164443288476054, 0.0])

        # Coefficients for 3/4 Time Horizontal Position
        coeffh34 = []
        coeffh34.append([0.0, 3.100617477, -1.181964222, -0.1841722583, 0.0])
        coeffh34.append([0.5, -4.3752304972, 3.4689619936, 0.9593266275, 0.0])
        coeffh34.append([1.0, 1.6003045119, -3.0938837523, 1.1468657482, 0.8])
        coeffh34.append([1.5, 0.6402166047, -0.6934269845, -0.7467896202, 0.8])
        coeffh34.append([2.5, -1.6024705588, 1.2272228296, -0.2129937751, 0.0])

        # Coefficients for 2/4 Time Horizontal Position
        coeffh24 = []
        coeffh24.append([0.0, -5.5070729496, 3.4935119065, 0.4837269093, 0.0])
        coeffh24.append([0.4, 3.4463426974, -3.1149756331, 0.6351414187, 0.4])
        coeffh24.append([0.6, 0.8172382052, -1.0471700146, -0.1972877108, 0.43])
        coeffh24.append([1.5, -3.0966141816, 1.1593731394, -0.0963048985, 0.0])
        coeffh24.append([1.75, 6.2081, -1.1630874968, -0.0972334878, 0.0])

        coeffh = [coeffh24, coeffh34, coeffh44]

        self.coeff = [coeffv, coeffh]

        # Subscribe to the waypoint_request topic
        self.sub_request = rospy.Subscriber('/waypoint_request', Bool, self.process_request)

        # Subscribe to the WaveDNA data
        self.sub_wavedna = rospy.Subscriber('/conductor_time', pair, self.determine_state)

        # Publish to the desired_coordinates topic
        self.pub_desired = rospy.Publisher('/path_coordinates', StateData)

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
        sig = int(cond.data[0])
        t_range = self.determine_range(sig, t)

        self.z = (self.coeff[0][sig-2][t_range[0]][1] * ((t - self.coeff[0][sig-2][t_range[0]][0])**3) + 
                  self.coeff[0][sig-2][t_range[0]][2] * ((t - self.coeff[0][sig-2][t_range[0]][0])**2) + 
                  self.coeff[0][sig-2][t_range[0]][3] * (t - self.coeff[0][sig-2][t_range[0]][0]) + 
                  self.coeff[0][sig-2][t_range[0]][4])

        self.vz = (3 * self.coeff[0][sig-2][t_range[0]][1] * ((t - self.coeff[0][sig-2][t_range[0]][0])**2) + 
                   2 * self.coeff[0][sig-2][t_range[0]][2] * (t - self.coeff[0][sig-2][t_range[0]][0]) + 
                   self.coeff[0][sig-2][t_range[0]][3])

        self.az = (6 * self.coeff[0][sig-2][t_range[0]][1] * (t - self.coeff[0][sig-2][t_range[0]][0]) + 
                   2 * self.coeff[0][sig-2][t_range[0]][2])

        self.y = (self.coeff[1][sig-2][t_range[1]][1] * ((t - self.coeff[1][sig-2][t_range[1]][0])**3) + 
                  self.coeff[1][sig-2][t_range[1]][2] * ((t - self.coeff[1][sig-2][t_range[1]][0])**2) + 
                  self.coeff[1][sig-2][t_range[1]][3] * (t - self.coeff[1][sig-2][t_range[1]][0]) + 
                  self.coeff[1][sig-2][t_range[1]][4])

        self.vy = (3 * self.coeff[1][sig-2][t_range[1]][1] * ((t - self.coeff[1][sig-2][t_range[1]][0])**2) + 
                   2 * self.coeff[1][sig-2][t_range[1]][2] * (t - self.coeff[1][sig-2][t_range[1]][0]) + 
                   self.coeff[1][sig-2][t_range[1]][3])

        self.ay = (6 * self.coeff[1][sig-2][t_range[1]][1] * (t - self.coeff[1][sig-2][t_range[1]][0]) + 
                   2 * self.coeff[1][sig-2][t_range[1]][2])

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
        self.z = self.z * scale + base_height
        self.vz = self.vz * scale
        self.az = self.az * scale


    def determine_range(self, sig, t):

        v_range = 0
        h_range = 0

        if sig == 4:
            if t < 0.1:
                v_range = 0
                h_range = 0
            elif t < 0.4:
                v_range = 1
                h_range = 0
            elif t < 0.6:
                v_range = 2
                h_range = 0
            elif t < 0.9:
                v_range = 3
                h_range = 0
            elif t < 1.0:
                v_range = 4
                h_range = 0
            elif t < 1.1:
                v_range = 4
                h_range = 1
            elif t < 1.4:
                v_range = 5
                h_range = 1
            elif t < 1.5:
                v_range = 6
                h_range = 1
            elif t < 1.6:
                v_range = 6
                h_range = 2
            elif t < 1.9:
                v_range = 7
                h_range = 2
            elif t < 2.0:
                v_range = 8
                h_range = 2
            elif t < 2.1:
                v_range = 8
                h_range = 3
            elif t < 2.4:
                v_range = 9
                h_range = 3
            elif t < 2.5:
                v_range = 10
                h_range = 3
            elif t < 2.6:
                v_range = 10
                h_range = 4
            elif t < 2.9:
                v_range = 11
                h_range = 4
            elif t < 3.1:
                v_range = 12
                h_range = 4
            elif t < 3.4:
                v_range = 13
                h_range = 4
            elif t < 3.5:
                v_range = 14
                h_range = 4
            elif t < 3.6:
                v_range = 14
                h_range = 5
            elif t < 3.9:
                v_range = 15
                h_range = 5
            else:
                v_range = 16
                h_range = 5

        elif sig == 3:
            if t < 0.1:
                v_range = 0
                h_range = 0
            elif t < 0.4:
                v_range = 1
                h_range = 0
            elif t < 0.5:
                v_range = 2
                h_range = 0
            elif t < 0.6:
                v_range = 2
                h_range = 1
            elif t < 0.9:
                v_range = 3
                h_range = 1
            elif t < 1.0:
                v_range = 4
                h_range = 1
            elif t < 1.1:
                v_range = 4
                h_range = 2
            elif t < 1.4:
                v_range = 5
                h_range = 2
            elif t < 1.5:
                v_range = 6
                h_range = 2
            elif t < 1.6:
                v_range = 6
                h_range = 3
            elif t < 1.9:
                v_range = 7
                h_range = 3
            elif t < 2.1:
                v_range = 8
                h_range = 3
            elif t < 2.4:
                v_range = 9
                h_range = 3
            elif t < 2.5:
                v_range = 10
                h_range = 3
            elif t < 2.6:
                v_range = 10
                h_range = 4
            elif t < 2.9:
                v_range = 11
                h_range = 4
            else:
                v_range = 12
                h_range = 4

        else: # sig == 2
            if t < 0.1:
                v_range = 0
                h_range = 0
            elif t < 0.4:
                v_range = 1
                h_range = 0
            elif t < 0.6:
                v_range = 2
                h_range = 1
            elif t < 0.9:
                v_range = 3
                h_range = 2
            elif t < 1.1:
                v_range = 4
                h_range = 2
            elif t < 1.4:
                v_range = 5
                h_range = 2
            elif t < 1.5:
                v_range = 6
                h_range = 2
            elif t < 1.6:
                v_range = 6
                h_range = 3
            elif t < 1.75:
                v_range = 7
                h_range = 3
            elif t < 1.9:
                v_range = 7
                h_range = 4
            else:
                v_range = 8
                h_range = 4
                
        return [v_range, h_range]

# Setup the ROS node
if __name__=='__main__':

    # Initialize the ROS node
    rospy.init_node('conductor')

    # Create an instance of ViconCoordinates
    current_state = Conductor()

    # Do not exit until shutdown
    rospy.spin()
