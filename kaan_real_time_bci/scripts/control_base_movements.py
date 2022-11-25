#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import math
#import copy
import numpy as np

# ROS libraries
import rospy

# ROS messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry

class MoveRobot(object):
    def __init__(self, hz = 10.0):
        
        self._pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)
        self._hz = hz #[Hz]
        self.rate = rospy.Rate(self._hz)     # hz should be retrieved from the parameter server.
                                        # A too low value would cause the robot to move in fits and starts.
        # The following attributes will determine the robot movement speed. Max velocity and acceleration are retrieved from the parameter server, i.e. the robot will move as fast as possible.
        self._angular_vel = rospy.get_param('/mobile_base_controller/angular/z/max_velocity', 
                                            2.0
                                            ) #[rad/s]
        self._linear_vel  = rospy.get_param('/mobile_base_controller/linear/x/max_velocity', 
                                            1.0
                                            ) #[m/s]        
        self._angular_acc = rospy.get_param('/mobile_base_controller/angular/z/max_acceleration', 
                                            2.0
                                            ) #[rad/s^2]
        self._linear_acc  = rospy.get_param('/mobile_base_controller/linear/x/max_acceleration', 
                                            1.0
                                            ) #[m/s] 
        
        rospy.loginfo('Waiting for connection...')

        self.sub_move_robot = rospy.Subscriber('/socket_distributer_move_robot/data',String, self.move_Tiago) # self.sub_move_robot is a dummy variables no use, just call
        
        #self._odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self._update_odom)
        # self.event_sub = ...
    
    def move_Tiago(self,msg): # msg name not important based on me    
        # Ask the type of movement (rotation or translation)

        move_dir= msg.data
        #print(move_dir)

        if move_dir=="Right":
            move_type="R"
            mt_amount=-15
        elif move_dir=="Left":
            move_type="R"
            mt_amount=15
        elif move_dir=="Forward":
            move_type="T"
            mt_amount=1
        elif move_dir=="Close": # NOT DEFINED NOT NECESSARY
            move_type=" "
            mt_amount=" "
        else:
            move_type=" "
            mt_amount=" "
        
        if move_type!=" ": 
            if move_type.upper() == 'T':
                self.translation=True
                self.translation_target = float(mt_amount)
            elif move_type.upper() == 'R':
                self.translation=False
                self.rotation_target = float(mt_amount) * math.pi / 180.0
            self.move()

    def move(self):
        """ Move the robot base. The movement is controlled in open loop, following a trapezoidal velocity profile. The type of movement (translation or rotation) is determined depending on the values of translation_target and rotation_target (i.e. one should be set to None).
        """
        target = self.translation_target if self.translation else self.rotation_target
        if self.translation:
            vel_profile = self._trapezoidal_vel_profile(v_max = self._linear_vel/4, 
                                                        a_max = self._linear_acc/4, 
                                                        s_tot = abs(target) 
                                                        )
        elif not self.translation:
            vel_profile = self._trapezoidal_vel_profile(v_max = self._angular_vel/8, 
                                                        a_max = self._angular_acc/8, 
                                                        s_tot = abs(target) 
                                                        )
        if target <= 0: # if the rotation is clockwise or the translation is backward
                        # the sign of the velocity has to be inverted
            vel_profile *= -1
        for v in vel_profile:
            if self.translation:
                self._publish(linear = v)
            elif not self.translation:      # if rotation
                self._publish(angular = v)
            self.rate.sleep()   # keep the loop at the desired frequency


    def _trapezoidal_vel_profile(self, v_max, a_max, s_tot):
        """ Generate a trapezoidal velocity profile to control the base movement in feedforward.
        The initial velocity is assumed to be 0. Only works with positive values. If working with negative values, use the absolute values and change the sign of the return.

        Args:
            v_max (float): The maximum velocity. Measurement unit depends on input.
            a_max (float): The maximum acceleration. Measurement unit depends on input.
            s_tot (float): The overall displacement. Measurement unit depends on input.
        
        Returns:
            numpy.ndarray: The velocity values to be sent to the base (assumed to be 1/hz seconds apart).
        """
        # The duration of the acceleration and deceleration phase (and also the time instant at which the peak velocity is reached, assuming 0 at start).
        t_acc = min([math.sqrt(s_tot / a_max),  # if triangular
                     v_max / a_max              # if trapezoidal
                    ])
        # The duration of the constant speed phase. 
        t_vmax = max([0.0,                          # if triangular (i.e. no constant speed phase)
                      s_tot / v_max - v_max / a_max # if trapezoidal
                      ])
        v_peak = t_acc * a_max  # maximum velocity reached during the movement
        # Velocity values to send to the base.
        v_acc = np.linspace(0.0,
                            v_peak,
                            num = int(self._hz * t_acc),
                            endpoint = False
                            )
        v_const = [v_max] * (int(self._hz * t_vmax) - 1)
        v_dec = np.linspace(v_peak,
                            0.0, 
                            num = int(self._hz * t_acc),
                            endpoint = True
                            )
        return np.concatenate((v_acc, v_const, v_dec))


    def _publish(self, linear = 0.0, angular = 0.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._pub_cmd.publish(twist)

    '''
    def _update_odom(self, odom_msg):
        """ Callback of the odometry subscriber. Updates the attributes with the current position and orientation of the robot.

        Args:
            odom_msg (nav_msgs/Odometry): the robot odometry, as estimated by its onboard sensors.
        """
        self.pos = odom_msg.pose.pose.position
        self.ori = odom_msg.pose.pose.orientation
    '''

    '''
    def _quat_to_rotation_around_z(self, quat):
        # ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        qz = sin(rot/2)
    '''

if __name__ == '__main__':
    
    rospy.init_node('socket_server', anonymous = True)
    
    mr = MoveRobot()
    try:
        while not rospy.is_shutdown():
            pass
    
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down the node.')
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down the node.')
