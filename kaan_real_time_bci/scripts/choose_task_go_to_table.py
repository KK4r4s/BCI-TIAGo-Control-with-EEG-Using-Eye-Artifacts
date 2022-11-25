#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np

# ROS libraries
import rospy

# ROS messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range # back sensor of the robot
from sensor_msgs.msg import LaserScan

class MoveRobot(object):
    def __init__(self, hz = 10.0):
        
        self._pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)
        
        self._hz = hz #[Hz]
        self.rate = rospy.Rate(self._hz) 

        self.table_pos_x = 1
        self.table_pos_y = 2
        self.table_pos_ang = math.pi/2 # rad: 0 , math.pi/2 , math.pi, 3*math.pi/2 , 0

        self._angular_vel = rospy.get_param('/mobile_base_controller/angular/z/max_velocity', 2.0) #[rad/s]
        self._linear_vel  = rospy.get_param('/mobile_base_controller/linear/x/max_velocity', 1.0) #[m/s]        
        self._angular_acc = rospy.get_param('/mobile_base_controller/angular/z/max_acceleration', 2.0) #[rad/s^2]
        self._linear_acc  = rospy.get_param('/mobile_base_controller/linear/x/max_acceleration', 1.0) #[m/s] 
        
        rospy.loginfo('Waiting for connection...')
        # From Robot
        self._odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self._update_odom)
        self._range_sub = rospy.Subscriber('/sonar_base', Range, self._update_back_dist)
        self._laserscan_sub = rospy.Subscriber('/scan_raw', LaserScan, self._update_front_dist)
        # From Socket
        self.sub_table = rospy.Subscriber('/socket_distributer_table/data',String, self.Tiago_Table) # self.sub_move_robot is a dummy variables no use, just call
    
    def _update_back_dist(self, range_msgs):
        self.back_range = range_msgs.range # a float number
        
    def _update_front_dist(self, laserscan_msgs):
        self.front_dist = laserscan_msgs.ranges # vector of size 666x1
            
    def _update_odom(self, odom_msg):
        """ Callback of the odometry subscriber. Updates the attributes with the current position and orientation of the robot.

        Args:
            odom_msg (nav_msgs/Odometry): the robot odometry, as estimated by its onboard sensors.
        """
        self.pos = odom_msg.pose.pose.position
        self.ori = odom_msg.pose.pose.orientation

    def Tiago_Table(self,msg):   
        
        rec_data=msg.data # not used # Uncomment it if using with gui
        
        pos_x, pos_y, ori = self._bring_pos_ori() # units: meters, meters, radians
        
        # Calculate Distance and angle to the Table
        delta_dist, target_ang =self._calculate_delta(pos_x,pos_y)

        while abs(target_ang-ori)*180/math.pi > 0.1:              
            pos_x, pos_y, ori = self._bring_pos_ori() # units: meters, meters, radians
            min_rotation_angle = self._return_angle(desired_pos=target_ang, current_pos=ori)
            
            if abs(360-abs(target_ang - ori)*180/math.pi)<=0.1:
                break

            self.translation=False
            self.rotation_target = float(min_rotation_angle) # radians target_ang-ori
            self.move()
        print "1)", target_ang*180/math.pi, ori*180/math.pi 
        
        # Go to the Point on x-y axis
        while abs(pos_x-self.table_pos_x) > 0.1 or abs(pos_y-self.table_pos_y) > 0.1:
            pos_x, pos_y, ori = self._bring_pos_ori() # units: meters, meters, radians
            # Calculate Distance and angle to the Table
            delta_dist, target_ang =self._calculate_delta(pos_x,pos_y)
        
            if abs(target_ang - ori)*180/math.pi > 1:
                min_rotation_angle = self._return_angle(desired_pos=target_ang, current_pos=ori)

                # Rotate to the Desk
                self.translation=False
                self.rotation_target = float(min_rotation_angle) # radians target_ang-ori
                self.move()
            
           
            # Move Forward
            self.translation=True
            self.translation_target = float(delta_dist)
            self.move()

        print "2)", target_ang*180/math.pi, ori*180/math.pi 
        
        # Rotate Towards the Table
        while abs(self.table_pos_ang - ori)*180/math.pi > 0.5: 
            
            pos_x, pos_y, ori = self._bring_pos_ori() # units: meters, meters, radians
            min_rot_ang = self._return_angle(desired_pos=self.table_pos_ang, current_pos=ori)
            
            if abs(360 - (abs(self.table_pos_ang - ori)*180/math.pi))<= 0.4:
                break 

            # Rotate
            self.translation=False
            self.rotation_target = float(min_rot_ang) # radians
            self.move()
        print "3)",self.table_pos_ang*180/math.pi, ori*180/math.pi  
        
        pos_x, pos_y, ori = self._bring_pos_ori() # units: meters, meters, radians
        print "Final Pos_1: ", pos_x, " ", pos_y, " ",ori*180/math.pi, "degrees"
        
        # Continue to Move if you are far from the Table at least 15cm
        while np.any(np.array(self.front_dist[300:360])>0.15):
            self.translation=True
            self.rotation_target = float(self.front_dist[332] - 0.15) # radians
            self.move()
            
        pos_x, pos_y, ori = self._bring_pos_ori() # units: meters, meters, radians
        print "Final Pos_2: ", pos_x, " ", pos_y, " ",ori*180/math.pi, "degrees"
        print "Front LaserScan:", self.front_dist[332], "Back Range:", self.back_range
        print "TiaGo arrived to Table"

    def _calculate_delta(self,pos_x,pos_y):
        delta_x = self.table_pos_x - pos_x
        delta_y = self.table_pos_y - pos_y
        delta_dist = math.sqrt(delta_x**2+delta_y**2)

        if delta_x<0 and delta_y>0:
            target_ang = math.pi + math.atan(delta_y/delta_x) # radians
        elif delta_x<0 and delta_y<0:
            target_ang = math.pi + math.atan(delta_y/delta_x) # radians
        elif delta_x>0 and delta_y<0:
            target_ang =  2*math.pi + math.atan(delta_y/delta_x) # radians
        else:
            target_ang = math.atan(delta_y/delta_x) # radians

        return delta_dist, target_ang 

    def _return_angle(self,desired_pos,current_pos):

        angle_of_int_1 = desired_pos - current_pos
        if angle_of_int_1 >0:
            angle_of_int_2 = angle_of_int_1 - 2*math.pi
        else:
            angle_of_int_2 = 2*math.pi +angle_of_int_1 

        if abs(angle_of_int_2) < abs(angle_of_int_1):
            min_rotation_angle = angle_of_int_2
        else:
            min_rotation_angle = angle_of_int_1

        # print min_rotation_angle, angle_of_int_2, angle_of_int_1

        return min_rotation_angle
    def _bring_pos_ori(self):
        pos_x = self.pos.x
        pos_y = self.pos.y
        orientation = [self.ori.x, self.ori.y, self.ori.z, self.ori.w ]
        orientation = self._quat_to_rotation_euler(orientation) # convert to quaternion to rad and then degree 
        ori=orientation[2] # radians
        return pos_x, pos_y, ori

    def _quat_to_rotation_euler(self, quat , unit="radian", full_circle =True ):
        # ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        q_x, q_y, q_z, q_w = quat

        t0 = +2.0 * (q_w * q_x + q_y * q_z)
        t1 = +1.0 - 2.0 * (q_x * q_x + q_y * q_y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (q_w * q_y - q_z * q_x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (q_w * q_z + q_x * q_y)
        t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        yaw_z = math.atan2(t3, t4)

        roll_x=roll_x #*(180/math.pi)
        pitch_y=pitch_y #*(180/math.pi)
        yaw_z=yaw_z #*(180/math.pi)

        if unit=="radian":
            roll_x=roll_x 
            pitch_y=pitch_y 
            yaw_z=yaw_z 
            if full_circle:
                if roll_x<0:
                    roll_x=roll_x + 2*math.pi
                else:
                    pass
                if pitch_y<0:
                    pitch_y=pitch_y + 2*math.pi
                else:
                    pass
                if yaw_z<0:
                    yaw_z=yaw_z + 2*math.pi
                else:
                    pass
        elif unit=="degree":
            roll_x=roll_x * (180/math.pi)
            pitch_y=pitch_y * (180/math.pi)
            yaw_z=yaw_z * (180/math.pi)
            if full_circle:
                if roll_x<0:
                    roll_x=roll_x + 360
                else:
                    pass
                if pitch_y<0:
                    pitch_y=pitch_y + 360
                else:
                    pass
                if yaw_z<0:
                    yaw_z=yaw_z + 360
                else:
                    pass
        
        euler_rad = [roll_x, pitch_y, yaw_z]
        return euler_rad

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
            # Safety    
            # if self.translation:
            #    if np.any(np.array(self.front_dist[300:360]) < 0.20):
            #        print "Front of the TiaGo Closer than 0.2 meters to Object"
            #        self._publish()
            #        break
                #if self.back_range<0.15:
                #    print "Back of the TiaGo Closer than 0.15 meters to Object"
                #    self._publish()
                #    break
            # Safety
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
