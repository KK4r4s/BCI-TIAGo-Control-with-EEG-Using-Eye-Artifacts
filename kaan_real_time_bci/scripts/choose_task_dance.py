#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
controllers: 
        arm_controller :  7 joints, arm1: 0.07-2.67, arm2: -1.5-1.02, arm3: -3.46-1.5
                                    arm4: -0.32-2.28, arm5: -2.07-2.07, arm6: -1.39-1.39
                                    arm7: -2.07-2.07
                                    ex: move_point=[0.07, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0 ]

        head_controller : 2 joints, L/R: -1.24-1.24, D/U: -0.98-0.72
                                    ex: move_point = [-1.24, 0.72]

        torso_controller : 1 joint, 0-0.35, ex: move_point = 0.25 or move_point = [0.25 ]

        gripper_controller: 2 joints 0.001-0.044 both, ex: move_point = [0.044 , 0.044]
        
"""

import math
import numpy as np

# ROS libraries
import rospy

# ROS messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState 
from nav_msgs.msg import Odometry

class MoveRobot(object):
    def __init__(self, hz = 10.0):
        
        self._cmd_base_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 20)
        self._cmd_arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size = 20)
        self._cmd_gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size = 20)
        self._cmd_torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size = 20)
        self._cmd_head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size = 20)

        self._hz = hz #[Hz]
        self.rate = rospy.Rate(self._hz)     # hz should be retrieved from the parameter server.
        
        self.pos = []
        self.ori = []
        self.orientation = []
        self.position = []

        self._angular_vel = rospy.get_param('/mobile_base_controller/angular/z/max_velocity', 2.0) #[rad/s]
        self._linear_vel  = rospy.get_param('/mobile_base_controller/linear/x/max_velocity', 1.0) #[m/s]        
        self._angular_acc = rospy.get_param('/mobile_base_controller/angular/z/max_acceleration', 2.0) #[rad/s^2]
        self._linear_acc  = rospy.get_param('/mobile_base_controller/linear/x/max_acceleration', 1.0) #[m/s] 
        
        self.sub_dance = rospy.Subscriber('/socket_distributer_dance/data',String, self.TiaGo_dance)
        
        self._odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self._update_odom)

        msg_arm = rospy.wait_for_message('/arm_controller/state',JointTrajectoryControllerState )
        self._state_cb(msg_arm)
        self._state_sub = rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, self._state_cb)

        rospy.loginfo('Waiting for connection...')
    
    def _update_odom(self, odom_msg):
        self.pos = odom_msg.pose.pose.position
        self.ori = odom_msg.pose.pose.orientation
        
        position = [self.pos.x, self.pos.y, self.pos.z]
        orientation = [self.ori.x, self.ori.y, self.ori.z, self.ori.w ]
        orientation = self._quat_to_rotation_euler(orientation) # convert to quaternion to rad and then degree 
        
        self.position.append(position)
        self.orientation.append(orientation)

    def _quat_to_rotation_euler(self, quat , unit="degree", full_circle =False ):
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

    def _state_cb(self,msg):
        joint_info=msg.joint_names[0].split("_")[0]
        if  joint_info == "arm": # If new joints requiered to add 
            self.arm_pos=msg.actual.positions
            

    def TiaGo_dance(self,msg):    
        rec_data=msg.data 

        torso_jtc_p = [0.3]
        self._publish_torso(torso_jtc_p, dur =1 )  

        arm_jtc_p = [0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0 ] # Initial position
        self._publish_arm(arm_jtc_p, dur = 3 ) 
        while abs(self.arm_pos[1] - arm_jtc_p[1]) > 0.01:
            pass

        head_jtc_p=[ 1.20, 0.7 ]
        self._publish_head(head_jtc_p, dur =1 )

        arm_jtc_p = [0.2, 0.3, 0.0, 1.94, -1.57, 0.0, 0.0 ] # First arm movement
        self._publish_arm(arm_jtc_p, dur = 2 )
        while abs(self.arm_pos[1] - arm_jtc_p[1]) > 0.01:
            pass

        head_jtc_p=[ -1.24, -0.70 ]
        self._publish_head(head_jtc_p, dur = 1 )

        arm_jtc_p = [0.2, 0.3, 0.0, 0.6, 1.57, 0.0, -0.9 ] # Second arm movement
        self._publish_arm(arm_jtc_p, dur = 2 )
        while abs(self.arm_pos[4] - arm_jtc_p[4]) > 0.01:
            pass
        
        arm_jtc_p = [0.2, 0.3, 0.0, 1.94, -1.57, 0.0, 0.0 ] # First arm movement
        self._publish_arm(arm_jtc_p, dur = 2 )
        while abs(self.arm_pos[4] - arm_jtc_p[4]) > 0.01:
            pass

        arm_jtc_p = [0.2, 0.3, 0.0, 0.6, 1.57, 0.0, -0.9 ] # Second arm movement
        self._publish_arm(arm_jtc_p, dur = 2 )
        while abs(self.arm_pos[4] - arm_jtc_p[4]) > 0.01:
            pass

        torso_jtc_p = [0.15]
        self._publish_torso(torso_jtc_p, dur =2 )

        arm_jtc_p = [0.2, 0.3, 0.0, 1.94, -1.57, 0.0, 1.8 ] # Third arm movement
        self._publish_arm(arm_jtc_p, dur = 2 )
        while abs(self.arm_pos[4] - arm_jtc_p[4]) > 0.01:
            pass

        head_jtc_p=[ 0.0 ,0.0 ]
        self._publish_head(head_jtc_p, dur =1 )

        arm_jtc_p = [0.2, 0.3, 0.0, 1.94, 1.57, 0.0, -0.9 ] # Fourth arm movement
        self._publish_arm(arm_jtc_p, dur = 1 )
        while abs(self.arm_pos[5] - arm_jtc_p[5]) > 0.01:
            pass

        torso_jtc_p = [0.3]
        self._publish_torso(torso_jtc_p, dur =2 )

        arm_jtc_p = [2.6, 0.0, -0.2, 1.94, -1.57, 0.5, 1.15 ] # Fifth arm movement
        self._publish_arm(arm_jtc_p, dur = 3 )
        while abs(self.arm_pos[5] - arm_jtc_p[5]) > 0.01:
            pass

        torso_jtc_p = [0.1]
        self._publish_torso(torso_jtc_p, dur =2 )

        arm_jtc_p = [0.2, -0.55, -2.7, 1.94, 1.57, 0.0, -0.9 ] # Sixth movement
        # Tiago Turn 
        self._publish_arm(arm_jtc_p, dur = 3 )
        while abs(self.arm_pos[5] - arm_jtc_p[5]) > 0.01:
            self._publish_rotate(angular=2)
        print("Orientation_1stloop: ", self.orientation[-1][2])
        while abs(self.orientation[-1][2])>1:
            if self.orientation[-1][2]<0:
                self._publish_rotate(angular=0.3)
            elif self.orientation[-1][2]>0:
                self._publish_rotate(angular=-0.3) 
        print("Orientation_2ndloop: ", self.orientation[-1][2])
        
        torso_jtc_p = [0.3]
        self._publish_torso(torso_jtc_p, dur =5 )    

        head_jtc_p=[ -0.72, 0.26 ]
        self._publish_head(head_jtc_p, dur = 1 )

        arm_jtc_p = [0.2, -0.55, -2.7, 2.25, 1.57, 0.0, -0.9 ] #  Seventh movement
        self._publish_arm(arm_jtc_p, dur = 1 )
        while abs(self.arm_pos[4] - arm_jtc_p[4]) > 0.01:
            pass

        torso_jtc_p = [0.1]
        self._publish_torso(torso_jtc_p, dur =2 )

        arm_jtc_p = [0.2, 0.55, -2.7, 1.40, 1.57, 0.0, -0.9 ] #  Eighth movement
        self._publish_arm(arm_jtc_p, dur = 2 )
        while abs(self.arm_pos[1] - arm_jtc_p[1]) > 0.01:
            pass

        torso_jtc_p = [0.3]
        self._publish_torso(torso_jtc_p, dur =2 )

        arm_jtc_p = [0.2, -0.55, -2.7, 2.10, 1.57, 0.0, -0.9 ] #  Nineth movement
        self._publish_arm(arm_jtc_p, dur = 2 )
        while abs(self.arm_pos[1] - arm_jtc_p[1]) > 0.01:
            pass

        torso_jtc_p = [0.1]
        self._publish_torso(torso_jtc_p, dur =2 )

        arm_jtc_p = [0.2, 0.55, -2.7, 1.40, 1.57, 0.0, -0.9 ] #  Eighth movement
        self._publish_arm(arm_jtc_p, dur = 2 )
        while abs(self.arm_pos[1] - arm_jtc_p[1]) > 0.01:
            pass

        torso_jtc_p = [0.3]
        self._publish_torso(torso_jtc_p, dur =2 )

        arm_jtc_p = [0.2, -0.55, -2.7, 2.10, 1.57, 0.0, -0.9 ] #  Nineth movement
        self._publish_arm(arm_jtc_p, dur = 2 )
        while abs(self.arm_pos[1] - arm_jtc_p[1]) > 0.01:
            pass
        
        # Return to Initial Position

        head_jtc_p=[ 0.0, 0.0 ]
        self._publish_head(head_jtc_p, dur = 1 )

        arm_jtc_p = [0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0 ] 
        self._publish_arm(arm_jtc_p, dur = 3 ) 
        while abs(self.arm_pos[2] - arm_jtc_p[2]) > 0.01:
            pass

        torso_jtc_p = [0.2]
        self._publish_torso(torso_jtc_p, dur =2 )
        
        print("Performance Completed")
        print("Final Position: ",self.pos.x, self.pos.y, self.orientation[-1][2])


    def _publish_torso(self,torso_jtc_p,dur):
        if type(torso_jtc_p) is float:
            torso_jtc_p=[torso_jtc_p]
        else: 
            pass
        traj = JointTrajectory()
        traj.joint_names = ['torso_lift_joint']
        
        point = JointTrajectoryPoint()
        point.positions.append(torso_jtc_p[0])
        point.time_from_start = rospy.Duration(dur)

        traj.points.append(point)
        self._cmd_torso_pub.publish(traj)
        self.rate.sleep()

        
    def _publish_arm(self,arm_jtc_p,dur):
        traj = JointTrajectory()
        traj.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
         'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

        point = JointTrajectoryPoint()
        point.positions.append(arm_jtc_p[0])
        point.positions.append(arm_jtc_p[1])
        point.positions.append(arm_jtc_p[2])
        point.positions.append(arm_jtc_p[3])
        point.positions.append(arm_jtc_p[4])
        point.positions.append(arm_jtc_p[5])
        point.positions.append(arm_jtc_p[6])
        point.time_from_start = rospy.Duration(dur)

        traj.points.append(point)
        self._cmd_arm_pub.publish(traj)
        self.rate.sleep()

    def _publish_head(self,head_jtc_p,dur):
        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']

        point = JointTrajectoryPoint()
        point.positions.append(head_jtc_p[0])
        point.positions.append(head_jtc_p[1])
        point.time_from_start = rospy.Duration(dur)

        traj.points.append(point)
        self._cmd_head_pub.publish(traj)
        self.rate.sleep()

   

    def _publish_rotate(self, linear = 0.0, angular = 0.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._cmd_base_pub.publish(twist)

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
