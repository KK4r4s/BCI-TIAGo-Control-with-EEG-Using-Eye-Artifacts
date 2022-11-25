#!/usr/bin/env python
# Make the robot grasp objects from table top

# Python libs
import sys, os
import json
import numpy as np
import math

# ROS libs
import tf2_ros  # probably not needed  
import rospy
import moveit_commander 

# ROS messages

from actionlib import SimpleActionClient
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
#from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import MoveItErrorCodes
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Quaternion, PointStamped, Pose, Point, PoseStamped, Vector3, Vector3Stamped
from kaan_real_time_bci.srv import PickPose, PickPoseResponse

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class TableTopGrasp:
    def __init__(self):
        '''
        # Listen to transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        '''

        # Connect to the pick and place server (by PAL Robotics)
        rospy.loginfo("Waiting for /pickup_pose Action Server...")
        self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
        rospy.sleep(1.0)
        if not self.pick_as.wait_for_server(rospy.Duration(60)):
            rospy.logerr("Could not connect to /pickup_pose Action Server.")
            exit()

        # Define a service to trigger the pick task
        rospy.Service('/tabletop_pick/object_pose', PickPose, self.pick)
        
        # Define controllers for all the robot's groups. TODO: use `motion_utils` instead.
        rospy.loginfo("Setting publishers to torso and head controller...") 
        # TODO: check if they are needed
        self.torso_cmd = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_cmd = rospy.Publisher(
            '/head_controller/command', JointTrajectory, queue_size=1)
        self.gripper_cmd = rospy.Publisher(
            '/gripper_controller/command', JointTrajectory, queue_size=1)
        
        self.gripper_state_sub = rospy.Subscriber('gripper_controller/state', JointTrajectoryControllerState, self.check_grasp)
        self.check_gripper_state = False
        
        # Initialize a moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        # Exploit MoveIt to move TIAGo
        self.robot = moveit_commander.RobotCommander()  # the interface with the robot
        self.scene = moveit_commander.PlanningSceneInterface()  # the interface with the world surrounding the robot
        # Define a commander for each group of the robot (if all of them are needed)
        self.arm = moveit_commander.MoveGroupCommander('arm')
        # Pre-define the pose to which the arm has to go before attemping to plan the grasp
        self.arm.remember_joint_values('pregrasp_pose', 
                                        values = [45 * math.pi/180, 
                                                  50 * math.pi/180, 
                                                 -30 * math.pi/180, 
                                                  90 * math.pi/180, 
                                                 -90 * math.pi/180, 
                                                 -30 * math.pi/180, 
                                                 -30 * math.pi/180]
                                        )

        self.arm.set_max_velocity_scaling_factor(0.5)     # no effect on pick server
        self.arm.set_max_acceleration_scaling_factor(0.5) # no effect on pick server
        rospy.loginfo('MoveIt commander initialized.\n')

        rospy.loginfo("Connecting to clear octomap service...")
        self.clear_octomap_srv = rospy.ServiceProxy(
            '/clear_octomap', Empty)
        self.clear_octomap_srv.wait_for_service()
        rospy.loginfo("Connected!")
        rospy.loginfo("The tabletop_grasp node is operating")

    
    def pick(self, pose):
        # Call the Pick and Place server
        pickup_pose = PickUpPoseGoal()
        pickup_pose.object_pose = pose.object_pose
        ttop_grasp.pick_as.send_goal_and_wait(pickup_pose)

        result = ttop_grasp.pick_as.get_result()
        if str(moveit_error_dict[result.error_code]) != "SUCCESS":
            if str(moveit_error_dict[result.error_code]) == "CONTROL_FAILED":
                # The source of the above error is unclear, but it usually happens when the gripper has already arrived in the final position. Hence 'hope' to be already there and close the gripper anyway. TODO: check if the current position is close to the desired final one. If yes, close gripper; if no, return error.
                rospy.logwarn("Failed to pick, try to close the gripper anyway.")
                touch_links = ttop_grasp.robot.get_link_names(group = 'gripper')
                ttop_grasp.scene.attach_box('arm_tool_link', 'part', touch_links = touch_links)
                ttop_grasp.close_gripper()
            if str(moveit_error_dict[result.error_code]) == "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE":
                rospy.logwarn("Failed to pick, clearing the octomap before trying again.")
                # TODO: trigger new trial (for a given number of attempts)
        
        #self.check_gripper_state = True

        response = PickPoseResponse()
        response.result = moveit_error_dict[result.error_code]
        response.grasp = True #self.grasp
        return response


    def open_gripper(self):
        rospy.loginfo("Opening gripper...")

        jt = JointTrajectory()
        jt.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.04, 0.04]
        jtp.time_from_start = rospy.Duration(1.0)
        jt.points.append(jtp)
        self.gripper_cmd.publish(jt)
        rospy.loginfo("Gripper open.")


    def close_gripper(self):
        rospy.loginfo("Closing gripper...")

        jt = JointTrajectory()
        jt.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.02, 0.02]
        jtp.time_from_start = rospy.Duration(1.0)
        jt.points.append(jtp)
        self.gripper_cmd.publish(jt)
        rospy.loginfo("Gripper closed.")


    def check_grasp(self, msg):
        if not self.check_gripper_state:
            return 
        eps = 0.01      #[m]
        v_still = 0.1   #[m/s]  check if makes sense
        v = msg.actual.velocities
        e = msg.error.positions
        print('Gripper plates are moving at ' + str(v))
        if abs(v[0]) + abs(v[1]) < v_still:
            return  # If the gripper plates are still moving, wait for them to stop
        
        print('Gripper position error is ' + str(e))
        if e[0] + e[1] > eps:
            # If the error is (relatively) large and the plates are not moving, then something is blocking the gripper
            rospy.loginfo('There is something in the gripper!')
            self.grasp = True
        else:
            # If the gripper is in the desired position but it has stopped moving, then assume that there is nothing in the gripper
            rospy.loginfo('There is nothing in the gripper.')
            self.grasp = False

    def remove_planning_scene_grasped_object():
        self.scene.remove_attached_object("arm_tool_link")
        rospy.sleep(1)
        self.scene.remove_world_object("part")
        

    def prepare_robot(self, attempts_num = 3):
        plan = self.arm.plan('pregrasp_pose')
        if plan.joint_trajectory.points: # True if trajectory contains points
            self.arm.execute(plan)
        else:
            rospy.logwarn('The pregrasp pose cannot be safely reached now.' +
                            'Clearing the octomap and trying again.')
            for attempt in range(attempts_num):
                self.clear_octomap_srv.call(EmptyRequest())
                plan = self.arm.plan('pregrasp_pose')
                if plan.joint_trajectory.points: # True if trajectory contains points
                    self.arm.execute(plan)
                else:
                    rospy.logerr('The pregrasp pose cannot be safely reached now.')
                    return False

        rospy.loginfo("Robot prepared.")
        return True

if __name__ == "__main__":
    rospy.init_node('tabletop_grasp', anonymous = True)
    ttop_grasp = TableTopGrasp()

    try:
        while not rospy.is_shutdown():
            pass     
            
    except KeyboardInterrupt:
        print("Shutting down the coordinate finder module.")
