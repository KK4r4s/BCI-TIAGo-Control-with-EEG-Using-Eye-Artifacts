#!/usr/bin/env python

import os
import numpy as np
import cv2
import math

import rospy
import tf2_ros
from utils import decoder
from utils import transforms

from std_msgs.msg import String
from geometry_msgs.msg import Transform, PoseStamped, Twist
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kaan_real_time_bci.srv import ClustersPose, PickPose


class MoveCamera:
    def __init__(self):
        # Get ready to send command to the head and the torso to re-orient the camera
        self.head_cmd = rospy.Publisher('/head_controller/command', 
                                        JointTrajectory, 
                                        queue_size=1)
        self.torso_cmd = rospy.Publisher('/torso_controller/command', 
                                         JointTrajectory, 
                                         queue_size=1)
        rospy.sleep(2)
    
    def lift_torso(self, h = 0.30):
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [h]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)


    def lower_head(self):
        rospy.loginfo("Moving head down")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.6]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")


class PickAndPlaceManager:
    '''NOTE: currently just pick'''
    def __init__(self):
        self._pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)
        
        # Connect to the server for plane segmentation and object extraction
        rospy.loginfo("Waiting for /tabletop_segmentation/get_objects Action Server...")
        self.segmentation_server = rospy.ServiceProxy('/tabletop_segmentation/get_poses', ClustersPose)
        rospy.sleep(1.0)
        
        # Connect to the server for pick (and place) actions
        self.pick_server = rospy.ServiceProxy('/tabletop_pick/object_pose', PickPose)
        rospy.sleep(1.0)
        
        ''' TODO: implement this instead of hard-coding the Transform!
        # Start TransformListener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        '''

        # Compatibility with Kaan framework
        self.mv_camera = MoveCamera()
        # Turn back
        self._hz = 10 #[Hz]
        self.rate = rospy.Rate(self._hz)     # hz should be retrieved from the parameter server.
          
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

        self.sub_scan = rospy.Subscriber('/socket_distributer_scan/data', String, self.scan_cb)
        self.sub_pick = rospy.Subscriber('/socket_distributer_pick/data', String, self.pick_cb) 


    def scan_cb(self, msg):
        # Get the location of this script
        wd = os.path.abspath(os.path.dirname(__file__))
        # Bring the camera in the predefined position
        self.mv_camera.lift_torso()
        self.mv_camera.lower_head()
        rospy.sleep(5)
        # Get the pose of the objects
        self.poses = self.segmentation_server()
        # NOTE: this does not make much sense as a topic callback is here mainly used to call a service... Nevertheless it is a quick solution to combine a re-usable implementation of the scan&pick pipeline with Kaan's application architecture. 

        # Sort the poses based on y coordinate (increasing) before drawing (so to browse through objects from left to right).
        self.poses.objects_pose.sort(key=lambda x: x.pose.position.y, reverse = True)

        # Get an image and draw the points of the centers of the objects
        imgs = self.draw_points(self.poses.objects_pose)
        # Create the directory in which images will be saved if does not exist
        try:
            os.mkdir(os.path.join(wd, '..', 'images'))
        except OSError:
            pass    # the `images` folder already exists
        for i, img in enumerate(imgs):
            ''' Kept for demo purpose
            cv2.imshow('highlight obj ' + str(i + 1), img)
            cv2.waitKey(0)
            '''
            # Save each image to later load and display it in the GUI
            cv2.imwrite(os.path.join(wd, '..', 'images',str(i) + '.png'), img)
            # NOTE: this is a workaround as streaming the camera images in real-time with the superimposed centers is not trivial having GUI-ROS communication via socket. Given this issue, the quasi-static situation, and the saving of resources, it looks a suitable solution.
        rospy.loginfo('Scan has finished.')


    def pick_cb(self, msg):
        event, obj_id = msg.data.split('/')
        if not event == 'Pick':
            return
        # Adapt the objects center to the plane height (as the plane height is more reliable).
        for i in range(len(self.poses.objects_pose)):
            self.poses.objects_pose[i].pose.position.z = self.poses.plane_pose.pose.position.z + 0.10
            # NOTE: the 0.10 is meant to be half of the object height, make it a parameter

        # Wait for the user to ask for a message 
        outcome = pap_manager.pick_server(self.poses.objects_pose[int(obj_id)])
        rospy.loginfo('Pick finished with outcome ' + outcome.result)
        if outcome.grasp:
            rospy.loginfo('Successful grasp')
            self.mv_camera.lift_torso(h = 0.34) # raise the torso to lift the object
            # NOTE: if torso is already at full extension after pick, the object will slide on the table...
            # TODO: add the `bring to patient` task here
            self._come_back()

    def _come_back(self):

        self.translation=True
        self.translation_target = float(-1.5)
        self.move()
        
        self.translation=False
        self.rotation_target = float(180) * math.pi / 180.0
        self.move()

        self.translation=True
        self.translation_target = float(1)
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

    def draw_points(self, poses):
        """Acquire one image and draw the centroid of the points contained in Pose messages.

        Args:
            poses (geometry_msgs/Pose): Poses of the objects to highlight. Just the `position` field of the Pose message is used.
        """
        # Get the image
        img = self._get_image(color_topic = '/xtion/rgb/image_raw')
        # Get the keypoints expressed in pixel coordinates
        points = self._project_points_to_image([pose.pose.position for pose in poses])

        imgs_with_circles = []
        for i, highlight_point in enumerate(points):
            img_with_circles = img.copy()
            for point in points:
                cv2.circle(img_with_circles, point, radius = 7, color = (255, 255, 255), thickness = -1)
                if point == highlight_point:
                    cv2.circle(img_with_circles, point, radius = 5, color = (0, 255, 0), thickness = -1)
                else:
                    cv2.circle(img_with_circles, point, radius = 5, color = (150, 150, 150), thickness = -1)
            imgs_with_circles.append(img_with_circles)
        return imgs_with_circles


    def _get_camera_orientation(self): #, cam_frame, to_frame = 'odom'):
        # TODO: use `tf2_ros` instead. Don't be lazy!
        T = Transform()
        ''' `rosrun tf tf_echo xtion_rgb_optical_frame base_footprint`'''
        T.translation.x = 0.022
        T.translation.y = 1.212
        T.translation.z = 0.532
        T.rotation.x = -0.626
        T.rotation.y = 0.619
        T.rotation.z = -0.329
        T.rotation.w = -0.329
        
        return T


    def _project_points_to_image(self, points):
        """Project the Point ROS objects to the camera plane.

        Args:
            points ([geometry_msgs/Point]): List of Point ROS objects to be projected to the camera plane. 

        Returns:
            list: List of 2-item tuples. Each tuple corresponds to the (u, v) pixel coordinates of one of the input `points` (in the same order).
        """
        # Get the intrinsic matrix parameters
        __, fx, fy, cx, cy = self._get_camera_matrix(cam_info_topic = '/xtion/rgb/camera_info')
        # Get the transform from the current 
        # NOTE: actually the 'from' and 'to' reference frames are hardcoded... Make it flexible!
        T = self._get_camera_orientation()
        # Initialize the list of centers in pixel coordinates
        centers_px = [None] * len(points)
        for i, point in enumerate(points):
            point = transforms.apply_transform_to_point(t = T.translation, q = T.rotation, p = point)
            centers_px[i] = self._project_3d_to_2d(point, fx, fy, cx, cy)
        return centers_px


    def _get_camera_matrix(self, cam_info_topic):
        """Wait for a message from the topic broadcasting the camera intrinsic matrix and unpack it.

        Args:
            cam_info_topic (string): The name of the ROS topic on which the camera intrinsic matrix is published.

        Returns:
            tuple: A tuple with 5 elements: 
                - item at index 0 is a `string` with the name of the camera reference frame; 
                - items at index 1 and 2 are the focal distance along the x and y direction (in px)
                - items at index 3 and 4 are the principal point offset in the x and y direction (in px)
        """
        # Populate necessary K matrix values for 3D pose computation.
        cam_info = rospy.wait_for_message(cam_info_topic, CameraInfo)
        frame_id = cam_info.header.frame_id
        fx = cam_info.K[0]
        fy = cam_info.K[4]
        cx = cam_info.K[2]
        cy = cam_info.K[5]
        return frame_id, fx, fy, cx, cy
    

    def _get_image(self, color_topic):
        """Wait for a message from the topic publishing images acquired by the camera and make it available as a OpenCV-compatible image.

        Args:
            color_topic (string): The name of the ROS topic on which camera images are published.

        Returns:
            np.ndarray: The first image received on the `color_topic`. Expected shape is (h, w, 3).
        """
        # Check whether the color topic is publishing Image or CompressedImage messages
        color_msg_type, __ = decoder.check_for_type(color_topic)
        if color_msg_type == Image:
            img_ros2cv = decoder.decode_Image_RGB
        elif color_msg_type == CompressedImage:
            img_ros2cv = decoder.decode_CompressedImage_RGB
        else:
            raise("Invalid message type on topic", color_topic)

        img_msg = rospy.wait_for_message(color_topic, color_msg_type, timeout = 5)
        return img_ros2cv(img_msg)

    
    def _project_3d_to_2d(self, p, fx, fy, cx = 0, cy = 0):
        """Project a Point object to the image plane (i.e. from Cartesian coordinates to pixel coordinates). 

        Args:
            p (geometry_msgs/Pose): The point to project to the camera plane.
            fx (int): Focal length in the horizontal direction. Expressed in [px].
            fy (int): Focal length in the vertical direction. Expressed in [px].
            cx (int, optional): Principal point offset in the horizontal direction. Expressed in [px]. Defaults to 0.
            cy (int, optional): Principal point offset in the vertical direction. Expressed in [px]. Defaults to 0.
        """
        x, y, z = [p.x, p.y, p.z]
        u = x * fx / z + cx
        v = y * fy / z + cy
        return abs(int(u)), abs(int(v))


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('pick_and_place_manager')
    pap_manager = PickAndPlaceManager()
    rospy.sleep(10)
    try:
        while not rospy.is_shutdown():
            pass
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down the pick and place manager')


    