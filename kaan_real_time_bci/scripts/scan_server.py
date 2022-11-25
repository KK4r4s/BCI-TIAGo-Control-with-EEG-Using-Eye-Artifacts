#! /usr/bin/env python3

'''
NOTE FOR KAAN (in case I forget): the above line is called a `shebang` and it points to the Python interpreter that will be used to run this script. It point to virtual environment that I have locally on my PC. In you case, you can install the requirements system-wide (it should be just Open3D) for Python3, then change the shebang to `#!/usr/bin/env python3`. 
'''

# Python libs
import os
import sys
import numpy as np
import open3d as o3d
import copy
import matplotlib.pyplot as plt

# ROS libs
import rospy
from utils import cloud_conversion
from utils import transforms

# ROS messages
from sensor_msgs.msg import PointCloud2
#from sensor_msgs import point_cloud2
from geometry_msgs.msg import Transform, Point, Quaternion, PoseStamped
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kaan_real_time_bci.srv import ClustersPose, ClustersPoseResponse, ClustersPointcloud, ClustersPointcloudResponse


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
    
    def lift_torso(self):
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.30]
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


class TableTopSegmentation:
    def __init__(self, topic, n_objects):
        # Initialize an empty cloud
        self.pcd = o3d.geometry.PointCloud()
        
        # The n_objects clouds with more points will be picked
        self.n_objects = n_objects  # TODO: allow this to be set to None 
        # Save the name of the topic from which the pointcloud will be read
        self.topic = topic  # TODO: define a custom `srv` receiving a String to specifying the topic when calling the service
        
        # Define a server to display the point cloud upon user request
        rospy.Service('tabletop_segmentation/show_pointcloud', Empty, self.show_pointcloud_srv)
        # Define a service to acquire a point cloud and extract objects on a plane (if any)
        rospy.Service('tabletop_segmentation/get_objects', ClustersPointcloud, self.get_objects_srv)
        # Define a service to get the poses of the objects above the table (calls `get_objects` internally) for off-the-shelf usage of `tabletop_grasp`.
        rospy.Service('tabletop_segmentation/get_poses', ClustersPose, self.get_poses_srv)

        rospy.loginfo('Done. The tabletop segmentation node is operating.')


    def get_objects(self, cloud):
        # Use a RANSAC algorithm to extract the main plane (assumed to be the tabletop)
        plane, non_plane = self.plane_segmentation(cloud)
        # Get the most recent transform from camera frame to planning frame (assumed to be z-up)
        trans = self._get_camera_orientation()
        # Rotate the cloud to make it z-up to remove the points that are below the plane from the `non_plane` cloud. TODO: implement the same behavior without transforming the whole cloud (but keep in mind the downstream feature extraction!).
        if trans is not None:
            trans = transforms.transform_ros2np(trans)
            # Transform the plane cloud to the new reference frame and get its center
            plane = plane.transform(trans)
            plane_center = plane.get_center()
            # NOTE: only the plane center actually has to be transformed. The whole cloud is transformed here to visualize the result.

            # Transform the non-plane cloud to the new reference frame
            non_plane = non_plane.transform(trans)
            # Remove all the points below the plane height from the non-plane cloud
            above_plane_idx = np.nonzero(np.asarray(non_plane.points)[:, -1] > plane_center[-1])[0]
            non_plane_above = non_plane.select_by_index(above_plane_idx.tolist())
        # Clusterize the remaining above-plane points
        objects =  self._get_clusters(non_plane_above)
        ''' Uncomment to visualize the result of the clusterization process
        # Visualize the output
        self.geometries = copy.deepcopy(objects)
        self.geometries += [plane]
        self.show_pointcloud(None)  # as show_pointcloud is meant to be a Service callback, it requires an argument, hence a dummy one is passed when called as a function.
        '''
        return objects, plane

    def get_objects_srv(self, request):
        """TODO: remove need for tf; define custom service to remove object poses.

        Args:
            request (_type_): _description_
        """
        # Wait for a cloud on the selected topic and keep the points within a certain distance range
        cloud = self._get_cloud(topic = self.topic, max_dist = 3.0) # TODO: expose `max_dist`
        # Get the clouds of the objects above the plane and of the plane itself
        objects, plane = self.get_objects(cloud)
        # Fill in the response message        
        response = ClustersPointcloudResponse()
        response.objects_cloud = [cloud_conversion.pointcloud_open3d_to_ros(obj) for obj in objects]
        response.objects_cloud[:].header.frame_id = 'base_footprint'    # TODO: set as param
        response.plane_cloud = cloud_conversion.pointcloud_open3d_to_numpy(plane)
        response.plane_cloud[:].header.frame_id = 'base_footprint'    # TODO: set as param

        return response


    def get_poses(self, cloud):
        # Get the poses of the objects, for `tabletop_grasp.py` compatibility
        objects, plane = self.get_objects(cloud)    # set request to None as calling as method and not as service
        # TODO: improve the "fitting" (currently there is no fitting at all). Height is fixed, diameter and center estimate are already extracted... Just refine the process until going below an error threshold or reaching a number of iterations. Even better, given the initial guess, try to fit a cylinder and a parallelepiped (square base) and iterate only the best one.
        features = {}
        for i, obj in enumerate(objects):
            points = np.asarray(obj.points)
            # Get an estimate of the object radius (the objects is assumed to have cylindrical shape)
            radius = np.max(points[:, 1]) - np.min(points[:, 1]) / 2
            #print(radius)
            # Get an estimate of the object centroid
            ''' cause a quite huge error in the x direction (i.e. axis exiting from camera)
            centroid = np.median(points, axis = [0])
            centroid[0] += radius
            '''
            centroid = obj.get_center()
            #print(centroid)
            if centroid[0] > 0.8:
                continue
            features['obj' + str(i)] = (list(centroid), radius)
        # Get the centroid of the plane
        features['plane'] = plane.get_center()

        return features
        # TODO: define a srv with empty request and returning a list of PoseStamped `objects_pose` and a PoseStamped `plane_pose`.

    
    def get_poses_srv(self, request):
        """TODO: remove need for tf; define custom service to remove object poses.

        Args:
            request (_type_): _description_
        """
        # Wait for a cloud on the selected topic and keep the points within a certain distance range
        cloud = self._get_cloud(topic = self.topic, max_dist = 3.0)
        # Get the objects pose (to date, the centroid of the objects)
        features = self.get_poses(cloud)
        
        frame = 'base_footprint'  # TODO: set as param
        response = ClustersPoseResponse()   # initialize the response message
        response.plane_pose.pose.position = Point(features['plane'][0],
                                               features['plane'][1],
                                               features['plane'][2]
                                              )
        response.plane_pose.pose.orientation = Quaternion(0.0, -0.707, 0.0, 0.707) # probably not used...
        response.plane_pose.header.frame_id = frame

        for key in features:
            if key.startswith('obj'):
                pose = PoseStamped()
                pose.header.frame_id = 'base_footprint'
                pose.pose.position.x = features[key][0][0]
                pose.pose.position.y = features[key][0][1]
                pose.pose.position.z = features[key][0][2]
                pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
                response.objects_pose.append(pose)

        return response
    

    def plane_segmentation(self, cloud):
        plane_model, inliers = cloud.segment_plane(distance_threshold = 0.01, 
                                                   ransac_n = 3, 
                                                   num_iterations = 100
                                                   )
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        # Paint the plane pointcloud in red
        inlier_cloud = cloud.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([0, 0, 0])
        # Define a pointcloud with the non-plane points only
        outlier_cloud = cloud.select_by_index(inliers, invert = True)

        return inlier_cloud, outlier_cloud


    def show_pointcloud(self, geometries):
        # Draw the current pcd
        o3d.visualization.draw_geometries(self.geometries,
                                           zoom = 1.0,
                                           front = [0.0, 0.0, -1.0],
                                           lookat = [0.0, 0.0, 1.0],
                                           up = [0.0, -1.0, 0.0])

    def show_pointcloud_srv(self, request):
        # Draw the current pcd
        self.show_pointcloud(self.geometries)
    

    def _get_camera_orientation(self): #, cam_frame, to_frame = 'odom'):
        # TODO: as `tf` is giving problems, use a hardcoded transform instead, must be fixed
        T = Transform()
        ''' `rosrun tf tf_echo base_footprint xtion_rgb_optical_frame`'''
        T.translation.x = 0.245
        T.translation.y = 0.022
        T.translation.z = 1.301
        T.rotation.x = -0.626
        T.rotation.y = 0.619
        T.rotation.z = -0.329
        T.rotation.w = 0.329
        
        return T


    def _get_cloud(self, topic, max_dist = 5.0):
        rospy.loginfo('Waiting for a PointCloud2 on topic ' + topic)
        msg = rospy.wait_for_message(topic, PointCloud2)
        
        # Convert the ROS message into a Open3D PointCloud object
        cloud, __ = cloud_conversion.pointcloud_ros_to_open3d(msg)
        too_far_idx = np.nonzero(np.sum(np.asarray(cloud.points)**2, axis = 1) > max_dist)[0]
        cloud = cloud.select_by_index(too_far_idx.tolist(), invert = True)
        return cloud


    def _get_clusters(self, cloud):
        # Use the DBSCAN algorithm to get the pointclouds of the objects on the plane
        labels = np.array(cloud.cluster_dbscan(eps = 1e-2, 
                                                min_points = 50, 
                                                print_progress = False)
                                            )   # TODO: expose the DBSCAN params (optional)
        rospy.loginfo("Point cloud has " + str(labels.max() + 1) + " clusters")
        # Paint each cluster with a different color for visualization
        colors = plt.get_cmap("tab20")(labels / (labels.max() if labels.max() > 0 else 1))
        colors[labels < 0] = 0
        cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
        # Sort the labels based on their number of occurences
        unique, counts = np.unique(labels, return_counts = True)
        sorted_labels = sorted(zip(unique, counts), 
                               key = lambda x: x[1], 
                               reverse = True
                               )
        # Pick the n_objects cloud with more points
        indexes = np.arange(len(cloud.points))    # ugly and non-Pythonic
        clusters = [cloud.select_by_index(indexes[labels == label])
                   for label, count in sorted_labels if not label == -1 #[:self.n_objects]
                   ]
        return clusters

        
if __name__ == '__main__':
    rospy.init_node('tabletop_segmentation', anonymous = True)
    ''' Someone else has to take care of orienting the camera!!
    # Move the robot's torso and head to adjust the camera
    mv_cam = MoveCamera()
    mv_cam.lift_torso()
    mv_cam.lower_head()
    rospy.sleep(3)  # wait for the movements to finish and the point cloud to stabilize
    '''
    # Start the point cloud processing pipeline
    ttop_seg = TableTopSegmentation(n_objects = 4, topic = '/xtion/depth_registered/points')
    
    try:
        while not rospy.is_shutdown():
            pass
    except KeyboardInterrupt:
        print("Shutting down the table top segmentation module.")
    