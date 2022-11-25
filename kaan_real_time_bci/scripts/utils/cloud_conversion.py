"""ROS/Open3D pointcloud converter

This script can be imported as a module and contains the following public methods:

* `pointcloud_ros_to_open3d`
    * `pointcloud_ros_to_numpy`
    * `pointcloud_numpy_to_open3d`
* `pointcloud_open3d_to_ros`
    * `pointcloud_open3d_to_numpy`
    * `pointcloud_numpy_to_open3d`
"""

import numpy as np
import open3d as o3d

import ros_numpy
from sensor_msgs.msg import PointCloud2


def pointcloud_ros_to_open3d(msg):
    """Convert a ROS PointCloud2 message into a Open3D PointCloud object.

    Args:
        msg (sensor_msgs.PointCloud2): A ROS PointCloud2 message.

    Returns:
        open3d.geometry.PointCloud: An Open3D PointCloud object with the same data of the input message.
    """
    xyz, rgb, ros_cloud_shape = pointcloud_ros_to_numpy(msg)
    pcd = pointcloud_numpy_to_open3d(xyz, rgb)

    return pcd, ros_cloud_shape


def pointcloud_ros_to_numpy(msg):
    """Convert a ROS PointCloud2 message into two np.ndarrays (one for XYZ coordinates, and one for RGB values).
    From: https://answers.ros.org/question/321829/color-problems-extracting-rgb-from-pointcloud2/

    Args:
        msg (sensor_msgs.PointCloud2): A ROS PointCloud2 message.

    Returns:
        A tuple containing a first `np.ndarray` (XYZ coordinates of the points in the cloud), a second `np.ndarray` (RGB values), and a tuple (`(int, int)` shape of the input cloud). The second item (RGB values) can be None if the input PointCloud2 has no color information. 
    """
    pc = ros_numpy.numpify(msg)
    shape = pc.shape + (3, )    # add a dimension to store XYZ and RGB info
    
    xyz = np.zeros(shape) 
    xyz[..., 0] = pc['x']
    xyz[..., 1] = pc['y']
    xyz[..., 2] = pc['z']

    rgb = None
    if 'rgb' in pc.dtype.fields:
        # If the cloud has an RGB field, split it to obtain 3 int values from a single float value.
        pc = ros_numpy.point_cloud2.split_rgb_field(pc)

        rgb = np.zeros(shape)
        rgb[..., 0] = pc['r']
        rgb[..., 1] = pc['g']
        rgb[..., 2] = pc['b']
    
    return xyz, rgb, shape[:-1]


def pointcloud_numpy_to_open3d(xyz, rgb = None):
    """Convert two `np.ndarrays` (one with cartesian data and one with RGB values) into an Open3D pointcloud object.

    Args:
        xyz (np.ndarray): An array of shape `(..., 3)` with the cartesian coordinates of the points in the cloud. The order of the point values is assumed to match the one in `rgb`.
        rgb (np.ndarray, optional): An array of shape `(..., 3)` with the RGB values of the points in the cloud. The order of the point values is assumed to match the one in `xyz`.

    Returns:
        open3d.geometry.PointCloud: An Open3D PointCloud object with the same geometry data as `xyz` and, if any, the same color information as `rgb`.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(_format_ndarray_for_o3d(xyz))
    if rgb is not None:
        rgb = (rgb - rgb.min()) / (rgb.max() - rgb.min()) # normalize RGB values in the [0 1] range
        pcd.colors = o3d.utility.Vector3dVector(_format_ndarray_for_o3d(rgb))

    return pcd


def pointcloud_open3d_to_ros(cloud, shape = None, merge_rgb = True):
    """Convert an Open3D PointCloud object into a ROS PointCloud2 message.

    Args:
        cloud (open3d.geometry.PointCloud): An Open3D PointCloud object.
        shape ((int, int), optional): A tuple with two elements, specifiying the 2D structure of the point cloud. Defaults to None.
        merge_rgb (bool, optional): If True, the `np.uint8` R, G and B values are merged into a single `np.float32` value. Defaults to True.

    Returns:
        sensor_msgs.PointCloud2: A ROS PointCloud2 message, with the same information of the input cloud.
    """
    xyz, rgb = pointcloud_open3d_to_numpy(cloud, shape)
    msg = pointcloud_numpy_to_ros(xyz, rgb, merge_rgb = merge_rgb)
    return msg


def pointcloud_open3d_to_numpy(cloud, shape):
    """Convert an Open3D PointCloud object into two `np.ndarrays` (one with cartesian data and one with RGB values) into an Open3D pointcloud object.

    Args:
        cloud (open3d.geometry.PointCloud): An Open3D PointCloud object.
        shape (tuple, optional): A tuple with two `int` elements, specifiying the 2D structure of the point cloud. Defaults to None.

    Raises:
        ValueError: If the height and width value given with `shape` do not match the number of points in the cloud.

    Returns:
        A tuple containing a first `np.ndarray` (XYZ coordinates of the points in the cloud), and a second `nd.array` (RGB values). If the input cloud has no objects, the second item will be an array of zeros of proper shape.
    """
    # Extract the cartesian coordinates of the points in the cloud object
    xyz = np.asarray(cloud.points)
    # Extract the RGB info (if any) of the points in the cloud object
    if cloud.colors: 
        # If the input cloud has color information, format it to be saved in the ROS cloud
        rgb = np.asarray(cloud.colors)
        rgb = (rgb * 255.0).astype(np.uint32)
    else:
        # If the input cloud does not have color info, paint it black.
        # NOTE: this is a workaround to keep the same structure for the `data` array below (i.e. to always have `r`, `g`, and `b` fields).
        rgb = np.zeros((xyz.shape[0], 3), dtype = np.uint32)
    # If a `shape` argument is given, check for the validity and reshape data.
    if shape is not None:
        h, w = shape    # unpack the shape tuple
        if not h * w == xyz.shape[0]:
            raise ValueError('Specified shape argument ' + str(shape) + ' would give ' + str(h * w) + 'points. This value does not match the number of points in the cloud ' + str(xyz.shape[0]))
        xyz = np.reshape(xyz, newshape = shape + (3,))
        rgb = np.reshape(rgb, newshape = shape + (3,))
    else:
        shape = xyz.shape[0]
    
    return xyz, rgb


def pointcloud_numpy_to_ros(xyz, rgb = None, merge_rgb = True):
    """Convert an Open3D PointCloud object into a ROS PointCloud2 message.

    Args:
        xyz (np.ndarray): An array of shape (..., 3) with the cartesian coordinates of the points in the cloud. The order of the point values is assumed to match the one in `rgb`.
        rgb (np.ndarray, optional): An array of shape (..., 3) with the RGB values of the points in the cloud. The order of the point values is assumed to match the one in `xyz`.
        merge_rgb (bool, optional): If `True`, the `r`, `g` and `b` (type `np.uint8`) fields are merged into a single `rgb` (type `np.float32`) fields. Defaults to `True`.

    Returns:
        sensor_msgs.PointCloud2: A ROS message with the same data of the input Open3D cloud.
    """
    # Create a structured array (according with the `merge_rgb` argument)
    dtypes = [('x',    np.float32),
              ('y',    np.float32),
              ('z',    np.float32),
            ]
    dtype_rgb = [('rgb', np.float32)] if merge_rgb else [('r',    np.uint8),
                                                         ('g',    np.uint8),
                                                         ('b',    np.uint8)
                                                        ]
    dtypes.extend(dtype_rgb)
    
    if rgb is None:
        # If no color is given in input (i.e. if this function is used alone and not called from `pointcloud_open3d_to_ros`), create an array of proper shape and fill it with zero (i.e. the output cloud will be black).
        rgb = np.zeros_like(xyz, uint8)
    if not xyz.shape == rgb.shape:
        raise ValueError('The shapes of the `xyz` ' + str(xyz.shape) + ' and `rgb` ' + str(rgb.shape) + ' arguments do not match.')
    data = np.zeros(xyz.shape[:-1],             # 1D or 2D 
                    dtype = dtypes
                    )
    # Fill in the fields with the point coordinates
    data['x'] = xyz[..., 0]
    data['y'] = xyz[..., 1]
    data['z'] = xyz[..., 2]
    # Fill in the fields with the RGB information.
    r = rgb[..., 0]
    g = rgb[..., 1]
    b = rgb[..., 2]
    if merge_rgb:
        '''
        NOTE: the following two lines are taken from `ros_numpy.point_cloud2.merge_rgb_fields`
        '''
        rgb = np.array((r << 16) | (g << 8) | (b << 0), dtype = np.uint32)
        rgb.dtype = np.float32
        data['rgb'] = rgb
    else:
        data['r'] = r.astype(np.uint8) 
        data['g'] = g.astype(np.uint8)
        data['b'] = b.astype(np.uint8)
    
    return ros_numpy.msgify(PointCloud2, data)


def _format_ndarray_for_o3d(arr):
    """Check if the input array is structured (i.e. points are organized in a 2D matrix) or not. Structured arrays are left untouched, structured ones are flattened in order to match the (n, 3) shape required from Open3D PointCloud fields.

    Args:
        arr (np.ndarray): The array to check and, possibly, flatten.

    Raises:
        ValueError: The shape of the input array is not (n, 3) or (h, w, 3).

    Returns:
        np.ndarray: The input array, reshaped to match (n, 3) shape.
    """
    # Remove any axes of lenght 1 (i.e. dummy dimensions).
    arr = np.squeeze(arr)
    # Check if cloud shape is suitable for the intended use
    if (not arr.shape[-1] == 3) or (not len(arr.shape) <= 3):
        raise ValueError('The input array is expected to be a numpy array with shape (n, 3) or (h, w, 3). After squeezing, the input array has shape ' + str(arr.shape))
    
    # Reshape the array as (n, 3). If the input array already has shape (n, 3), it remains unchanged.
    arr = np.reshape(arr, newshape = (-1, 3))

    return arr