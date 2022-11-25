import numpy as np
import math

from geometry_msgs.msg import Point, Quaternion
# TODO: make it work with different kinds of messages (Vector3, PointStamped...)


def translation_ros2np(msg):
    vector = np.array([msg.x, msg.y, msg.z])
    M = np.identity(4)
    M[:3, 3] = vector
    return M


def rotation_ros2np(msg):
    quaternion = np.array([msg.x, msg.y, msg.z, msg.w])
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    '''
    if nq < _EPS:
        return np.identity(4)
    '''
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64) 


def transform_ros2np(msg):
    return np.dot(translation_ros2np(msg.translation), rotation_ros2np(msg.rotation))

def conjugate(q):
    # return the conjugate of the given quaternion ()
    return Quaternion(-q.x, -q.y, -q.z, q.w)

def multiply(q1, q2):
    # multiply the two given quaternions
    w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z
    z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x
    return Quaternion(x, y, z, w)

def apply_rotation_to_point(q, p):
    # apply the rotation specified by the quaternion to a point (Point)
    pre_multiply = multiply(q, Quaternion(p.x, p.y, p.z, 0))
    post_multiply = multiply(pre_multiply, conjugate(q))
    return Point(post_multiply.x, post_multiply.y, post_multiply.z)

def apply_translation_to_point(t, p):
    # apply the rotation specified by the vector to a point
    return(Point(p.x + t.x, p.y + t.y, p.z + t.z))

def apply_transform_to_point(t, q, p):
    # TF convention in ROS is to represent transforms as a translation followed by a rotation
    #return apply_rotation_to_point(q, apply_translation_to_point(t, p))
    # ... but it does not seem to be true (?!). I am definetly misunderstanding something.
    return apply_translation_to_point(t, apply_rotation_to_point(q, p))

