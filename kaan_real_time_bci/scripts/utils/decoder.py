"""ROS to OpenCV image converter

This script can be imported as a module and contains the following public methods:

* `check_for_type`
* `decode_CompressedImage_depth`
* `decode_Image_depth`
* `decode_Image_RGB`
* `decode_CompressedImage_RGB`

"""

import cv2
import struct
import numpy as np

import rospy
from sensor_msgs.msg import Image, CompressedImage

# NOTE: requires OpenCV >= 3.0 (as the one I have locally installed is), with older versions,
#       cv2.IMREAD_UNCHANGED and cv2.IMREAD_COLOR should be adapted

def check_for_type(topic):
    """ Check wheter a topic is publishing Image or CompressedImage messages.

    Args:
        topic (string): The name of the topic to check.

    Returns:
        A tuple containing the message type and the image encoding.
    """
    try:    # assume the topic is publishing Image messages
        # NOTE: the timeout could be exposed as an argument
        encoding = rospy.wait_for_message(topic, Image, timeout = 5).encoding
        msg_type = Image
    except rospy.ROSException as e:
        rospy.logerr('No Image or CompressedImage can be received on topic' + topic)
    try:    # check if the topic is publishing CompressedImage messages instead
        encoding, __ = rospy.wait_for_message(topic, 
                                              CompressedImage, 
                                              timeout = 5
                                              ).format.split(';')
        encoding = encoding.strip()
        msg_type = CompressedImage
    except rospy.ROSException:
        # TODO: avoid entering the CompressedImage condition if the image is actually an Image. Otherwise, Image objects are returned correctly, but the error message below is shown.
        rospy.logerr('No Image or CompressedImage can be received on topic' + topic)
    
    return msg_type, encoding


def decode_CompressedImage_depth(msg, header_size = 12, depth_in_m = True):
    """Convert a ROS CompressedImage message with depth information into a numpy storing the depth information in the specified measurement unit.
    From https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/

    Args:
        msg (sensor_msgs.CompressedImage): The ROS CompressedImage message to be converted.
        header_size (int, optional): The size of the message header in bytes. Defaults to 12.
        depth_in_m (bool, optional): If `True`, depth will be expressed in m, otherwise in `mm`. Defaults to `True`.

    Raises:
        Exception: Raised if the compression type (extracted from the `format` field of the message) does not have the expected `compressedDepth` value.
        Exception: Raised if the compressed image cannot be decoded. This problem maybe due to a wrong `header_size` value. The user is prompted to try to adjust that value.
        Exception: Raised if the decoding from the specified datatype is not implemented.

    Returns:
        A tuple with depth data as a `np.ndarray` and the original depth format as a string.
    """
    # 'msg' as type CompressedImage
    depth_fmt, compr_type = msg.format.split(';')
    # Remove white space
    depth_fmt = depth_fmt.strip()
    compr_type = compr_type.strip()
    if compr_type != "compressedDepth":
        raise Exception("Compression type is not 'compressedDepth'."
                        "You probably subscribed to the wrong topic.")

    # Remove header from raw data
    raw_data = msg.data[header_size:]

    # TODO: try frombuffer instead of fromstring
    depth_img_raw = cv2.imdecode(np.fromstring(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
    if depth_img_raw is None:   # probably wrong header
        raise Exception("Could not decode compressed depth image."
                        "You may need to change 'header_size'.")

    if depth_fmt == "16UC1":    # data in mm as uint16
        # Convert data to m if requested by the user (dividing should automatically turn depth data
        # into float)
        depth_data = depth_img_raw / 1000 if depth_in_m else depth_img_raw
    elif depth_fmt == "32FC1":
        raw_header = msg.data[:header_size]
        # header: int, float, float
        [compfmt, depthQuantA, depthQuantB] = struct.unpack('iff', raw_header)
        depth_data = depthQuantA / (depth_img_raw.astype(np.float32)-depthQuantB)
        # filter max values
        depth_data[depth_img_raw == 0] = 0

        # depth_data provides distance in meters as f32, convert to mm if request by the user
        depth_data = depth_data if depth_in_m else (depth_data * 1000).astype(np.uint16)
        # reshape needed here?
    else:
        raise Exception("Decoding of '" + depth_fmt + "' is not implemented!")

    # Data are uint16 if in mm, in float32 if in m. Also the encoding is returned.
    return  depth_data, depth_fmt


def decode_Image_depth(msg, depth_in_m = True):
    """Convert a ROS Image message with depth information into a numpy storing the depth information in the specified measurement unit.

    Args:
        msg (sensor_msgs.Image): The ROS Image message to be converted.
        depth_in_m (bool, optional): If `True`, depth will be expressed in m, otherwise in mm. Defaults to `True`.

    Returns:
        A tuple with depth data as a `np.ndarray` and the original depth format as a string.
    """
    if msg.encoding == "16UC1":
        byte_array = np.fromstring(msg.data, np.uint16)      # array of bytes
        depth_data = np.frombuffer(byte_array, dtype = np.uint16)  # numpy 2D array
        depth_data = (depth_data / 1000) if depth_in_m else depth_data
    elif msg.encoding == "32FC1":
        byte_array = np.fromstring(msg.data, np.float32)      # array of bytes
        depth_data = np.frombuffer(byte_array, dtype = np.float32)  # numpy 2D array
        depth_data = depth_data if depth_in_m else (depth_data * 1000).astype(np.uint16)
    
    depth_data = np.reshape(depth_data,
                            newshape = (msg.height, msg.width)
                            )
    # Data are uint16 if in mm, in float32 if in m. Also the encoding is returned.
    return  depth_data, msg.encoding


def decode_Image_RGB(msg):
    """Convert a ROS Image message into an OpenCV image.

    Args:
        msg (sensor_msgs.Image): The ROS message to convert.

    Returns:
        An OpenCV image object.
    """
    byte_array = np.frombuffer(msg.data, np.uint8)
    return np.reshape(byte_array, newshape = (msg.height, msg.width, 3))


def decode_CompressedImage_RGB(msg):
    """Convert a ROS Image message into an OpenCV image.

    Args:
        msg (sensor_msgs.CompressedImage): The ROS message to convert.

    Returns:
        An OpenCV image object.
    """
    byte_array = np.frombuffer(msg.data, np.uint8)
    image = cv2.imdecode(byte_array, cv2.IMREAD_COLOR)
    return image