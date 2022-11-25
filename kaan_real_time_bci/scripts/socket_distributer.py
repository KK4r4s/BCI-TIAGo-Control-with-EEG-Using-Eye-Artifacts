#! /usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Implementation of a socket server wrapped in a ROS publisher. In this base version, the ROS node publishes the data as a string message without any processing. For more complex behaviors, create a child class, and modify the callback.
'''
import math
#import copy
import numpy as np

# ROS libraries
import rospy

# ROS messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry

import socket
import json
# 
class ROSSocketServer(object):
    def __init__(self, host = "127.0.0.1", port = 65432, bufsize = 2048):
        """ Create an object with a socket server. The purpose of this node is to exchange data with a non-ROS process running the socket client. Data are assumed to be formatted as JSON strings.

        Args:
            host (str, optional): The server IP address. Defaults to "127.0.0.1", the standard loopback interface address (localhost).
            port (int, optional): Port to listen on (non-privileged ports are > 1023). Defaults to 65432.
            bufsize (int, optional): The maximum amount of data to be received (in bytes). From socket official documentation `For the best match with hardware and network realities, the value of bufsize should be a relatively small power of 2, e.g. 4096.`
        """
        self.host    = host     
        self.port    = port
        self.bufsize = bufsize

        # Define a publisher to broadcast the data received through the socket to all the nodes in the ROS network.
        self.sp_move_robot = rospy.Publisher('/socket_distributer_move_robot/data', String, queue_size= 10)
        
        self.sp_nurse = rospy.Publisher('/socket_distributer_nurse/data', String, queue_size= 10)

        self.sp_table = rospy.Publisher('/socket_distributer_table/data', String, queue_size= 10)

        self.sp_scan = rospy.Publisher('/socket_distributer_scan/data', String, queue_size= 10)

        self.sp_dance = rospy.Publisher('/socket_distributer_dance/data', String, queue_size= 10)
        
        self.sp_pick = rospy.Publisher('/socket_distributer_pick/data', String, queue_size= 10)    

        # Define socket to establish communication with python scripts
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        '''
        Set the socket options to allow re-using the address right after a Ctrl+C interruption of the node (from https://stackoverflow.com/questions/4465959/python-errno-98-address-already-in-use)
        '''

        self.s.settimeout(None) # None: Waiting for a client forever...
        print(self.s.gettimeout())

        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((self.host, self.port))
        self.s.listen() # Enable the server to accept connections
        rospy.loginfo('Waiting for connection...')
        # Blocks the execution and waits for an incoming connection, whenever a new client connects to the server, a new socket object is returned, i.e. a tuple with the connection and the address of the client.
        
        self.conn, addr = self.s.accept()   
        # rospy.loginfo("Connected by", addr)
            
    def loop(self):
        data = self.conn.recv(self.bufsize)
        if not data:
            rospy.logwarn('The server has received an empty byte object. That means that the client has closed the connection.')
            raise ConnectionError
        self.callback(json.loads(data.decode('utf-8')))
        

    def callback(self, data):
        """ Function defining how to process the data received through the socket.

        Args:
            data (_type_): data received through the socket, already converted back to Python objects.
        """
        print(data)

        page, new_data = data.split("_")

        if page=="1": # Move TiaGo page 
            move_type = new_data
            self.sp_move_robot.publish(move_type) # move_type can be Right, Left or Forward
            
        elif page=="2":

            pred_task=new_data
            
            if pred_task=="nurse":
                self.sp_nurse.publish(pred_task)
            elif pred_task=="table":
                self.sp_table.publish(pred_task)
            elif pred_task == "scan":
                self.sp_scan.publish(pred_task)
            elif pred_task == "dance":
                self.sp_dance.publish(pred_task)
        elif page=="3":
            req_task=new_data # Right/info, Left/info, Pick/info
            self.sp_pick.publish(req_task)

    def close_connection(self):
        # Close the socket if the node is stopped.
        self.s.close()
        rospy.loginfo('Closing the connection.')

if __name__ == '__main__':
    HOST = "127.0.0.1"  # Standard loopback interface address (localhost). When using the loopback interface (IPv4 127.0.0.1 or IPv6 ::1), data never leaves the host or touches the external network.
    PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

    rospy.init_node('socket_server', anonymous = True)
    server = ROSSocketServer()
    rospy.on_shutdown(server.close_connection)
    try:
        while not rospy.is_shutdown():
            server.loop()
    except ConnectionError or KeyboardInterrupt:
        rospy.loginfo("Shutting down the server node.")