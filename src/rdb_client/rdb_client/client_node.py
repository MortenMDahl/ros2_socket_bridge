#!/usr/bin/env python3

# Copyright 2021 Morten Melby Dahl.
# Copyright 2021 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse
from _thread import *
import sockets

# from tcpSocket import TCPSocket
#from udpSocket import UDPSocket

# ip_server = '127.0.0.1'
# port = 3000 #spesific to robot. A server has to be started for each robot.




class ClientNode(Node):
    def __init__(self, robot_name, robot_type, server_ip, server_port, port_list_data, port_list_commands):
        super().__init__('client_node')
        self.name = robot_name + '_client_node'
        #self.declare_parameter('port')
        self.port_list_data = self.get_parameter('port_list_data').value
        self.port_list_commands = self.get_parameter('port_list_commands').value
        # For testing
        self.server_port = 3000

        self.server_ip = str(self.get_parameter('server_ip').value)


        self.port_list_data = []
        self.port_list_commands = []
        self.thread_objects = []
        self.connection_list_cmd = []

        connection_response = None

        # Create initial message for setting up multiple sockets on the server
        init_msg = 'init:' + robot_name + ':' + robot_type + ':'
        for i in range(len(port_list_data)):
            init_msg.append(str(port_list_data[i]) + ';')
        init_msg = init_msg[:-1]
        init_msg.append(':')
        for j in range(len(port_list_commands)):
            init_msg.append(str(port_list_commands) + ';')
        init_msg = init_msg[:-1]

        # Create socket object and connect to server
        self.clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        print('Connecting to server...')
        try:
            clientSocket.connect((server_ip, server_port))
        except socket.error as e:
            print('Error: ' + str(e))

        # Send the init message and wait for response
        clientSocket.send(init_msg.encode('utf-8'))
        while connection_response = None:
            connection_response = clientSocket.recv(2048)
            connection_response.decode('utf-8')

            if connection_response == 'Matching init received.':
                if robot_type == 'mobile':
                    mobile_init()

                    # Create threads which takes incoming messages and publishes to correct topic
                    # Create callback which serializes and sends messages

                elif robot_type == 'manipulator':
                    manipulator_init()

                elif robot_type == 'multi':
                    mobile_init()
                    manipulator_init()

            # Start sending to correct ports
        #do stuff with incoming based on topics


    def mobile_init(self):
        self.laser = socket.socket() # Maybe change to a set socket object? Like TCP, UDP or other
        self.odom = socket.socket()
        self.goal_nav = socket.socket()
        self.init_nav = socket.socket()

        odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        laser_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # TODO: Create threads for incoming data
        init_nav_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        goal_nav_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

    def manipulator_init(self):
        joint_pos_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

        goal_manipulator_pose_pub = None # TODO
        # TODO: Create threads for incoming data and posting to correct subs


    def laser_callback(self):
        return -1
        #encode laser data
        #send to correct port

    def odom_callback(self):
        return -1

