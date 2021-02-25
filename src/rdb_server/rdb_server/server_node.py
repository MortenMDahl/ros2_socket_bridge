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
import socket
from _thread import *
import threading

import rdb_server.serializer
import rdb_server.thread_objects
import rdb_server.server_sockets

# from tcpSocket import TCPSocket
#from udpSocket import UDPSocket

# ip_server = '127.0.0.1'
# port = 3000 #spesific to robot. A server has to be started for each robot.


class ServerNode(Node):
    def __init__(self, robot_name, robot_type, server_port):
        super().__init__('server_node')
        self.robot_name = str(robot_name)
        self.name = self.robot_name + '_server_node'

        self.server_port = int(server_port or 0)

        # Get parameter from param.yaml
        self.declare_parameter('server_ip')
        self.server_ip = str(self.get_parameter('server_ip').value)

        print('Server IP: ', self.server_ip)
        print('Server port: ', self.server_port)


        self.port_list_data = []
        self.port_list_commands = []
        self.thread_objects = []
        self.connection_list_cmd = []


        # Makes socket object and waits for connection

        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)

        try:
            self.serverSocket.bind((self.server_ip, self.server_port))

        except socket.error as e:
            print('Error binding server socket: ' + str(e))

        print(str(self.name) + ' waiting for connection...')
        self.serverSocket.listen(5) # Enables the server to accept connections

        # Start thread which accept incoming connections
        # and handles received messages
        threading.Thread(target=self.server_handler).start()

    def server_handler(self):
        msg = None

        while rclpy.ok(): 
            self.server, address = self.serverSocket.accept() # Waits incoming connection
            print(str(self.name) + ': Data received from ' + address[0] + ':' + address[1])

            while msg == None:
                msg = self.server.recv(2048)
                data = msg.decode('utf-8').split(':')

            # Reset the msg for next cycle
            msg = None

            # Handle the received message
            server_msg_handler(data)


    def server_msg_handler(self, data):
        if data[0] == 'init':
            self.server.send('Connected to server.'.encode('utf-8'))
            print('Received init message.')
            if data[1] == robot_name:
                if data[2] == robot_type:

                    # Confirm with the client that the settings match
                    confirm_init_msg = 'Matching init received.'.encode('utf-8')
                    client.send(confirm_init_msg)

                    # Establish port lists and convert contents to integers
                    self.port_list_data = data[3].split(';')
                    self.port_list_data = list(map(int, self.port_list_data))

                    self.port_list_commands = data[4].split(';')
                    self.port_list_commands = list(map(int, self.port_list_commands))

                    if robot_type == 'mobile':

                        mobile_init(robot_name)
                        # Manually create thread objects
                        # TODO: Automate creation of thread objects.
                        self.thread_objects.append(ThreadObject('laser', self.port_list_data[0], self.laser_pub))
                        self.thread_objects.append(ThreadObject('odom', self.port_list_data[1], self.odom_pub))

                        for thread_object in thread_objects:
                            threading.Thread(target=self.data_stream_thread, args=(thread_object.name, thread_object.port, thread_object.publisher)).start()



                    elif robot_type == 'manipulator':

                        #self.thread_objects.append(ThreadObject('something', self.port_list_data[0], self.publisher_here))
                        manipulator_init()

                        for thread_object in thread_objects:
                            threading.Thread(target=self.data_stream_thread, args=(thread_object.name, thread_object.port, thread_object.publisher)).start()

                        # Create threads for receiving data
                    elif robot_type == 'multi':
                        mobile_init()
                        manipulator_init()
                        # Create threads for receiving data
                    else:
                        print('Error while initializing robot type: ' + robot_type)
                        # exit?

                        # Connect initialized sockets from **_init
                else:
                    print('Error: robot_type does not match with client.')
            else:
                print('Error: robot_name does not match with client.')
        else:
            # What to do if message is not init
            # "Disconnect" message?

            print(str(data))

        # Receive init message from robot
        # Function which starts threaded connections for topics
            # Same IP, port listed in init message

    def data_stream_thread(self, thread_name, port, pub):

        # Create a sensor_socket inside the socket object
        soc = Sockets.sensor_socket(self.ip, port, thread_name)

        connected = False
        print(thread_name + " thread started.\n Connecting... ")

        cli, addr = soc.connect_sensor_socket()
        print(thread_name + ' thread: Connection successful.')
        connected = True

        if thread_name == 'laser':
            while connected:
                data = cli.recv(2048) # Endre størrelse på buffer?
                data = data.decode('utf-8')

                laser_msg = Serializer.laser_deserialize(data)

                pub.publish(laser_msg)

        elif thread_name == 'odom':
            while connected:
                data = cli.recv(2048) # Endre størrelse på buffer?
                data = data.decode('utf-8')

                odom_msg = Serializer.odom_deserialize(data)

                pub.publish(odom_msg)

        elif thread_name == 'something_else':
            while connected:
                data = cli.recv(2048)
                # Mer her



    def mobile_init(self, robot_name):

        # Make callback functions
        #   -> Create sockets for sending messages
        #   -> execute callback functions

        command_topic_names = ['init_nav_pose', 'goal_nav_pose']
        for i in range(len(self.port_list_commands)):
            cmd_soc = Sockets.command_socket(self.ip, self.port_list_commands[i], command_topic_names[i])
            cmd_connection, cmd_client_address = cmd_soc.connect_sensor_socket()
            self.connection_list_cmd.append([cmd_soc, cmd_connection, cmd_client_address])

        self.init_nav_pose_sub = create_subscriber(PoseWithCovarianceStamped, robot_name + '/initialpose', self.init_nav_pose_callback, 10)
        self.goal_nav_pose_sub = create_subscriber(PoseStamped, robot_name + '/goal_pose', self.goal_nav_pose_callback, 10)

        # TODO: Create threads for incoming data
        # These has to be spesific to type of robot. Possibly create publishers when connections are made.
        self.odom_pub = create_publisher(Odometry, robot_name + '/odom', 10)
        self.laser_pub = create_publisher(LaserScan, robot_name + '/scan', 10)

    def manipulator_init(self, robot_name):
        # Skal nok være en publisher
        self.joint_pos_sub = create_subscription(JointState, 'joint_states', self.joint_callback, 10)

        # Skal nok være en subscriber
        self.goal_manipulator_pose_pub = None # TODO
        # TODO: Create threads for incoming data and posting to correct subs

    def init_nav_pose_callback(self, msg):
        serialized_msg = Serializer.pose_covar_stamped_serialize(msg)
        self.connection_list_cmd[0,1].send(serialized_msg)

    def goal_nav_pose_callback(self, msg):
        serialized_msg = Serializer.pose_stamped_serialize(msg)
        self.connection_list_cmd[1,1].send(serialized_msg)


def main(argv=sys.argv[1:]):
    # Get parameters from launch file
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-name', '--robot_name')
    parser.add_argument('-type', '--robot_type')
    parser.add_argument('-p', '--server_port')
    args = parser.parse_args(remove_ros_args(args=argv))

    # Initialize rclpy and create node object
    rclpy.init(args=argv)
    server_node = ServerNode(args.robot_name,args.robot_type,args.server_port)

    # Spin the node
    rclpy.spin(server_node)

    try:
        server_node.destroy_node()
        rclpy.shutdown()
    except:
        print('Error: ' + "rclpy shutdown failed")


if __name__ == '__main__':
    main()