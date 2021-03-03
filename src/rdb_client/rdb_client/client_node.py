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

import time
from rdb_client.serializer import *
from rdb_client.bridge_objects import *
# from tcpSocket import TCPSocket
#from udpSocket import UDPSocket

# ip_server = '127.0.0.1'
# port = 3000 #spesific to robot. A server has to be started for each robot.




class ClientNode(Node):
    def __init__(self, robot_name, robot_type):
        super().__init__('client_node')
        self.robot_name = str(robot_name)
        self.robot_type = str(robot_type)
        self.name = robot_name + '_client_node'

        self.receive_objects = []
        self.transmit_objects = []

        self.UDP_PROTOCOL = 'UDP'
        self.TCP_PROTOCOL = 'TCP'
        self.DIRECTION_RECEIVE = 'receive'
        self.DIRECTION_TRANSMIT = 'transmit'
        self.BUFFER_SIZE = 4096

        connected = False

        # Creates serialization object
        self.serializer = Serialization

        # Get parameters from config file
        self.declare_parameter('server_ip')
        self.server_ip = str(self.get_parameter('server_ip').value)
        self.declare_parameter('server_port')
        self.server_port = int(self.get_parameter('server_port').value or 0)

        self.declare_parameter('data_topics')
        self.data_topics = self.get_parameter('data_topics').value
        self.declare_parameter('data_msg_types')
        self.data_msg_types = self.get_parameter('data_msg_types').value
        self.declare_parameter('data_ports')
        self.data_ports = self.get_parameter('data_ports').value
        self.declare_parameter('data_protocols')
        self.data_protocols = self.get_parameter('data_protocols').value
        self.declare_parameter('data_qos')
        self.data_qos = []
        data_qos_temp = self.get_parameter('data_qos').value


        self.declare_parameter('command_topics')
        self.command_topics = self.get_parameter('command_topics').value
        self.declare_parameter('command_msg_types')
        self.command_msg_types = self.get_parameter('command_msg_types').value
        self.declare_parameter('command_ports')
        self.command_ports = self.get_parameter('command_ports').value
        self.declare_parameter('command_protocols')
        self.command_protocols = self.get_parameter('command_protocols').value
        self.declare_parameter('command_qos')
        self.command_qos = []
        command_qos_temp = self.get_parameter('command_qos').value


        if not (len(self.data_topics) == len(self.data_ports) == len(self.data_protocols)):
            raise Exception('Data topics not matching amount of ports or protocols assigned. Shutting down.')
            rclpy.shutdown()

        if not (len(self.command_topics) == len(self.command_ports) == len(self.command_protocols)):
            raise Exception('Command topics not matching amount of ports or protocols assigned. Shutting down.')
            rclpy.shutdown()


        self.thread_objects = []
        self.connection_list_cmd = []

        connection_response = None

        # Create initial message for setting up multiple sockets on the server
        init_msg = 'init:'+ str(robot_name) + ':'               # 0, 1

        for i in range(len(self.data_topics)):
            init_msg += str(self.data_topics[i]) + ';'
        init_msg = init_msg[:-1]                                # 2
        init_msg += ':'

        for j in range(len(self.data_msg_types)):
            init_msg += str(self.data_msg_types[j]) + ';'
        init_msg = init_msg[:-1]                                # 3
        init_msg += ':'    

        for k in range(len(self.data_ports)):
            init_msg += str(self.data_ports[k]) + ';'
        init_msg = init_msg[:-1]                                # 4
        init_msg += ':'

        for l in range(len(self.data_protocols)):
            init_msg += str(self.data_protocols[l]) + ';'
        init_msg = init_msg[:-1]                                # 5
        init_msg += ':'

        for m in range(len(data_qos_temp)):
            init_msg += str(data_qos_temp[m]) + ';'
        init_msg = init_msg[:-1]                                # 6
        init_msg += ':'

        for n in range(len(self.command_topics)):
            init_msg += str(self.command_topics[n]) + ';'
        init_msg = init_msg[:-1]                                # 7
        init_msg += ':'

        for o in range(len(self.command_msg_types)):
            init_msg += str(self.command_msg_types[o]) + ';'
        init_msg = init_msg[:-1]                                # 8
        init_msg += ':'

        for p in range(len(self.command_ports)):
            init_msg += str(self.command_ports[p]) + ';'
        init_msg = init_msg[:-1]                                # 9
        init_msg += ':'

        for q in range(len(self.command_protocols)):
            init_msg += str(self.command_protocols[q]) + ';'
        init_msg = init_msg[:-1]                                # 10
        init_msg += ':'

        for r in range(len(command_qos_temp)):
            init_msg += str(command_qos_temp[r]) + ';'
        init_msg = init_msg[:-1]                                # 11

        '''
        Change the qos from being a string into being either integer or class
        This is done after creating the init message, as not to confuse if an object is to be sent.
        Objects must be sent as strings.
        '''
        for qos in data_qos_temp:
                    try:
                        self.data_qos.append(int(qos))
                    except Exception:
                        self.data_qos.append(self.str_to_class(qos))

        for qos in command_qos_temp:
                    try:
                        self.command_qos.append(int(qos))
                    except Exception:
                        self.command_qos.append(self.str_to_class(qos))

        # Creates bridge objects later to be used for receiving and sending messages.
        for i in range(len(self.data_topics)):
            self.transmit_objects.append(BridgeObject(self.DIRECTION_TRANSMIT, self.data_topics[i], self.data_msg_types[i], self.data_ports[i], self.data_protocols[i], self.data_qos[i]))

        for j in range(len(self.command_topics)):
            self.receive_objects.append(BridgeObject(self.DIRECTION_RECEIVE, self.command_topics[j], self.command_msg_types[j], self.command_ports[j], self.command_protocols[j], self.command_qos[j]))

        # Create a TCP socket object and connect to server
        self.clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        self.clientSocket.settimeout(None)
        print('Connecting to server...')
        
        while not connected:
            try:
                self.clientSocket.connect_ex((self.server_ip, self.server_port))
                connected = True
            except socket.error as e:
                pass

        # Send the init message and wait for response
        print('Connected! Sending init message...')
        self.clientSocket.send(init_msg.encode('utf-8'))
        connection_response = self.clientSocket.recv(2048)

        connection_response = connection_response.decode('utf-8')

        while connection_response != None:
            if connection_response == 'Matching init received.':
                print('Matching init confirmed.')
                '''
                Sleeping to ensure that the server readies the ports for communication
                before attempting to connect.
                '''
                time.sleep(2)
                print('Establishing connections...')
                for obj in self.transmit_objects:
                    obj.address = (self.server_ip, obj.port)
                    if obj.protocol == self.UDP_PROTOCOL:
                        try:
                            obj.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                            obj.soc.settimeout(15)
                            obj.soc.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)
                        except Exception as e:
                            print("Error creating UDP socket for ", obj.name, ": ", e)

                    elif obj.protocol == self.TCP_PROTOCOL:
                        try:
                            obj.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            obj.soc.settimeout(15)
                            obj.soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
                            obj.soc.connect((self.server_ip, obj.port))
                        except Exception as e:
                            print("Error connecting ", obj.name, " to requested address: ", e)
                    # Creates a subscriber for each object with its appropriate callback function based on protocol.
                    obj.subscriber = self.create_subscription(self.str_to_class(obj.msg_type), obj.name, obj.callback, obj.qos)
                print('Transmission channels established!')

                for obj in self.receive_objects:
                    # Create publisher to correct topic
                    obj.publisher = self.create_publisher(self.str_to_class(obj.msg_type), obj.name, obj.qos)

                    if obj.protocol == self.UDP_PROTOCOL:
                        try:
                            obj.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                            obj.soc.settimeout(15)
                            obj.soc.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)
                        except Exception as e:
                            print("Error creating UDP socket for ", obj.name, ": ", e)
                    elif obj.protocol == self.TCP_PROTOCOL:
                        try:
                            obj.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                            obj.soc.settimeout(15)
                            obj.soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
                            obj.soc.connect((self.server_ip, obj.port))
                        except Exception as e:
                            print("Error connecting ", obj.name, " to requested address: ", e)

                    # Creates thread for handling incoming messages.
                    threading.Thread(target=self.receive_connection_thread, args = [obj]).start()
                print('Receiving connections established!')

            else:
                print(connection_response)

            connection_response = None

            # Start sending to correct ports
        #do stuff with incoming based on topics

    def receive_connection_thread(self, obj):
        if obj.protocol == self.UDP_PROTOCOL:
            while not obj.connected:
                try:
                    connection, client_address = obj.soc.recvfrom(self.BUFFER_SIZE)
                    obj.connected = True
                except Exception as e:
                    print(self.name, "- Error while connecting: ", e)

            print(str(obj.name) + " connected!")

            while obj.connected:
                data, addr = obj.soc.recvfrom(self.BUFFER_SIZE)
                serialized_msg = data.decode('utf-8')

                deserialized_msg = self.serializer.deserialize(self.str_to_class(obj.msg_type), serialized_msg)
                obj.publisher.publish(data)


        elif obj.protocol == self.TCP_PROTOCOL:
            while not obj.connected:
                obj.soc.connect((self.server_ip, obj.port))
                obj.connected = True

            while obj.connected:
                try:
                    data = obj.soc.recv(self.BUFFER_SIZE)
                    msg = self.serializer.deserialize(self.str_to_class(obj.msg_type), data)
                    obj.publisher.publish(msg)
                except socket.timeout:
                    continue


    def mobile_init(self):
        # TCP sockets - TODO: Automate this based on a list?
        self.init_nav = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.init_nav.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        time.sleep(1) # Waiting due to server being slightly slower while initializing

        print('Connecting init_nav')
        self.init_nav.connect((self.server_ip, self.command_ports[0]))

        self.goal_nav = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.goal_nav.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        time.sleep(1) # Waiting due to server being slightly slower while initializing

        print('Connecting nav_goal')
        self.goal_nav.connect((self.server_ip, self.command_ports[1]))

        # UDP sockets
        print('Connecting laser')
        self.laser = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.laser.settimeout(0.1)
        self.laser.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)

        print('Connecting odom')
        self.odom = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.odom.settimeout(0.1)
        self.odom.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)

        

        odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        laser_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)

        # TODO: Create threads for incoming data
        init_nav_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        goal_nav_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

    def manipulator_init(self):
        joint_pos_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

        goal_manipulator_pose_pub = None # TODO
        # TODO: Create threads for incoming data and posting to correct subs


    def scan_callback(self, data):
        scan_serialized = self.serializer.serialize_laser(data)
        self.laser.sendto(scan_serialized.encode('utf-8'), (self.server_ip, self.data_ports[0]))

    def odom_callback(self, data):
        odom_serialized = self.serializer.serialize_odom(data)
        self.odom.sendto(odom_serialized.encode('utf-8'), (self.server_ip, self.data_ports[1]))

    def str_to_class(self, classname):
        return getattr(sys.modules[__name__], classname)


def main(argv=sys.argv[1:]):
    # Get parameters from launch file
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-name', '--robot_name')
    parser.add_argument('-type', '--robot_type')
    args = parser.parse_args(remove_ros_args(args=argv))

    # Initialize rclpy and create node object
    rclpy.init(args=argv)
    client_node = ClientNode(args.robot_name,args.robot_type)

    # Spin the node
    rclpy.spin(client_node)

    try:
        client_node.destroy_node()
        rclpy.shutdown()
    except:
        print('Error: ' + "rclpy shutdown failed")


if __name__ == '__main__':
    main()