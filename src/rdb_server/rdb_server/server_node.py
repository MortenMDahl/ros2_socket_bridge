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
from std_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from rclpy.qos import *
from rclpy.utilities import remove_ros_args
import argparse
import socket
from _thread import *
import threading

import time

from rdb_server.serializer import *
from rdb_server.bridge_objects import *
from rdb_server.server_sockets import *

# from tcpSocket import TCPSocket
#from udpSocket import UDPSocket

# ip_server = '127.0.0.1'
# port = 3000 #spesific to robot. A server has to be started for each robot.


class ServerNode(Node):
	def __init__(self, robot_name, robot_type, server_port):
		super().__init__('server_node')
		self.robot_name = str(robot_name)
		self.robot_type = str(robot_type)

		self.name = self.robot_name + '_server_node'

		self.server_port = int(server_port or 0)

		# Get parameter from bringup.yaml
		self.declare_parameter('server_ip')
		self.server_ip = str(self.get_parameter('server_ip').value)


		self.data_ports = []
		self.command_ports = []
		self.receive_objects = []
		self.transmit_objects = []
		self.command_connection_list = []
		self.publisher_list = []
		self.serializer = Serialization

		self.UDP_PROTOCOL = 'UDP'
		self.TCP_PROTOCOL = 'TCP'
		self.DIRECTION_RECEIVE = 'receive'
		self.DIRECTION_TRANSMIT = 'transmit'
		self.BUFFER_SIZE = 4096

		# Makes socket object and waits for connection

		self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)


		try:
			self.serverSocket.bind((self.server_ip, self.server_port))

		except socket.error as e:
			print('Error binding server socket: ' + str(e))

		print(str(self.name) + ' waiting for connection at port ' + str(self.server_port) + '...')
		self.serverSocket.listen(5) # Enables the server to accept connections

		# Start thread which accept incoming connections
		# and handles received messages
		threading.Thread(target=self.server_handler).start()

	def server_handler(self):
		msg = None

		while rclpy.ok(): 
			try:

				self.server, address = self.serverSocket.accept() # Waits incoming connection
				print(self.name + ': Data received from ' + str(address[0]) + ':' + str(address[1]))

				while msg == None:
					msg = self.server.recv(2048)
					data = msg.decode('utf-8').split(':')

				# Reset the msg for next cycle
				msg = None

				# Handle the received message
				self.server_msg_handler(data)
			except socket.timeout:
				continue

	def receive_connection_thread(self, obj):
		if obj.protocol == self.UDP_PROTOCOL:
			while not obj.connected:
				try:
					connection, client_address = obj.soc.recvfrom(self.BUFFER_SIZE)
					obj.connected = True
				except socket.timeout:
					continue
				except Exception as e:
					print(obj.name, "- Error while connecting: ", e)

			print(str(obj.name) + " connected!")

			while obj.connected:
				data, addr = obj.soc.recvfrom(self.BUFFER_SIZE)
				serialized_msg = data.decode('utf-8')
				deserialized_msg = self.serializer.deserialize(self.str_to_class(obj.msg_type), serialized_msg)
				obj.publisher.publish(deserialized_msg)


		elif obj.protocol == self.TCP_PROTOCOL:
			while not obj.connected:
				obj.connection, obj.address = obj.soc.accept()
				obj.connected = True

			while obj.connected:
				data = obj.connection.recv(self.BUFFER_SIZE)
				data = data.decode('utf-8')
				msg = self.serializer.deserialize(obj.msg_type, data)
				obj.publisher.publish(msg)



	def server_msg_handler(self, data):
		if data[0] == 'init':
			print('Received init message.')
			if data[1] == self.robot_name:

				# Confirm with the client that the settings match
				confirm_init_msg = 'Matching init received.'.encode('utf-8')
				print('Matching init. Confirming with client.')
				self.server.send(confirm_init_msg)

				# Establish port lists and convert contents to integers
				self.data_topics = list(data[2].split(';'))
				self.data_msg_types = list(data[3].split(';'))

				self.data_ports = list(data[4].split(';'))
				self.data_ports = [int(port) for port in self.data_ports]

				self.data_protocols = list(data[5].split(';'))
				self.data_qos = []

				data_qos_temp = list(data[6].split(';'))

				for qos in data_qos_temp:
					try:
						self.data_qos.append(int(qos))
					except Exception:
						self.data_qos.append(self.str_to_class(qos))

				self.command_topics = list(data[7].split(';'))
				self.command_msg_types = list(data[8].split(';'))

				self.command_ports = list(data[9].split(';'))
				self.command_ports = [int(port) for port in self.command_ports]

				self.command_protocols = list(data[10].split(';'))
				self.command_qos = []

				command_qos_temp = list(data[11].split(';'))
				
				for qos in command_qos_temp:
					try:
						self.command_qos.append(int(qos))
					except Exception:
						self.command_qos.append(self.str_to_class(qos))


				# Takes the received data and assigns them to objects
				for i in range(len(self.data_topics)):
					self.receive_objects.append(BridgeObject(self.DIRECTION_RECEIVE, self.data_topics[i], self.data_msg_types[i], self.data_ports[i], self.data_protocols[i], self.data_qos[i]))

				for j in range(len(self.command_topics)):
					self.transmit_objects.append(BridgeObject(self.DIRECTION_TRANSMIT, self.command_topics[j], self.command_msg_types[j], self.command_ports[j], self.command_protocols[j], self.command_qos[j]))

				# Prepairing receive_objects to be used for communication
				# If the robot name is empty, create a publisher to the requested topic without robot namespace
				for obj in self.receive_objects:
					if self.robot_name != '':
						obj.publisher = self.create_publisher(self.str_to_class(obj.msg_type), self.robot_name + '/' + obj.name, obj.qos)
					else:
						obj.publisher = self.create_publisher(self.str_to_class(obj.msg_type), obj.name, obj.qos)
					

					if obj.protocol == self.UDP_PROTOCOL:
						# Creates an UDP socket
						try:
							obj.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
							obj.soc.settimeout(15)
							obj.soc.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)
							obj.soc.bind((self.server_ip, int(obj.port)))
						except Exception as e:
							print("Error binding ", obj.name, " to requested address: ", e)

					elif obj.protocol == self.TCP_PROTOCOL:
						# Creates a TCP socket
						try:
							obj.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
							obj.soc.settimeout(15)
							obj.soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
							obj.soc.bind((self.server_ip, int(obj.port)))
							obj.soc.listen(3)
						except Exception as e:
							print("Error binding ", obj.name, " to requested address: ", e)

					# Creates thread for connecting the sockets and handling incoming data based on protocol.
					# sende inn kun connection eller socket basert på TCP eller UDP?
					threading.Thread(target=self.receive_connection_thread, args = [obj]).start()

				# Creating subscriptions with callback functions for transmit_objects.
				for obj in self.transmit_objects:
					if obj.protocol == self.UDP_PROTOCOL:
						# Creates an UDP socket
						try:
							obj.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
							obj.soc.settimeout(15)
							obj.soc.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)
							obj.soc.bind((self.server_ip, int(obj.port)))
						except Exception as e:
							print("Error binding ", obj.name, " to requested address: ", e)

					elif obj.protocol == self.TCP_PROTOCOL:
						# Creates a TCP socket
						try:
							obj.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
							obj.soc.settimeout(15)
							obj.soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
							obj.soc.bind((self.server_ip, int(obj.port)))
							obj.soc.listen(3)
						except Exception as e:
							print("Error binding ", obj.name, " to requested address: ", e)

					obj.subscriber = self.create_subscription(self.str_to_class(obj.msg_type), obj.name, obj.callback, obj.qos)

			else:
				print('Error: robot_name does not match with client.')
		else:
			# What to do if message is not init
			# "Disconnect" message?

			print(str(data))


	def data_stream_thread(self, thread_name, port, pub, protocol):
		# Create a socket object
		soc = Sockets(self.server_ip, port, thread_name)
		# Create a sensor_socket inside the socket object and bind it.
		sensor_soc = soc.sensor_socket()

		# Bind the socket
		address = (self.server_ip, int(port))
		sensor_soc.bind(address)

		connected = False
		print(thread_name + " thread started.\nConnecting... ")
		cli, addr = soc.connect_sensor_socket()

		print(thread_name + ' thread: Connection successful.')
		connected = True

		if thread_name == 'laser':
			while connected:
				msg, addr = sensor_soc.recvfrom(2048) # Endre størrelse på buffer?
				msg = msg.decode('utf-8')	
				laser_msg = self.serializer.laser_deserialize(msg)

				pub.publish(laser_msg)

		elif thread_name == 'odom':
			while connected:
				msg, addr = sensor_soc.recvfrom(2048) # Endre størrelse på buffer?

				msg = msg.decode('utf-8')

				odom_msg = self.serializer.odom_deserialize(msg)

				pub.publish(odom_msg)

		elif thread_name == 'something_else':
			while connected:
				data = cli.recvfrom(2048)
				# Mer her

	def robot_init(self, robot_info):
		return -1


	def mobile_init(self, robot_name):
		for name in self.data_topics:
			self.publisher_list.append()
		# Disse burde være i en param liste eller bli mottatt av roboten.


		command_topic_names = ['init_nav_pose', 'goal_nav_pose']
		for i in range(len(self.command_ports)):
			soc = Sockets(self.server_ip, int(self.command_ports[i]), command_topic_names[i])
			cmd_soc = soc.command_socket()

			# Bind the socket
			address = (self.server_ip, int(self.command_ports[i]))
			cmd_soc.bind(address)

			cmd_connection, cmd_client_address = soc.connect_command_socket()
			self.command_connection_list.append([cmd_soc, cmd_connection, cmd_client_address])

		self.init_nav_pose_sub = self.create_subscription(PoseWithCovarianceStamped, robot_name + '/initialpose', self.init_nav_pose_callback, 10)
		self.goal_nav_pose_sub = self.create_subscription(PoseStamped, robot_name + '/goal_pose', self.goal_nav_pose_callback, 10)

		# TODO: Create threads for incoming data
		# These has to be spesific to type of robot. Possibly create publishers when connections are made.
		self.odom_pub = self.create_publisher(Odometry, robot_name + '/odom', 10)
		self.laser_pub = self.create_publisher(LaserScan, robot_name + '/scan', 10)

	def manipulator_init(self, robot_name):
		# Skal nok være en publisher
		self.joint_pos_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

		# Skal nok være en subscriber
		self.goal_manipulator_pose_pub = None # TODO
		# TODO: Create threads for incoming data and posting to correct subs

	def init_nav_pose_callback(self, msg):
		serialized_msg = Serializer.pose_covar_stamped_serialize(msg)
		self.command_connection_list[0,1].send(serialized_msg)

	def goal_nav_pose_callback(self, msg):
		serialized_msg = Serializer.pose_stamped_serialize(msg)
		self.command_connection_list[1,1].send(serialized_msg)

	def str_to_class(self, classname):
		return getattr(sys.modules[__name__], classname)


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