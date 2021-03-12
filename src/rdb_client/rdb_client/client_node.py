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
import pickle
import cryptography
from cryptography.fernet import Fernet

from rdb_client.bridge_objects import *
#from rdb_client.msg import * # Imports user-made message types


class ClientNode(Node):
	def __init__(self, robot_name, encryption_key, use_name):
		super().__init__('client_node')
		self.robot_name = str(robot_name)
		self.name = robot_name + '_client_node'

		if lower(use_name) == 'true':
			self.use_name = True
		else:
			self.use_name = False

		self.fernet = Fernet(encryption_key)
		self.key = encryption_key

		self.receive_objects = []
		self.transmit_objects = []

		self.UDP_PROTOCOL = 'UDP'
		self.TCP_PROTOCOL = 'TCP'
		self.DIRECTION_RECEIVE = 'receive'
		self.DIRECTION_TRANSMIT = 'transmit'
		self.BUFFER_SIZE = 4096

		connected = False

		# Get parameters from config file
		self.declare_parameter('server_ip')
		self.server_ip = str(self.get_parameter('server_ip').value)
		self.declare_parameter('server_port')
		self.server_port = int(self.get_parameter('server_port').value or 0)

		self.declare_parameter('transmit_topics')
		self.transmit_topics = self.get_parameter('transmit_topics').value
		self.declare_parameter('transmit_msg_types')
		self.transmit_msg_types = self.get_parameter('transmit_msg_types').value
		self.declare_parameter('transmit_ports')
		self.transmit_ports = self.get_parameter('transmit_ports').value
		self.declare_parameter('transmit_protocols')
		self.transmit_protocols = self.get_parameter('transmit_protocols').value
		self.declare_parameter('transmit_qos')
		self.transmit_qos = []
		transmit_qos_temp = self.get_parameter('transmit_qos').value


		self.declare_parameter('receive_topics')
		self.receive_topics = self.get_parameter('receive_topics').value
		self.declare_parameter('receive_msg_types')
		self.receive_msg_types = self.get_parameter('receive_msg_types').value
		self.declare_parameter('receive_ports')
		self.receive_ports = self.get_parameter('receive_ports').value
		self.declare_parameter('receive_protocols')
		self.receive_protocols = self.get_parameter('receive_protocols').value
		self.declare_parameter('receive_qos')
		self.receive_qos = []
		receive_qos_temp = self.get_parameter('receive_qos').value


		# Checks to see if the user has given the right amount of settings.
		if not (len(self.transmit_topics) == len(self.transmit_ports) == len(self.transmit_protocols)):
			raise Exception('transmit topics not matching amount of ports or protocols assigned. Shutting down.')
			rclpy.shutdown()

		if not (len(self.receive_topics) == len(self.receive_ports) == len(self.receive_protocols)):
			raise Exception('receive topics not matching amount of ports or protocols assigned. Shutting down.')
			rclpy.shutdown()


		self.thread_objects = []
		self.connection_list_cmd = []

		connection_response = None

		# Create initial message for setting up multiple sockets on the server
		init_msg = 'init:'+ str(robot_name) + ':'               # 0, 1

		for i in range(len(self.transmit_topics)):
			init_msg += str(self.transmit_topics[i]) + ';'
		init_msg = init_msg[:-1]                                # 2
		init_msg += ':'

		for j in range(len(self.transmit_msg_types)):
			init_msg += str(self.transmit_msg_types[j]) + ';'
		init_msg = init_msg[:-1]                                # 3
		init_msg += ':'    

		for k in range(len(self.transmit_ports)):
			init_msg += str(self.transmit_ports[k]) + ';'
		init_msg = init_msg[:-1]                                # 4
		init_msg += ':'

		for l in range(len(self.transmit_protocols)):
			init_msg += str(self.transmit_protocols[l]) + ';'
		init_msg = init_msg[:-1]                                # 5
		init_msg += ':'

		for m in range(len(transmit_qos_temp)):
			init_msg += str(transmit_qos_temp[m]) + ';'
		init_msg = init_msg[:-1]                                # 6
		init_msg += ':'

		for n in range(len(self.receive_topics)):
			init_msg += str(self.receive_topics[n]) + ';'
		init_msg = init_msg[:-1]                                # 7
		init_msg += ':'

		for o in range(len(self.receive_msg_types)):
			init_msg += str(self.receive_msg_types[o]) + ';'
		init_msg = init_msg[:-1]                                # 8
		init_msg += ':'

		for p in range(len(self.receive_ports)):
			init_msg += str(self.receive_ports[p]) + ';'
		init_msg = init_msg[:-1]                                # 9
		init_msg += ':'

		for q in range(len(self.receive_protocols)):
			init_msg += str(self.receive_protocols[q]) + ';'
		init_msg = init_msg[:-1]                                # 10
		init_msg += ':'

		for r in range(len(receive_qos_temp)):
			init_msg += str(receive_qos_temp[r]) + ';'
		init_msg = init_msg[:-1]                                # 11


		'''
		Change the qos from being a string into being either integer or class.
		The str_to_class converts any string to a class, if the class is defined.
		'''
		for qos in transmit_qos_temp:
					try:
						self.transmit_qos.append(int(qos))
					except Exception:
						self.transmit_qos.append(self.str_to_class(qos))

		for qos in receive_qos_temp:
					try:
						self.receive_qos.append(int(qos))
					except Exception:
						self.receive_qos.append(self.str_to_class(qos))

		# Creates bridge objects later to be used for receiving and sending messages.
		for i in range(len(self.transmit_topics)):
			self.transmit_objects.append(BridgeObject(self.DIRECTION_TRANSMIT, self.key, self.transmit_topics[i], self.transmit_msg_types[i], self.transmit_ports[i], self.transmit_protocols[i], self.transmit_qos[i]))

		for j in range(len(self.receive_topics)):
			self.receive_objects.append(BridgeObject(self.DIRECTION_RECEIVE, self.key, self.receive_topics[j], self.receive_msg_types[j], self.receive_ports[j], self.receive_protocols[j], self.receive_qos[j]))

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
							obj.soc.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576) # Øke buffer størrelse?
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
					if self.use_name:
						obj.publisher = self.create_publisher(self.str_to_class(obj.msg_type), self.robot_name + '/' + obj.name, obj.qos)
					else:
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


	def receive_connection_thread(self, obj):
		if obj.protocol == self.UDP_PROTOCOL:
			while not obj.connected:
				try:
					connection, client_address = obj.soc.recvfrom(self.BUFFER_SIZE)
					obj.connected = True
				except socket.timeout:
					continue
				except Exception as e:
					print(self.name, "- Error while connecting: ", e)

			print(str(obj.name) + " connected!")

			while obj.connected:
				data_encrypted, addr = obj.soc.recvfrom(self.BUFFER_SIZE)
				try:
					data = self.fernet.decrypt(data_encrypted)
					msg = pickle.loads(data)
					obj.publisher.publish(msg)
				except cryptography.fernet.InvalidToken:
					print('Received message with invalid tolken!')


		elif obj.protocol == self.TCP_PROTOCOL:
			while not obj.connected:
				obj.soc.connect((self.server_ip, obj.port))
				obj.connected = True

			while obj.connected:
				try:
					data_encrypted = obj.soc.recv(self.BUFFER_SIZE)
					data = self.fernet.decrypt(data_encrypted)
					msg = pickle.loads(data)
					obj.publisher.publish(msg)
				except socket.timeout:
					continue
				except cryptography.fernet.InvalidToken:
					print('Received message with invalid tolken!')

	def str_to_class(self, classname):
		return getattr(sys.modules[__name__], classname)


def main(argv=sys.argv[1:]):
	# Get parameters from launch file
	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('-name', '--robot_name')
	parser.add_argument('-key', '--encryption_key')
	parser.add_argument('-usename', '--use_name')
	args = parser.parse_args(remove_ros_args(args=argv))

	# Initialize rclpy and create node object
	rclpy.init(args=argv)
	client_node = ClientNode(args.robot_name, args.encryption_key, args.use_name)

	# Spin the node
	rclpy.spin(client_node)

	try:
		client_node.destroy_node()
		rclpy.shutdown()
	except:
		print('Error: ' + "rclpy shutdown failed")


if __name__ == '__main__':
	main()