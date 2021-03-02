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

import os
import time
import rclpy
import socket
import _thread as thread
import threading

class Sockets:

    def __init__(self, ip, port, socket_name):

        # Declaring different variables used for setting up the socket
        self.BUFFER_SIZE = 4096
        self.connected = False
        self.name = socket_name
        self.ip = ip
        self.port = int(port or 0)


    def sensor_socket(self):
        print("Creating sensor socket: " + str(self.name) + ". IP: " + str(self.ip) + ". Port: "+ str(self.port) + ".")

        try:
            self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sensor_socket.settimeout(15)
            self.sensor_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1048576)

        except Exception as e:
            print("Error binding ", self.name, " to requested address: ", e)

        return self.sensor_socket

    def command_socket(self):
        print("Creating command socket: " + str(self.name) + ". IP: " + str(self.ip) + ". Port: "+ str(self.port) + ".")

        try:
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        except Exception as e:
           print("Error binding ", self.name, " to requested address: ", e)
        return self.command_socket

    def connect_sensor_socket(self):
        while(not self.connected):
            if self.sensor_socket != None:
                try:
                    # Receives a single message over UDP to confirm the connection. 
                    connection, client_address = self.sensor_socket.recvfrom(self.BUFFER_SIZE)
                    self.connected = True
                except Exception as e:
                    print(self.name, "- Error while connecting: ", e)
        return connection, client_address

    def connect_command_socket(self):
        self.command_socket.listen(3)
        while not self.connected:
            try:
                # Sets the port to accept incoming connections
                self.cmd_connection, self.client_address = self.command_socket.accept()
                self.cmd_connection.settimeout(10.0)
                self.connected = True
            except:
                print('Could not connect ' + self.name)
        return self.cmd_connection, self.client_address

    def send_command(self, cmd):
        self.cmd_connection.sendall(cmd.encode('utf-8'))

    def command_shutdown(self):
        print("Shutdown command received, shutting down command socket.")
        self.cmd_connection.shutdown(socket.SHUT_RDWR)
        self.cmd_connection.close()
        self.command_socket.close()
        self.connected = False