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
import os
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry


class Serialization:
	def serialize_header(data):
		out = str(data.header.stamp.sec) + ';'
		out += str(data.header.stamp.nanosec) + ';'
		out += str(data.header.frame_id) + ':'
		return out


	def serialize_laser(data):
		out = str(data.header.stamp.sec) + ';'
		out += str(data.header.stamp.nanosec) + ';'
		out += str(data.header.frame_id) + ':'

		out += str(data.angle_min) + ';'
		out += str(data.angle_max) + ';'
		out += str(data.angle_increment) + ';'
		out += str(data.time_increment) + ';'
		out += str(data.scan_time) + ';'
		out += str(data.range_min) + ';'
		out += str(data.range_max) + ':'

		for i in range(len(data.ranges)):
			out += str(data.ranges[i]) + ';'
		out = out[:-1]
		out += ':'

		for j in range(len(data.intensities)):
			out += str(data.intensities[j]) + ';'
		out = out[:-1]

		return out



	def deserialize_laser(data):
		lasermsg = LaserScan()
		data = data.split(':')

		headerdata = data[0].split(';')
		lasermsg.header.stamp.sec = int(headerdata[0])
		lasermsg.header.stamp.nanosec = int(headerdata[1])
		lasermsg.header.frame_id = headerdata[2]

		laserdata = data[1].split(';')
		lasermsg.angle_min = float(laserdata[0])
		lasermsg.angle_max = float(laserdata[1])
		lasermsg.angle_increment = float(laserdata[2])
		lasermsg.time_increment = float(laserdata[3])
		lasermsg.scan_time = float(laserdata[4])
		lasermsg.range_min = float(laserdata[5])
		lasermsg.range_max = float(laserdata[6])

		rangedata = data[2].split(';')
		lasermsg.ranges = [float(rangevar or 'inf') for rangevar in rangedata]
		if len(data) > 3:
			intensitydata = data[3].split(';')
			lasermsg.intensities = [float(intensity or 'inf') for intensity in intensitydata]
		else:
			lasermsg.intensities = []

		return lasermsg



	def serialize_odom(data):
		out = str(data.header.stamp.sec) + ';'
		out += str(data.header.stamp.nanosec) + ';'
		out += str(data.header.frame_id) + ':'

		out += str(data.child_frame_id) + ':'

		out += str(data.pose.pose.position.x) + ';' + str(data.pose.pose.position.y) + ';' + str(data.pose.pose.position.z) + ':'
		
		out += str(data.pose.pose.orientation.x) + ';' + str(data.pose.pose.orientation.y) + ';' + str(data.pose.pose.orientation.z) + ';' + str(data.pose.pose.orientation.w) + ':'
		
		for i in range(len(data.pose.covariance)):
			out += str(data.pose.covariance[i]) + ';'
		out = out[:-1]
		out += ':'

		out += str(data.twist.twist.linear.x) + ';' + str(data.twist.twist.linear.y) + ';' + str(data.twist.twist.linear.z) + ':'
		out += str(data.twist.twist.angular.x) + ';' + str(data.twist.twist.angular.y) + ';' + str(data.twist.twist.angular.z) + ':'

		for j in range(len(data.twist.covariance)):
			out += str(data.twist.covariance[j]) + ';'
		out = out[:-1]

		return out



	def deserialize_odom(data):
		odommsg = Odometry()
		data = data.split(':')

		headerdata = data[0].split(';')
		odommsg.header.stamp.sec = int(headerdata[0])
		odommsg.header.stamp.nanosec = int(headerdata[1])
		odommsg.header.frame_id = headerdata[2]

		odommsg.child_frame_id = data[1]

		posedata = data[2].split(';')
		odommsg.pose.pose.position.x = float(posedata[0])
		odommsg.pose.pose.position.y = float(posedata[1])
		odommsg.pose.pose.position.z = float(posedata[2])

		orientdata = data[3].split(';')
		odommsg.pose.pose.orientation.x = float(orientdata[0])
		odommsg.pose.pose.orientation.y = float(orientdata[1])
		odommsg.pose.pose.orientation.z = float(orientdata[2])
		odommsg.pose.pose.orientation.w = float(orientdata[3])

		covar_posedata = data[4].split(';')

		odommsg.pose.covariance = [float(covar_data or 'inf') for covar_data in covar_posedata]

		twistdata_linear = data[5].split(';')
		odommsg.twist.twist.linear.x = float(twistdata_linear[0])
		odommsg.twist.twist.linear.y = float(twistdata_linear[1])
		odommsg.twist.twist.linear.z = float(twistdata_linear[2])

		twistdata_angular = data[6].split(';')
		odommsg.twist.twist.angular.x = float(twistdata_angular[0])
		odommsg.twist.twist.angular.y = float(twistdata_angular[1])
		odommsg.twist.twist.angular.z = float(twistdata_angular[2])

		covar_twistdata = data[7].split(';')
		odommsg.twist.covariance = [float(covar_data or 'inf') for covar_data in covar_twistdata]
		return odommsg

	# For use in /initialpose
	def pose_covar_stamped_serialize(data):
		out = str(data.header.stamp.sec) + ';'
		out += str(data.header.stamp.nanosec) + ';'
		out += str(data.header.frame_id) + ':'

		out += str(data.pose.pose.position.x) + ';' + str(data.pose.pose.position.y) + ';' + str(data.pose.pose.position.z) + ':'
		
		out += str(data.pose.pose.orientation.x) + ';' + str(data.pose.pose.orientation.y) + ';' + str(data.pose.pose.orientation.z) + ';' + str(data.pose.pose.orientation.w) + ':'
		
		for i in range(len(data.pose.covariance)):
			out += str(data.pose.covariance[i]) + ';'
		out = out[:-1]
		return out

	def pose_covar_stamped_deserialize(data):
		posemsg = PoseWithCovarianceStamped()
		data = data.split(':')

		headerdata = data[0].split(';')
		posemsg.header.stamp.sec = int(headerdata[0])
		posemsg.header.stamp.nanosec = int(headerdata[1])
		posemsg.header.frame_id = headerdata[2]

		posemsg.child_frame_id = data[1]

		posedata = data[2].split(';')
		posemsg.pose.pose.position.x = float(posedata[0])
		posemsg.pose.pose.position.y = float(posedata[1])
		posemsg.pose.pose.position.z = float(posedata[2])

		orientdata = data[3].split(';')
		posemsg.pose.pose.orientation.x = float(orientdata[0])
		posemsg.pose.pose.orientation.y = float(orientdata[1])
		posemsg.pose.pose.orientation.z = float(orientdata[2])
		posemsg.pose.pose.orientation.w = float(orientdata[3])

		covar = data[4].split(';')
		posemsg.pose.covariance = [float(covar_data or 'inf') for covar_data in covar]
		return posemsg




	def pose_stamped_serialize(data):
		out = self.serialize_header(data)

		out += str(data.pose.pose.position.x) + ';' + str(data.pose.pose.position.y) + ';' + str(data.pose.pose.position.z) + ':'
		out += str(data.pose.pose.orientation.x) + ';' + str(data.pose.pose.orientation.y) + ';' + str(data.pose.pose.orientation.z) + ';' + str(data.pose.pose.orientation.w)
		return out
	def pose_stamped_deserialize(data):
		posemsg = PoseStamped()
		data = data.split(';')

		headerdata = data[0].split(';')
		posemsg.header.stamp.sec = int(headerdata[0])
		posemsg.header.stamp.nanosec = int(headerdata[1])
		posemsg.header.frame_id = headerdata[2]

		posedata = data[1].split(';')
		posemsg.pose.pose.position.x = float(posedata[0])
		posemsg.pose.pose.position.y = float(posedata[1])
		posemsg.pose.pose.position.z = float(posedata[2])

		orientdata = data[2].split(';')
		posemsg.pose.pose.orientation.x = float(orientdata[0])
		posemsg.pose.pose.orientation.y = float(orientdata[1])
		posemsg.pose.pose.orientation.z = float(orientdata[2])
		posemsg.pose.pose.orientation.w = float(orientdata[3])

		return posemsg


	def joint_pos_serialize(data):
		return -1
		#TODO

	def joint_pos_deserialize(data):
		return -1
		#TODO

	def deserialize(msg_type, data):
		if msg_type == LaserScan:
			return Serialization.deserialize_laser(data)

		elif msg_type == Odometry:
			return Serialization.deserialize_odom(data)

		elif msg_type == PoseWithCovarianceStamped:
			return Serialization.pose_covar_stamped_deserialize(data)

		elif msg_type == PoseStamped:
			return Serialization.pose_stamped_deserialize(data)

		elif msg_type == JointState:
			return Serialization.joint_pos_deserialize(data)

	def serialize(msg_type, data):
		if msg_type == LaserScan:
			return Serialization.serialize_laser(data)

		elif msg_type == Odometry:
			return Serialization.serialize_odom(data)

		elif msg_type == PoseWithCovarianceStamped:
			return Serialization.pose_covar_stamped_serialize(data)

		elif msg_type == PoseStamped:
			return Serialization.pose_stamped_serialize(data)

		elif msg_type == JointState:
			return Serialization.joint_pos_serialize(data)