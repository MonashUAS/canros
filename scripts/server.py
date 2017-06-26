#!/usr/bin/python

from __future__ import print_function
from Queue import Queue
import logging
import sys
import re

import canros
import uavcan
import rospy

# Must be set for the functions in this script to work.
uavcan_node = None

class Base(object):
	def __init__(self, uavcan_type):
		self.__uavcan_type = uavcan_type
		super(Base, self).__init__(self.UAVCAN_Type.full_name)
		self.ROS_Subscribe()
		self.UAVCAN_Subscribe()

	@property
	def UAVCAN_Type(self):
		return self.__uavcan_type

	# Should be overriden
	def ROS_Subscribe(self):
		raise NotImplementedError()
	def UAVCAN_Subscribe(self):
		raise NotImplementedError()

class Message(Base, canros.Message):
	@property
	def ROS_Publisher(self):
		try:
			return self.__ros_publisher
		except AttributeError:
			self.__ros_publisher = self.Publisher(queue_size=10)
			return self.__ros_publisher

	def ROS_Subscribe(self):
		def handler(event):
			if event._connection_header["callerid"] == rospy.get_name():
				# The message came from canros so ignore
				return
			uavcan_msg = copy_ros_uavcan(self.UAVCAN_Type(), event)
			uavcan_node.broadcast(uavcan_msg, priority=uavcan.TRANSFER_PRIORITY_LOWEST)
		self.Subscriber(handler)

	def UAVCAN_Subscribe(self):
		def handler(event):
			ros_msg = copy_uavcan_ros(self.Type(), event.message)
			if self.HasIdFeild:
				setattr(ros_msg, canros.uavcan_id_field_name, event.transfer.source_node_id)
			self.ROS_Publisher.publish(ros_msg)
		uavcan_node.add_handler(self.UAVCAN_Type, handler)

class Service(Base, canros.Service):
	# This is the canros server so the service naming scheme is the reverse of the canros API.
	@property
	def Request_Name(self): return super(Service, self).Response_Name
	@property
	def Response_Name(self): return super(Service, self).Request_Name
	@property
	def Request_Type(self): return super(Service, self).Response_Type
	@property
	def Response_Type(self): return super(Service, self).Request_Type
	@property
	def Request_Topic(self): return super(Service, self).Response_Topic
	@property
	def Response_Topic(self): return super(Service, self).Request_Topic

	@property
	def ROS_ServiceProxy(self):
		try:
			return self.__ros_service_proxy
		except AttributeError:
			self.__ros_service_proxy = self.ServiceProxy()
			return self.__ros_service_proxy

	def ROS_Subscribe(self):
		def handler(event):
			uavcan_req = copy_ros_uavcan(self.UAVCAN_Type.Request(), event, request=True)

			q = Queue()
			def callback(event):
				q.put(event.response if event else None)

			uavcan_id = getattr(event, canros.uavcan_id_field_name)
			if uavcan_id == 0:
				return
			uavcan_node.request(uavcan_req, uavcan_id, callback, timeout=1)   # Default UAVCAN service timeout is 1 second

			uavcan_resp = q.get()
			if uavcan_resp == None:
				return
			return copy_uavcan_ros(self.Response_Type(), uavcan_resp, request=False)
		self.Service(handler)

	def UAVCAN_Subscribe(self):
		def handler(event):
			ros_req = copy_uavcan_ros(self.Request_Type(), event.request, request=True)
			setattr(ros_req, canros.uavcan_id_field_name, event.transfer.source_node_id)
			try:
				ros_resp = self.ROS_ServiceProxy(ros_req)
			except rospy.ServiceException as ex:
				return
			return copy_ros_uavcan(self.UAVCAN_Type.Response(), ros_resp, request=False)
		uavcan_node.add_handler(self.UAVCAN_Type, handler)

# Returns a ROS type for a given UAVCAN type.
def ros_type_from_type(uavcan_type):
	if uavcan_type.category == uavcan_type.CATEGORY_COMPOUND:
		if uavcan_type.kind == uavcan_type.KIND_MESSAGE:
			return canros.Message(uavcan_type.full_name).Type
		elif uavcan_type.kind == uavcan_type.KIND_SERVICE:
			return canros.Service(uavcan_type.full_name).Type

	# No ROS type so return something that evaluates to None.
	return lambda: None

'''
Copies a UAVCAN message into a ROS message including service requests or responses.

uavcan_type: Only required when the message category is not compound.
request: Set to True for service requests and False for service responses. Do not set for messages.
'''
def copy_uavcan_ros(ros_msg, uavcan_msg, uavcan_type=None, request=None):
	if uavcan_type == None:
		uavcan_type = uavcan_msg._type

	if uavcan_type.category == uavcan_type.CATEGORY_COMPOUND:
		try:
			fields = uavcan_type.fields
			union = uavcan_type.union
		except AttributeError:
			fields = uavcan_type.request_fields if request else uavcan_type.response_fields
			union = uavcan_type.request_union if request else uavcan_type.response_union

		for i, field in enumerate(fields):
			if field.type.category == field.type.CATEGORY_VOID:
				continue
			try:
				attr = getattr(uavcan_msg, field.name)
			except AttributeError:
				# expecting an AttributeError if a non union field is accessed
				pass
			else:
				val = copy_uavcan_ros(getattr(ros_msg, field.name), attr, uavcan_type=field.type, request=request)
				setattr(ros_msg, field.name, val)
				if union:
					setattr(ros_msg, canros.union_tag_field_name, i)
		return ros_msg

	if uavcan_type.category == uavcan_type.CATEGORY_ARRAY:
		ros_msg = [ros_type_from_type(uavcan_type.value_type)()]*len(uavcan_msg)
		for i in range(0, len(uavcan_msg)):
			ros_msg[i] = copy_uavcan_ros(ros_msg[i], uavcan_msg[i], uavcan_type=uavcan_type.value_type, request=request)
		return ros_msg

	if uavcan_type.category == uavcan_type.CATEGORY_PRIMITIVE:
		return uavcan_msg

	raise Exception("Could not copy UAVCAN message")

'''
Copies a ROS message into a UAVCAN message including service requests or responses.

uavcan_type: Only required when the message category is not compound.
request: Set to True for service requests and False for service responses. Do not set for messages.
'''
def copy_ros_uavcan(uavcan_msg, ros_msg, uavcan_type=None, request=None):
	if uavcan_type == None:
		uavcan_type = uavcan_msg._type

	if uavcan_type.category == uavcan_type.CATEGORY_COMPOUND:
		try:
			fields = uavcan_type.fields
			union = uavcan_type.union
		except AttributeError:
			fields = uavcan_type.request_fields if request else uavcan_type.response_fields
			union = uavcan_type.request_union if request else uavcan_type.response_union

		for i, field in enumerate(fields):
			if field.type.category == field.type.CATEGORY_VOID:
				continue
			if (not union) or i == ros_msg.canros_union_tag:
				val = copy_ros_uavcan(getattr(uavcan_msg, field.name), getattr(ros_msg, field.name), uavcan_type=field.type, request=request)
				setattr(uavcan_msg, field.name, val)
		return uavcan_msg

	if uavcan_type.category == uavcan_type.CATEGORY_ARRAY:
		# edge case for message fields of type uint8[] which get converted to strings
		if type(ros_msg) is str:
			return canros.to_uint8(ros_msg)

		for i in range(0, len(ros_msg)):
			uavcan_msg[i] = copy_ros_uavcan(uavcan_msg[i], ros_msg[i], uavcan_type=uavcan_type.value_type, request=request)

		return uavcan_msg

	if uavcan_type.category == uavcan_type.CATEGORY_PRIMITIVE:
		return ros_msg

	raise Exception("Could not copy ros message")

'''
Returns the hardware id for the system.
Roughly follows MachineIDReader from https://github.com/UAVCAN/libuavcan/blob/master/libuavcan_drivers/linux/include/uavcan_linux/system_utils.hpp.
'''
def hardware_id():
	search_locations = [
		"/etc/machine-id",
		"/var/lib/dbus/machine-id",
		"/sys/class/dmi/id/product_uuid"
	]

	for loc in search_locations:
		try:
			with open(loc, 'r') as f:
				first_line = f.readline().lower()
				hex_string = re.sub(r'[^0-9a-f]', '', first_line)
				byte_array = map(ord, hex_string.decode("hex"))
				if len(byte_array) >= 16:
					return byte_array[0:16]
		except IOError:
			pass

	raise Exception("Unable to obtain a hardware ID for this system")

def print_help(err):
	print(
		err,
		"",
		"Run canros with:",
		"rosrun canros server.py <can_interface> <uavcan_id>",
		"",
		"uavcan_id:\tUAVCAN node id for canros. Must be between 1 and 127 inclusive.",
		"can_interface:\tAddress of CAN interface.",
	sep='\n')

def main(argv):
	# Read command line arguments
	if len(argv) != 2:
		print_help("Invalid number of arguments")
		return

	uavcan_node_id = int(argv[1])
	if uavcan_node_id < 1 or uavcan_node_id > 127:
		print_help("Invalid node ID")
		return

	can_interface = argv[0]

	# Init UAVCAN logging
	uavcan.driver.slcan.logger.addHandler(logging.StreamHandler())
	uavcan.driver.slcan.logger.setLevel('DEBUG')

	# Set UAVCAN node information
	uavcan_node_info = uavcan.protocol.GetNodeInfo.Response()
	uavcan_node_info.name = canros.uavcan_name
	uavcan_node_info.software_version.major = 0
	uavcan_node_info.software_version.minor = 1
	uavcan_node_info.hardware_version.unique_id = hardware_id()

	# Start ROS and UAVCAN nodes
	global uavcan_node
	uavcan_node = uavcan.make_node(can_interface, node_id=uavcan_node_id, node_info=uavcan_node_info)
	rospy.init_node("canros")

	# Load types
	for _, typ in uavcan.TYPENAMES.iteritems():
		Message(typ) if typ.kind == typ.KIND_MESSAGE else Service(typ)

	# Spin
	while True:
		uavcan_node.spin(0.1)
		if rospy.is_shutdown():
			raise Exception("ROS shutdown")

if __name__ == "__main__":
	main(sys.argv[1:])
