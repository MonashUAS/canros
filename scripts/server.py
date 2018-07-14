#!/usr/bin/python

from __future__ import print_function
from Queue import Queue
import logging
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
	def __init__(self, uavcan_type):
		super(Message, self).__init__(uavcan_type)
		self.__ros_publisher = None

	@property
	def ROS_Publisher(self):
		if self.__ros_publisher is None:
			self.__ros_publisher = self.Publisher(queue_size=10)
		return self.__ros_publisher

	def ROS_Subscribe(self):
		def handler(event):
			if event._connection_header["callerid"] == rospy.get_name():
				# The message came from canros so ignore
				return
			uavcan_msg = canros.copy_ros_uavcan(self.UAVCAN_Type(), event)
			uavcan_node.broadcast(uavcan_msg, priority=uavcan.TRANSFER_PRIORITY_LOWEST)
		self.Subscriber(callback=handler)

	def UAVCAN_Subscribe(self):
		def handler(event):
			ros_msg = canros.copy_uavcan_ros(self.Type(), event.message)
			if self.HasIdFeild:
				setattr(ros_msg, canros.uavcan_id_field_name, event.transfer.source_node_id)
			self.ROS_Publisher.publish(ros_msg)
		uavcan_node.add_handler(self.UAVCAN_Type, handler)

class Service(Base, canros.Service):
	def __init__(self, uavcan_type):
		super(Service, self).__init__(uavcan_type)
		self.__ros_service_proxy = None

	# This is the canros server so the service naming scheme is the reverse of the canros API.
	@property
	def Request_Name(self):
		return super(Service, self).Response_Name
	@property
	def Response_Name(self):
		return super(Service, self).Request_Name
	@property
	def Request_Type(self):
		return super(Service, self).Response_Type
	@property
	def Response_Type(self):
		return super(Service, self).Request_Type
	@property
	def Request_Topic(self):
		return super(Service, self).Response_Topic
	@property
	def Response_Topic(self):
		return super(Service, self).Request_Topic

	@property
	def ROS_ServiceProxy(self):
		if self.__ros_service_proxy is None:
			self.__ros_service_proxy = self.ServiceProxy()
		return self.__ros_service_proxy

	def ROS_Subscribe(self):
		def handler(event):
			uavcan_req = canros.copy_ros_uavcan(self.UAVCAN_Type.Request(), event, request=True)

			q = Queue(maxsize=1)
			def callback(event):
				q.put(event.response if event else None)

			uavcan_id = getattr(event, canros.uavcan_id_field_name)
			if uavcan_id == 0:
				return
			uavcan_node.request(uavcan_req, uavcan_id, callback, timeout=1)   # Default UAVCAN service timeout is 1 second

			uavcan_resp = q.get()
			if uavcan_resp is None:
				return
			return canros.copy_uavcan_ros(self.Response_Type(), uavcan_resp, request=False)
		self.Service(handler)

	def UAVCAN_Subscribe(self):
		def handler(event):
			ros_req = canros.copy_uavcan_ros(self.Request_Type(), event.request, request=True)
			setattr(ros_req, canros.uavcan_id_field_name, event.transfer.source_node_id)
			try:
				ros_resp = self.ROS_ServiceProxy(ros_req)
			except rospy.ServiceException:
				return
			return canros.copy_ros_uavcan(self.UAVCAN_Type.Response(), ros_resp, request=False)
		uavcan_node.add_handler(self.UAVCAN_Type, handler)

'''
Returns the hardware id for the system.
Roughly follows MachineIDReader from:
https://github.com/UAVCAN/libuavcan/blob/master/libuavcan_drivers/linux/include/uavcan_linux/system_utils.hpp.
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

def main():
	# Init ROS node
	rospy.init_node(canros.ros_node_name)

	# Get can_interface parameter
	try:
		can_interface = rospy.get_param('~can_interface')
	except KeyError:
		print("'can_interface' ROS parameter must be set")
		return

	# Get uavcan_node_id parameter
	try:
		uavcan_node_id = int(rospy.get_param('~uavcan_id'))
		if uavcan_node_id < 0 or uavcan_node_id > 127:
			raise ValueError()
	except KeyError:
		print("'uavcan_id' ROS parameter must be set")
		return
	except ValueError:
		print("'uavcan_id' must be an integer from 0-127")
		return

	try:
		blacklist = list(rospy.get_param('~blacklist'))
	except KeyError:
		print("'blacklist' ROS parameter must be set")
	except ValueError:
		print("'blacklist' must be a list or strings")

	# Init UAVCAN logging
	uavcan.driver.slcan.logger.addHandler(logging.StreamHandler())
	uavcan.driver.slcan.logger.setLevel('DEBUG')

	# Set UAVCAN node information
	uavcan_node_info = uavcan.protocol.GetNodeInfo.Response()
	uavcan_node_info.name = canros.uavcan_name
	uavcan_node_info.software_version.major = 0
	uavcan_node_info.software_version.minor = 1
	uavcan_node_info.hardware_version.unique_id = hardware_id()

	# Start UAVCAN node
	global uavcan_node		#pylint: disable=W0603
	uavcan_node = uavcan.make_node(can_interface, node_id=uavcan_node_id, node_info=uavcan_node_info)

	# Load types
	for uavcan_name, typ in uavcan.TYPENAMES.iteritems():
		if typ.default_dtid is None:
			continue
		if uavcan_name in blacklist:
			continue

		_ = Message(typ) if typ.kind == typ.KIND_MESSAGE else Service(typ)

	# GetInfo
	def GetInfoHandler(_):
		rosmsg = canros.srv.GetNodeInfoResponse()
		rosmsg.node_info = canros.copy_uavcan_ros(rosmsg.node_info, uavcan_node.node_info, request=False)
		setattr(rosmsg.node_info.status, canros.uavcan_id_field_name, uavcan_node.node_id)
		return rosmsg
	rospy.Service(canros.get_info_topic, canros.srv.GetNodeInfo, GetInfoHandler)

	# Spin
	uavcan_errors = 0
	while not rospy.is_shutdown():
		try:
			uavcan_node.spin(0)
			if uavcan_errors > 0:
				uavcan_errors = 0
		except uavcan.transport.TransferError:
			uavcan_errors += 1
			if uavcan_errors >= 1000:
				print("Too many UAVCAN transport errors")
				break

	uavcan_node.close()
	print("canros server exited successfully")

if __name__ == "__main__":
	main()
