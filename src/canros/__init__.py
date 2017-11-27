from concurrent.futures import ThreadPoolExecutor   # pip install futures
import time

import monotonic   # pip install monotonic
import uavcan
import rospy

try:
	# Try importing message and service definitions as they may not exist yet.
	import canros.msg as msg
	import canros.srv as srv
except ImportError:
	# If message and service definitions don't exist yet then
	# functions that rely upon them will raise a NameError.
	pass

ros_node_name = "canros"
uavcan_name = "org.monashuas.canros"

msgs_with_id = ["uavcan.protocol.NodeStatus"]

uavcan_id_field_name = "canros_uavcan_id"
union_tag_field_name = "canros_union_tag"
union_const_prefix = "CANROS_UNION_TAG_"

ros_topic_prefix = "/canros"
get_info_topic = ros_topic_prefix + "/GetInfo"

'''
The Base class holds methods and properties used by both Messages and Services.

INIT
name:
The full UAVCAN name for the type as a string (e.g. uavcan.protocol.param.NumericValue).
'''
class _Base(object):
	def __init__(self, name):
		self.__uavcan_name = name

	# The full UAVCAN name for the type as a string (e.g. uavcan.protocol.param.NumericValue).
	@property
	def UAVCAN_Name(self):
		return self.__uavcan_name

	# The full ROS name for the type as a string (e.g. uavcan__protocol__param__NumericValue).
	@property
	def Name(self):
		return ros_name(self.UAVCAN_Name)

'''
The Message class holds methods and properties specific to messages.
Creating instances of this type is the recommended way to use messages in canros.

INIT
uavcan_name:
The full UAVCAN name for the type as a string (e.g. uavcan.protocol.param.NumericValue).
'''
class Message(_Base):
	# The ROS type for this message.
	@property
	def Type(self):
		return getattr(msg, self.Name)

	# The ROS topic this message will be sent and received on.
	@property
	def Topic(self):
		return ros_topic(self.UAVCAN_Name, "msg")

	# Whether this message has a UAVCAN id field.
	@property
	def HasIdFeild(self):
		return ros_has_id_field(self.UAVCAN_Name)

	'''
	Returns a ROS Publisher for this message.
	Arguments are the same as for creating a ROS Publisher except the topic and type fields have already been allocated.

	Example:
	numeric_value = canros.Message("uavcan.protocol.param.NumericValue")
	pub = numeric_value.Publisher(queue_size=10)
	'''
	def Publisher(self, **kw):
		return rospy.Publisher(self.Topic, self.Type, **kw)

	'''
	Returns a ROS Subscriber for this message.
	Arguments are the same as for creating a ROS Subscriber except the topic and type fields have already been allocated.

	Example:
	numeric_value = canros.Message("uavcan.protocol.param.NumericValue")
	sub = numeric_value.Subscriber()
	'''
	def Subscriber(self, **kw):
		return rospy.Subscriber(self.Topic, self.Type, **kw)

'''
The Service class holds methods and properties specific to services.
Creating instances of this type is the recommended way to use services in canros.

This class uses 'Request' and 'Response' to differentiate between service requests that originate in the ROS and UAVCAN networks.
'Request' is used when a service request is sent from the ROS network into the UAVCAN network and the response comes back into the ROS network.
'Response' is used when a service request is sent from the UAVCAN network into the ROS network and the response is sent back into the UAVCAN network.

INIT
uavcan_name:
The full UAVCAN name for the type as a string (e.g. uavcan.protocol.param.GetSet).
'''
class Service(_Base):
	# The ROS type for this service.
	@property
	def Type(self):
		return getattr(srv, self.Name)

	# The ROS name for service Requests.
	@property
	def Request_Name(self):
		return self.Name + "Request"

	# The ROS type for service Requests.
	@property
	def Request_Type(self):
		return getattr(srv, self.Request_Name)

	# The ROS topic for service Requests.
	@property
	def Request_Topic(self):
		return ros_topic(self.UAVCAN_Name, "srv/req")

	# The ROS name for service Responses.
	@property
	def Response_Name(self):
		return self.Name + "Response"

	# The ROS type for service Responses.
	@property
	def Response_Type(self):
		return getattr(srv, self.Response_Name)

	# The ROS topic for service Responses.
	@property
	def Response_Topic(self):
		return ros_topic(self.UAVCAN_Name, "srv/resp")

	'''
	Returns a ROS ServiceProxy for this service.
	Arguments are the same as for creating a ROS ServiceProxy except the topic and type fields have already been allocated.

	Example:
	get_set = canros.Message("uavcan.protocol.param.GetSet")
	srv_proxy = get_set.ServiceProxy()
	'''
	def ServiceProxy(self, **kw):
		return rospy.ServiceProxy(self.Request_Topic, self.Type, **kw)

	'''
	Returns a ROS Service for this service.
	Arguments are the same as for creating a ROS Service except the topic and type fields have already been allocated.

	Example:
	get_set = canros.Message("uavcan.protocol.param.GetSet")
	service = get_set.Service()
	'''
	def Service(self, handler, **kw):
		return rospy.Service(self.Response_Topic, self.Type, handler, **kw)

# Converts a string to a list of uint8s.
def to_uint8(string):
	return map(ord, string)

# Converts a list of uint8s to a string.
def to_string(uint8_list):
	return str(bytearray(uint8_list))

'''
Returns the ROS topic for a given UAVCAN full name and type.

name:
The full UAVCAN name for the type as a string (e.g. uavcan.protocol.param.GetSet).
typ:
The type being sent. Must be one of "msg", "srv/req" or "srv/resp".
'''
def ros_topic(name, typ):
	return ros_topic_prefix + "/" + typ + "/" + name.replace(".", "/")

# Returns the ROS name for a given full UAVCAN name.
def ros_name(name):
	return name.replace(".", "__")

# Whether the full UAVCAN name type has a UAVCAN id field.
def ros_has_id_field(name):
	return name in msgs_with_id

# Returns a ROS type for a given UAVCAN type.
def ros_type_from_type(uavcan_type):
	if uavcan_type.category == uavcan_type.CATEGORY_COMPOUND:
		if uavcan_type.kind == uavcan_type.KIND_MESSAGE:
			return Message(uavcan_type.full_name).Type
		elif uavcan_type.kind == uavcan_type.KIND_SERVICE:
			return Service(uavcan_type.full_name).Type

	# No ROS type so return something that evaluates to None.
	return lambda: None

'''
Copies a UAVCAN message into a ROS message including service requests or responses.

uavcan_type: Only required when the message category is not compound.
request: Set to True for service requests and False for service responses. Do not set for messages.
'''
def copy_uavcan_ros(ros_msg, uavcan_msg, uavcan_type=None, request=None):
	uavcan_type = uavcan_type or uavcan.get_uavcan_data_type(uavcan_msg)

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
					setattr(ros_msg, union_tag_field_name, i)
		return ros_msg

	if uavcan_type.category == uavcan_type.CATEGORY_ARRAY:
		if isinstance(ros_msg, str):
			ros_msg = []
		for uavcan_item in uavcan_msg:
			ros_msg.append(copy_uavcan_ros(
				ros_type_from_type(uavcan_type.value_type)(),
				uavcan_item,
				uavcan_type=uavcan_type.value_type,
				request=request
			))
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
	uavcan_type = uavcan_type or uavcan.get_uavcan_data_type(uavcan_msg)

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
				val = copy_ros_uavcan(
					getattr(uavcan_msg, field.name),
					getattr(ros_msg, field.name),
					uavcan_type=field.type,
					request=request
				)
				if val != getattr(uavcan_msg, field.name):
					setattr(uavcan_msg, field.name, val)
		return uavcan_msg

	if uavcan_type.category == uavcan_type.CATEGORY_ARRAY:
		# edge case for message fields of type uint8[] which get converted to strings
		if isinstance(ros_msg, str):
			return to_uint8(ros_msg)

		for ros_item in ros_msg:
			uavcan_msg.append(copy_ros_uavcan(
				uavcan_msg.new_item(),
				ros_item,
				uavcan_type=uavcan_type.value_type,
				request=request
			))

		return uavcan_msg

	if uavcan_type.category == uavcan_type.CATEGORY_PRIMITIVE:
		return ros_msg

	raise Exception("Could not copy ros message")

'''
An imitation pyuavcan node.
Implements a subset of features of a pyuavcan node using canros and ROS.
Can be used with pyuavcan apps: https://github.com/UAVCAN/pyuavcan/tree/master/uavcan/app.
'''
class uavcan_node(object):
	# Mimics an event object that is passed to callbaks and handlers.
	class _Event(object):
		# Mimics a transfer object that is inside an event object.
		class _Transfer(object):
			def __init__(self, source_node_id):
				self.source_node_id = source_node_id
				self.dest_node_id = 0
				self.ts_monotonic = monotonic.monotonic()
				self.ts_real = time.time()

		def __init__(self, node, uavcan_type, ros_event, uavcan_id=0, request=None):
			self.node = node
			if uavcan_type.kind == uavcan_type.KIND_MESSAGE:
				self.message = copy_ros_uavcan(uavcan_type(), ros_event)
			elif uavcan_type.kind == uavcan_type.KIND_SERVICE:
				if request:
					self.request = copy_ros_uavcan(uavcan_type.Request(), ros_event, request=True)
				else:
					self.response = copy_ros_uavcan(uavcan_type.Response(), ros_event, request=False)
			if uavcan_id == 0:
				try:
					uavcan_id = getattr(ros_event, uavcan_id_field_name)
				except AttributeError:
					pass
			self.transfer = uavcan_node._Event._Transfer(uavcan_id)

	# Mimics a message handler. Used in add_handler.
	class _MessageHandler(object):
		def __init__(self, node, uavcan_type, callback):
			def cb_wrapper(ros_event):
				if ros_event._connection_header["callerid"] != rospy.get_name():
					callback(uavcan_node._Event(node, uavcan_type, ros_event))
			sub = Message(uavcan_type.full_name).Subscriber(callback=cb_wrapper)
			self.remove = sub.unregister

	# Mimics a service handler. Used in add_handler.
	class _ServiceHandler(object):
		def __init__(self, node, uavcan_type, callback):
			canros_service = Service(uavcan_type.full_name)
			def cb_wrapper(ros_event):
				uavcan_resp = callback(uavcan_node._Event(node, uavcan_type, ros_event, request=True))
				return copy_uavcan_ros(canros_service.Response_Type(), uavcan_resp, request=False)
			service = canros_service.Service(cb_wrapper)
			self.remove = service.shutdown

	def __init__(self):
		self.is_anonymous = False   # canros is never anonymous

		self.__publishers = {}
		self.__service_proxies = {}
		self.__handlers = []

		self.__request_executor = ThreadPoolExecutor(max_workers=5)

		self.__node_info = None
		self.__node_info_sp = rospy.ServiceProxy(get_info_topic, srv.GetNodeInfo)
		self.__node_info_last = -1

	def __get_node_info(self):
		if monotonic.monotonic() - self.__node_info_last >= 1:
			self.__node_info_last = monotonic.monotonic()
			try:
				self.__node_info = self.__node_info_sp().node_info
			except rospy.ServiceException:
				raise Exception("Could not connect to canros server.")
		return self.__node_info

	@property
	def node_info(self):
		return copy_ros_uavcan(uavcan.protocol.GetNodeInfo.Response(), self.__get_node_info(), request=False)

	@property
	def node_id(self):
		return getattr(self.__get_node_info().status, uavcan_id_field_name)

	def add_handler(self, uavcan_type, callback):
		if uavcan_type.kind == uavcan_type.KIND_MESSAGE:
			handler = uavcan_node._MessageHandler(self, uavcan_type, callback)
		elif uavcan_type.kind == uavcan_type.KIND_SERVICE:
			handler = uavcan_node._ServiceHandler(self, uavcan_type, callback)
		self.__handlers.append(handler)
		return handler

	def broadcast(self, uavcan_msg, priority=None):
		#pylint: disable=W0613
		uavcan_full_name = uavcan.get_uavcan_data_type(uavcan_msg).full_name
		if not uavcan_full_name in self.__publishers:
			self.__publishers[uavcan_full_name] = Message(uavcan_full_name).Publisher(queue_size=10)
		publisher = self.__publishers[uavcan_full_name]
		publisher.publish(copy_uavcan_ros(publisher.data_class(), uavcan_msg))

	def request(self, uavcan_msg, node_id, callback, priority=None, timeout=None):
		#pylint: disable=W0613
		uavcan_type = uavcan.get_uavcan_data_type(uavcan_msg)
		if not uavcan_type.full_name in self.__service_proxies:
			self.__service_proxies[uavcan_type.full_name] = Service(uavcan_type.full_name).ServiceProxy()
		service_proxy = self.__service_proxies[uavcan_type.full_name]

		ros_req = copy_uavcan_ros(service_proxy.request_class(), uavcan_msg, request=True)
		setattr(ros_req, uavcan_id_field_name, node_id)

		def service_proxy_call():
			try:
				return service_proxy(ros_req)
			except rospy.ServiceException:
				return None

		def request_finished(fut):
			ros_resp = fut.result()
			if ros_resp is None:
				uavcan_resp = None
			else:
				uavcan_resp = uavcan_node._Event(self, uavcan_type, ros_resp, uavcan_id=node_id, request=False)
			callback(uavcan_resp)

		self.__request_executor.submit(service_proxy_call).add_done_callback(request_finished)

	class defer(rospy.Timer):
		def __init__(self, timeout_seconds, callback):
			super(uavcan_node.defer, self).__init__(rospy.Duration(timeout_seconds), lambda event: callback(), oneshot=True)
			self.remove = self.shutdown

	class periodic(rospy.Timer):
		def __init__(self, period, callback):
			super(uavcan_node.periodic, self).__init__(rospy.Duration(period), lambda event: callback())
			self.remove = self.shutdown

	def close(self):
		for _, publisher in self.__publishers.iteritems():
			publisher.unregister()
		for _, service_proxy in self.__service_proxies.iteritems():
			service_proxy.close()
		for handler in self.__handlers:
			handler.remove()
		self.__node_info_sp.close()
