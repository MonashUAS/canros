# Try importing message and service definitions as they may not exist yet.
import rospy

try:
	import canros.msg as msg
	import canros.srv as srv
except ImportError:
	# If message and service definitions don't exist yet then
	# functions that rely upon them will raise with a NameError.
	pass

uavcan_id = 123
uavcan_name = "org.monashuas.canros"

msgs_with_id = ["uavcan.protocol.NodeStatus"]

uavcan_id_field_name = "canros_uavcan_id"
union_tag_field_name = "canros_union_tag"
union_const_prefix = "CANROS_UNION_TAG_"

'''
The Base class holds methods and properties used by both Messages and Services.

INIT
uavcan_name:
The full UAVCAN name for the type as a string (e.g. uavcan.protocol.param.NumericValue).
'''
class _Base(object):
	def __init__(self, uavcan_name):
		self.__uavcan_name = uavcan_name

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
	def Subscriber(self, handler, **kw):
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

uavcan_name:
The full UAVCAN name for the type as a string (e.g. uavcan.protocol.param.GetSet).
typ:
The type being sent. Must be one of "msg", "srv/req" or "srv/resp".
'''
def ros_topic(uavcan_name, typ):
	return "/canros/" + typ + "/" + uavcan_name.replace(".", "/")

# Returns the ROS name for a given full UAVCAN name.
def ros_name(uavcan_name):
	return uavcan_name.replace(".", "__")

# Whether the full UAVCAN name type has a UAVCAN id field.
def ros_has_id_field(uavcan_name):
	return uavcan_name in msgs_with_id
