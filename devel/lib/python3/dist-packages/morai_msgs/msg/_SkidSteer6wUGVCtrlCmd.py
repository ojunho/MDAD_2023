# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from morai_msgs/SkidSteer6wUGVCtrlCmd.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SkidSteer6wUGVCtrlCmd(genpy.Message):
  _md5sum = "cb8a43878b1b2c65f50bd53e5a4c03f4"
  _type = "morai_msgs/SkidSteer6wUGVCtrlCmd"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 cmd_type

bool Forward_input
bool Backward_input
bool Left_Turn_input
bool Right_Turn_input

float32 left_front_wheel_rpm
float32 left_middle_wheel_rpm
float32 left_rear_wheel_rpm
float32 right_front_wheel_rpm
float32 right_middle_wheel_rpm
float32 right_rear_wheel_rpm

float32 Target_linear_velocity
float32 Target_angular_velocity

"""
  __slots__ = ['cmd_type','Forward_input','Backward_input','Left_Turn_input','Right_Turn_input','left_front_wheel_rpm','left_middle_wheel_rpm','left_rear_wheel_rpm','right_front_wheel_rpm','right_middle_wheel_rpm','right_rear_wheel_rpm','Target_linear_velocity','Target_angular_velocity']
  _slot_types = ['int32','bool','bool','bool','bool','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       cmd_type,Forward_input,Backward_input,Left_Turn_input,Right_Turn_input,left_front_wheel_rpm,left_middle_wheel_rpm,left_rear_wheel_rpm,right_front_wheel_rpm,right_middle_wheel_rpm,right_rear_wheel_rpm,Target_linear_velocity,Target_angular_velocity

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SkidSteer6wUGVCtrlCmd, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.cmd_type is None:
        self.cmd_type = 0
      if self.Forward_input is None:
        self.Forward_input = False
      if self.Backward_input is None:
        self.Backward_input = False
      if self.Left_Turn_input is None:
        self.Left_Turn_input = False
      if self.Right_Turn_input is None:
        self.Right_Turn_input = False
      if self.left_front_wheel_rpm is None:
        self.left_front_wheel_rpm = 0.
      if self.left_middle_wheel_rpm is None:
        self.left_middle_wheel_rpm = 0.
      if self.left_rear_wheel_rpm is None:
        self.left_rear_wheel_rpm = 0.
      if self.right_front_wheel_rpm is None:
        self.right_front_wheel_rpm = 0.
      if self.right_middle_wheel_rpm is None:
        self.right_middle_wheel_rpm = 0.
      if self.right_rear_wheel_rpm is None:
        self.right_rear_wheel_rpm = 0.
      if self.Target_linear_velocity is None:
        self.Target_linear_velocity = 0.
      if self.Target_angular_velocity is None:
        self.Target_angular_velocity = 0.
    else:
      self.cmd_type = 0
      self.Forward_input = False
      self.Backward_input = False
      self.Left_Turn_input = False
      self.Right_Turn_input = False
      self.left_front_wheel_rpm = 0.
      self.left_middle_wheel_rpm = 0.
      self.left_rear_wheel_rpm = 0.
      self.right_front_wheel_rpm = 0.
      self.right_middle_wheel_rpm = 0.
      self.right_rear_wheel_rpm = 0.
      self.Target_linear_velocity = 0.
      self.Target_angular_velocity = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_i4B8f().pack(_x.cmd_type, _x.Forward_input, _x.Backward_input, _x.Left_Turn_input, _x.Right_Turn_input, _x.left_front_wheel_rpm, _x.left_middle_wheel_rpm, _x.left_rear_wheel_rpm, _x.right_front_wheel_rpm, _x.right_middle_wheel_rpm, _x.right_rear_wheel_rpm, _x.Target_linear_velocity, _x.Target_angular_velocity))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 40
      (_x.cmd_type, _x.Forward_input, _x.Backward_input, _x.Left_Turn_input, _x.Right_Turn_input, _x.left_front_wheel_rpm, _x.left_middle_wheel_rpm, _x.left_rear_wheel_rpm, _x.right_front_wheel_rpm, _x.right_middle_wheel_rpm, _x.right_rear_wheel_rpm, _x.Target_linear_velocity, _x.Target_angular_velocity,) = _get_struct_i4B8f().unpack(str[start:end])
      self.Forward_input = bool(self.Forward_input)
      self.Backward_input = bool(self.Backward_input)
      self.Left_Turn_input = bool(self.Left_Turn_input)
      self.Right_Turn_input = bool(self.Right_Turn_input)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_i4B8f().pack(_x.cmd_type, _x.Forward_input, _x.Backward_input, _x.Left_Turn_input, _x.Right_Turn_input, _x.left_front_wheel_rpm, _x.left_middle_wheel_rpm, _x.left_rear_wheel_rpm, _x.right_front_wheel_rpm, _x.right_middle_wheel_rpm, _x.right_rear_wheel_rpm, _x.Target_linear_velocity, _x.Target_angular_velocity))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 40
      (_x.cmd_type, _x.Forward_input, _x.Backward_input, _x.Left_Turn_input, _x.Right_Turn_input, _x.left_front_wheel_rpm, _x.left_middle_wheel_rpm, _x.left_rear_wheel_rpm, _x.right_front_wheel_rpm, _x.right_middle_wheel_rpm, _x.right_rear_wheel_rpm, _x.Target_linear_velocity, _x.Target_angular_velocity,) = _get_struct_i4B8f().unpack(str[start:end])
      self.Forward_input = bool(self.Forward_input)
      self.Backward_input = bool(self.Backward_input)
      self.Left_Turn_input = bool(self.Left_Turn_input)
      self.Right_Turn_input = bool(self.Right_Turn_input)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i4B8f = None
def _get_struct_i4B8f():
    global _struct_i4B8f
    if _struct_i4B8f is None:
        _struct_i4B8f = struct.Struct("<i4B8f")
    return _struct_i4B8f
