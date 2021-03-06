# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from mpc_msgs/VehicleStatus.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class VehicleStatus(genpy.Message):
  _md5sum = "b163f7630993eb80c9c2c58fe427804d"
  _type = "mpc_msgs/VehicleStatus"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 car_mode
uint8 error_level
uint8 door_state
uint8 lamp_L
uint8 lamp_R
float32 voltage
float32 current
float32 steer
uint8 hand_brake
uint8 gear
uint8 stop
float32 acc
float32 speed
"""
  __slots__ = ['car_mode','error_level','door_state','lamp_L','lamp_R','voltage','current','steer','hand_brake','gear','stop','acc','speed']
  _slot_types = ['uint8','uint8','uint8','uint8','uint8','float32','float32','float32','uint8','uint8','uint8','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       car_mode,error_level,door_state,lamp_L,lamp_R,voltage,current,steer,hand_brake,gear,stop,acc,speed

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(VehicleStatus, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.car_mode is None:
        self.car_mode = 0
      if self.error_level is None:
        self.error_level = 0
      if self.door_state is None:
        self.door_state = 0
      if self.lamp_L is None:
        self.lamp_L = 0
      if self.lamp_R is None:
        self.lamp_R = 0
      if self.voltage is None:
        self.voltage = 0.
      if self.current is None:
        self.current = 0.
      if self.steer is None:
        self.steer = 0.
      if self.hand_brake is None:
        self.hand_brake = 0
      if self.gear is None:
        self.gear = 0
      if self.stop is None:
        self.stop = 0
      if self.acc is None:
        self.acc = 0.
      if self.speed is None:
        self.speed = 0.
    else:
      self.car_mode = 0
      self.error_level = 0
      self.door_state = 0
      self.lamp_L = 0
      self.lamp_R = 0
      self.voltage = 0.
      self.current = 0.
      self.steer = 0.
      self.hand_brake = 0
      self.gear = 0
      self.stop = 0
      self.acc = 0.
      self.speed = 0.

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
      buff.write(_get_struct_5B3f3B2f().pack(_x.car_mode, _x.error_level, _x.door_state, _x.lamp_L, _x.lamp_R, _x.voltage, _x.current, _x.steer, _x.hand_brake, _x.gear, _x.stop, _x.acc, _x.speed))
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
      end += 28
      (_x.car_mode, _x.error_level, _x.door_state, _x.lamp_L, _x.lamp_R, _x.voltage, _x.current, _x.steer, _x.hand_brake, _x.gear, _x.stop, _x.acc, _x.speed,) = _get_struct_5B3f3B2f().unpack(str[start:end])
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
      buff.write(_get_struct_5B3f3B2f().pack(_x.car_mode, _x.error_level, _x.door_state, _x.lamp_L, _x.lamp_R, _x.voltage, _x.current, _x.steer, _x.hand_brake, _x.gear, _x.stop, _x.acc, _x.speed))
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
      end += 28
      (_x.car_mode, _x.error_level, _x.door_state, _x.lamp_L, _x.lamp_R, _x.voltage, _x.current, _x.steer, _x.hand_brake, _x.gear, _x.stop, _x.acc, _x.speed,) = _get_struct_5B3f3B2f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5B3f3B2f = None
def _get_struct_5B3f3B2f():
    global _struct_5B3f3B2f
    if _struct_5B3f3B2f is None:
        _struct_5B3f3B2f = struct.Struct("<5B3f3B2f")
    return _struct_5B3f3B2f
