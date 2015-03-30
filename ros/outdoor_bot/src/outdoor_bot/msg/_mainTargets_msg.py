"""autogenerated by genpy from outdoor_bot/mainTargets_msg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class mainTargets_msg(genpy.Message):
  _md5sum = "84c0a4f724e133f08505201eac435cf1"
  _type = "outdoor_bot/mainTargets_msg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 centerX
int32 centerY
int32 totalX
float32 rangeSquared

"""
  __slots__ = ['centerX','centerY','totalX','rangeSquared']
  _slot_types = ['int32','int32','int32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       centerX,centerY,totalX,rangeSquared

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(mainTargets_msg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.centerX is None:
        self.centerX = 0
      if self.centerY is None:
        self.centerY = 0
      if self.totalX is None:
        self.totalX = 0
      if self.rangeSquared is None:
        self.rangeSquared = 0.
    else:
      self.centerX = 0
      self.centerY = 0
      self.totalX = 0
      self.rangeSquared = 0.

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
      buff.write(_struct_3if.pack(_x.centerX, _x.centerY, _x.totalX, _x.rangeSquared))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.centerX, _x.centerY, _x.totalX, _x.rangeSquared,) = _struct_3if.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3if.pack(_x.centerX, _x.centerY, _x.totalX, _x.rangeSquared))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.centerX, _x.centerY, _x.totalX, _x.rangeSquared,) = _struct_3if.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3if = struct.Struct("<3if")