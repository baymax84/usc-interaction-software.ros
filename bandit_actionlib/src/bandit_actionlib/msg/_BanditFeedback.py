# autogenerated by genmsg_py from BanditFeedback.msg. Do not edit.
import roslib.message
import struct


class BanditFeedback(roslib.message.Message):
  _md5sum = "044f5eed111da7d3edd66fd6dd98add6"
  _type = "bandit_actionlib/BanditFeedback"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#feedback
float32 progress_time
float32[] progress_joint_id
float32[] progress_joint_angle


"""
  __slots__ = ['progress_time','progress_joint_id','progress_joint_angle']
  _slot_types = ['float32','float32[]','float32[]']

  ## Constructor. Any message fields that are implicitly/explicitly
  ## set to None will be assigned a default value. The recommend
  ## use is keyword arguments as this is more robust to future message
  ## changes.  You cannot mix in-order arguments and keyword arguments.
  ##
  ## The available fields are:
  ##   progress_time,progress_joint_id,progress_joint_angle
  ##
  ## @param self: self
  ## @param args: complete set of field values, in .msg order
  ## @param kwds: use keyword arguments corresponding to message field names
  ## to set specific fields. 
  def __init__(self, *args, **kwds):
    if args or kwds:
      super(BanditFeedback, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.progress_time is None:
        self.progress_time = 0.
      if self.progress_joint_id is None:
        self.progress_joint_id = []
      if self.progress_joint_angle is None:
        self.progress_joint_angle = []
    else:
      self.progress_time = 0.
      self.progress_joint_id = []
      self.progress_joint_angle = []

  ## internal API method
  def _get_types(self): return self._slot_types

  ## serialize message into buffer
  ## @param buff StringIO: buffer
  def serialize(self, buff):
    try:
      buff.write(struct.pack('<f', self.progress_time))
      #serialize self.progress_joint_id
      length = len(self.progress_joint_id)
      buff.write(struct.pack('<I', length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.progress_joint_id))
      #serialize self.progress_joint_angle
      length = len(self.progress_joint_angle)
      buff.write(struct.pack('<I', length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.progress_joint_angle))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  ## unpack serialized message in str into this message instance
  ## @param self: self
  ## @param str str: byte array of serialized message
  def deserialize(self, str):
    try:
      end = 0
      start = end
      end += 4
      (self.progress_time,) = struct.unpack('<f',str[start:end])
      #deserialize self.progress_joint_id
      start = end
      end += 4
      (length,) = struct.unpack('<I',str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.progress_joint_id = struct.unpack(pattern, str[start:end])
      #deserialize self.progress_joint_angle
      start = end
      end += 4
      (length,) = struct.unpack('<I',str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.progress_joint_angle = struct.unpack(pattern, str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  ## serialize message with numpy array types into buffer
  ## @param self: self
  ## @param buff StringIO: buffer
  ## @param numpy module: numpy python module
  def serialize_numpy(self, buff, numpy):
    try:
      buff.write(struct.pack('<f', self.progress_time))
      #serialize self.progress_joint_id
      length = len(self.progress_joint_id)
      buff.write(struct.pack('<I', length))
      pattern = '<%sf'%length
      buff.write(self.progress_joint_id.tostring())
      #serialize self.progress_joint_angle
      length = len(self.progress_joint_angle)
      buff.write(struct.pack('<I', length))
      pattern = '<%sf'%length
      buff.write(self.progress_joint_angle.tostring())
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  ## unpack serialized message in str into this message instance using numpy for array types
  ## @param self: self
  ## @param str str: byte array of serialized message
  ## @param numpy module: numpy python module
  def deserialize_numpy(self, str, numpy):
    try:
      end = 0
      start = end
      end += 4
      (self.progress_time,) = struct.unpack('<f',str[start:end])
      #deserialize self.progress_joint_id
      start = end
      end += 4
      (length,) = struct.unpack('<I',str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.progress_joint_id = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      #deserialize self.progress_joint_angle
      start = end
      end += 4
      (length,) = struct.unpack('<I',str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.progress_joint_angle = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

