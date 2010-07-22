# autogenerated by genmsg_py from BanditResult.msg. Do not edit.
import roslib.message
import struct


class BanditResult(roslib.message.Message):
  _md5sum = "9be3f9f2d0fb855326ff4b7f571fb85a"
  _type = "bandit_actionlib/BanditResult"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#result definition
int32 total_time
float32[] result_joint_id
float32[] result_joint_angle

"""
  __slots__ = ['total_time','result_joint_id','result_joint_angle']
  _slot_types = ['int32','float32[]','float32[]']

  ## Constructor. Any message fields that are implicitly/explicitly
  ## set to None will be assigned a default value. The recommend
  ## use is keyword arguments as this is more robust to future message
  ## changes.  You cannot mix in-order arguments and keyword arguments.
  ##
  ## The available fields are:
  ##   total_time,result_joint_id,result_joint_angle
  ##
  ## @param self: self
  ## @param args: complete set of field values, in .msg order
  ## @param kwds: use keyword arguments corresponding to message field names
  ## to set specific fields. 
  def __init__(self, *args, **kwds):
    if args or kwds:
      super(BanditResult, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.total_time is None:
        self.total_time = 0
      if self.result_joint_id is None:
        self.result_joint_id = []
      if self.result_joint_angle is None:
        self.result_joint_angle = []
    else:
      self.total_time = 0
      self.result_joint_id = []
      self.result_joint_angle = []

  ## internal API method
  def _get_types(self): return self._slot_types

  ## serialize message into buffer
  ## @param buff StringIO: buffer
  def serialize(self, buff):
    try:
      buff.write(struct.pack('<i', self.total_time))
      #serialize self.result_joint_id
      length = len(self.result_joint_id)
      buff.write(struct.pack('<I', length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.result_joint_id))
      #serialize self.result_joint_angle
      length = len(self.result_joint_angle)
      buff.write(struct.pack('<I', length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.result_joint_angle))
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
      (self.total_time,) = struct.unpack('<i',str[start:end])
      #deserialize self.result_joint_id
      start = end
      end += 4
      (length,) = struct.unpack('<I',str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.result_joint_id = struct.unpack(pattern, str[start:end])
      #deserialize self.result_joint_angle
      start = end
      end += 4
      (length,) = struct.unpack('<I',str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.result_joint_angle = struct.unpack(pattern, str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  ## serialize message with numpy array types into buffer
  ## @param self: self
  ## @param buff StringIO: buffer
  ## @param numpy module: numpy python module
  def serialize_numpy(self, buff, numpy):
    try:
      buff.write(struct.pack('<i', self.total_time))
      #serialize self.result_joint_id
      length = len(self.result_joint_id)
      buff.write(struct.pack('<I', length))
      pattern = '<%sf'%length
      buff.write(self.result_joint_id.tostring())
      #serialize self.result_joint_angle
      length = len(self.result_joint_angle)
      buff.write(struct.pack('<I', length))
      pattern = '<%sf'%length
      buff.write(self.result_joint_angle.tostring())
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
      (self.total_time,) = struct.unpack('<i',str[start:end])
      #deserialize self.result_joint_id
      start = end
      end += 4
      (length,) = struct.unpack('<I',str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.result_joint_id = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      #deserialize self.result_joint_angle
      start = end
      end += 4
      (length,) = struct.unpack('<I',str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.result_joint_angle = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

