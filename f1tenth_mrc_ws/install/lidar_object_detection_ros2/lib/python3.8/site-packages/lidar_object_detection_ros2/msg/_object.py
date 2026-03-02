# generated from rosidl_generator_py/resource/_idl.py.em
# with input from lidar_object_detection_ros2:msg/Object.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Object(type):
    """Metaclass of message 'Object'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('lidar_object_detection_ros2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'lidar_object_detection_ros2.msg.Object')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__object
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__object
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__object
            cls._TYPE_SUPPORT = module.type_support_msg__msg__object
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__object

            from lidar_object_detection_ros2.msg import LShape
            if LShape.__class__._TYPE_SUPPORT is None:
                LShape.__class__.__import_type_support__()

            from lidar_object_detection_ros2.msg import Pose2D
            if Pose2D.__class__._TYPE_SUPPORT is None:
                Pose2D.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Object(metaclass=Metaclass_Object):
    """Message class 'Object'."""

    __slots__ = [
        '_id',
        '_l_shape',
        '_pose',
    ]

    _fields_and_field_types = {
        'id': 'int32',
        'l_shape': 'lidar_object_detection_ros2/LShape',
        'pose': 'lidar_object_detection_ros2/Pose2D',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['lidar_object_detection_ros2', 'msg'], 'LShape'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['lidar_object_detection_ros2', 'msg'], 'Pose2D'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        from lidar_object_detection_ros2.msg import LShape
        self.l_shape = kwargs.get('l_shape', LShape())
        from lidar_object_detection_ros2.msg import Pose2D
        self.pose = kwargs.get('pose', Pose2D())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.id != other.id:
            return False
        if self.l_shape != other.l_shape:
            return False
        if self.pose != other.pose:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'id' field must be an integer in [-2147483648, 2147483647]"
        self._id = value

    @property
    def l_shape(self):
        """Message field 'l_shape'."""
        return self._l_shape

    @l_shape.setter
    def l_shape(self, value):
        if __debug__:
            from lidar_object_detection_ros2.msg import LShape
            assert \
                isinstance(value, LShape), \
                "The 'l_shape' field must be a sub message of type 'LShape'"
        self._l_shape = value

    @property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from lidar_object_detection_ros2.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'pose' field must be a sub message of type 'Pose2D'"
        self._pose = value
