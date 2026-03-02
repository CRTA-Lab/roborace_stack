# generated from rosidl_generator_py/resource/_idl.py.em
# with input from lidar_object_detection_ros2:msg/LShape.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LShape(type):
    """Metaclass of message 'LShape'."""

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
                'lidar_object_detection_ros2.msg.LShape')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__l_shape
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__l_shape
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__l_shape
            cls._TYPE_SUPPORT = module.type_support_msg__msg__l_shape
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__l_shape

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


class LShape(metaclass=Metaclass_LShape):
    """Message class 'LShape'."""

    __slots__ = [
        '_c1',
        '_theta',
        '_l1',
        '_l2',
    ]

    _fields_and_field_types = {
        'c1': 'lidar_object_detection_ros2/Pose2D',
        'theta': 'float',
        'l1': 'float',
        'l2': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['lidar_object_detection_ros2', 'msg'], 'Pose2D'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from lidar_object_detection_ros2.msg import Pose2D
        self.c1 = kwargs.get('c1', Pose2D())
        self.theta = kwargs.get('theta', float())
        self.l1 = kwargs.get('l1', float())
        self.l2 = kwargs.get('l2', float())

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
        if self.c1 != other.c1:
            return False
        if self.theta != other.theta:
            return False
        if self.l1 != other.l1:
            return False
        if self.l2 != other.l2:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def c1(self):
        """Message field 'c1'."""
        return self._c1

    @c1.setter
    def c1(self, value):
        if __debug__:
            from lidar_object_detection_ros2.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'c1' field must be a sub message of type 'Pose2D'"
        self._c1 = value

    @property
    def theta(self):
        """Message field 'theta'."""
        return self._theta

    @theta.setter
    def theta(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'theta' field must be of type 'float'"
        self._theta = value

    @property
    def l1(self):
        """Message field 'l1'."""
        return self._l1

    @l1.setter
    def l1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l1' field must be of type 'float'"
        self._l1 = value

    @property
    def l2(self):
        """Message field 'l2'."""
        return self._l2

    @l2.setter
    def l2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l2' field must be of type 'float'"
        self._l2 = value
