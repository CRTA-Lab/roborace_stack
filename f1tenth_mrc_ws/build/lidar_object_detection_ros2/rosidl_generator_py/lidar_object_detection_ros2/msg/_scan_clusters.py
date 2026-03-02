# generated from rosidl_generator_py/resource/_idl.py.em
# with input from lidar_object_detection_ros2:msg/ScanClusters.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'labels'
import array  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ScanClusters(type):
    """Metaclass of message 'ScanClusters'."""

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
                'lidar_object_detection_ros2.msg.ScanClusters')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__scan_clusters
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__scan_clusters
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__scan_clusters
            cls._TYPE_SUPPORT = module.type_support_msg__msg__scan_clusters
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__scan_clusters

            from lidar_object_detection_ros2.msg import Pose2D
            if Pose2D.__class__._TYPE_SUPPORT is None:
                Pose2D.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ScanClusters(metaclass=Metaclass_ScanClusters):
    """Message class 'ScanClusters'."""

    __slots__ = [
        '_header',
        '_points',
        '_labels',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'points': 'sequence<lidar_object_detection_ros2/Pose2D>',
        'labels': 'sequence<int32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['lidar_object_detection_ros2', 'msg'], 'Pose2D')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.points = kwargs.get('points', [])
        self.labels = array.array('i', kwargs.get('labels', []))

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
        if self.header != other.header:
            return False
        if self.points != other.points:
            return False
        if self.labels != other.labels:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def points(self):
        """Message field 'points'."""
        return self._points

    @points.setter
    def points(self, value):
        if __debug__:
            from lidar_object_detection_ros2.msg import Pose2D
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Pose2D) for v in value) and
                 True), \
                "The 'points' field must be a set or sequence and each value of type 'Pose2D'"
        self._points = value

    @property
    def labels(self):
        """Message field 'labels'."""
        return self._labels

    @labels.setter
    def labels(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'labels' array.array() must have the type code of 'i'"
            self._labels = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'labels' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._labels = array.array('i', value)
