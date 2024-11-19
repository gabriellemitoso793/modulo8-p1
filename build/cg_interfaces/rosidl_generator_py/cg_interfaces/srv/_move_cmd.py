# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cg_interfaces:srv/MoveCmd.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MoveCmd_Request(type):
    """Metaclass of message 'MoveCmd_Request'."""

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
            module = import_type_support('cg_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cg_interfaces.srv.MoveCmd_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__move_cmd__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__move_cmd__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__move_cmd__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__move_cmd__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__move_cmd__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveCmd_Request(metaclass=Metaclass_MoveCmd_Request):
    """Message class 'MoveCmd_Request'."""

    __slots__ = [
        '_direction',
    ]

    _fields_and_field_types = {
        'direction': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.direction = kwargs.get('direction', str())

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
        if self.direction != other.direction:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def direction(self):
        """Message field 'direction'."""
        return self._direction

    @direction.setter
    def direction(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'direction' field must be of type 'str'"
        self._direction = value


# Import statements for member types

# already imported above
# import builtins

# Member 'robot_pos'
# Member 'target_pos'
import numpy  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_MoveCmd_Response(type):
    """Metaclass of message 'MoveCmd_Response'."""

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
            module = import_type_support('cg_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cg_interfaces.srv.MoveCmd_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__move_cmd__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__move_cmd__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__move_cmd__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__move_cmd__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__move_cmd__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveCmd_Response(metaclass=Metaclass_MoveCmd_Response):
    """Message class 'MoveCmd_Response'."""

    __slots__ = [
        '_success',
        '_robot_pos',
        '_target_pos',
        '_left',
        '_down',
        '_up',
        '_right',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'robot_pos': 'int8[2]',
        'target_pos': 'int8[2]',
        'left': 'string',
        'down': 'string',
        'up': 'string',
        'right': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int8'), 2),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int8'), 2),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        if 'robot_pos' not in kwargs:
            self.robot_pos = numpy.zeros(2, dtype=numpy.int8)
        else:
            self.robot_pos = numpy.array(kwargs.get('robot_pos'), dtype=numpy.int8)
            assert self.robot_pos.shape == (2, )
        if 'target_pos' not in kwargs:
            self.target_pos = numpy.zeros(2, dtype=numpy.int8)
        else:
            self.target_pos = numpy.array(kwargs.get('target_pos'), dtype=numpy.int8)
            assert self.target_pos.shape == (2, )
        self.left = kwargs.get('left', str())
        self.down = kwargs.get('down', str())
        self.up = kwargs.get('up', str())
        self.right = kwargs.get('right', str())

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
        if self.success != other.success:
            return False
        if all(self.robot_pos != other.robot_pos):
            return False
        if all(self.target_pos != other.target_pos):
            return False
        if self.left != other.left:
            return False
        if self.down != other.down:
            return False
        if self.up != other.up:
            return False
        if self.right != other.right:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def robot_pos(self):
        """Message field 'robot_pos'."""
        return self._robot_pos

    @robot_pos.setter
    def robot_pos(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int8, \
                "The 'robot_pos' numpy.ndarray() must have the dtype of 'numpy.int8'"
            assert value.size == 2, \
                "The 'robot_pos' numpy.ndarray() must have a size of 2"
            self._robot_pos = value
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
                 len(value) == 2 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -128 and val < 128 for val in value)), \
                "The 'robot_pos' field must be a set or sequence with length 2 and each value of type 'int' and each integer in [-128, 127]"
        self._robot_pos = numpy.array(value, dtype=numpy.int8)

    @builtins.property
    def target_pos(self):
        """Message field 'target_pos'."""
        return self._target_pos

    @target_pos.setter
    def target_pos(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int8, \
                "The 'target_pos' numpy.ndarray() must have the dtype of 'numpy.int8'"
            assert value.size == 2, \
                "The 'target_pos' numpy.ndarray() must have a size of 2"
            self._target_pos = value
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
                 len(value) == 2 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -128 and val < 128 for val in value)), \
                "The 'target_pos' field must be a set or sequence with length 2 and each value of type 'int' and each integer in [-128, 127]"
        self._target_pos = numpy.array(value, dtype=numpy.int8)

    @builtins.property
    def left(self):
        """Message field 'left'."""
        return self._left

    @left.setter
    def left(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'left' field must be of type 'str'"
        self._left = value

    @builtins.property
    def down(self):
        """Message field 'down'."""
        return self._down

    @down.setter
    def down(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'down' field must be of type 'str'"
        self._down = value

    @builtins.property
    def up(self):
        """Message field 'up'."""
        return self._up

    @up.setter
    def up(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'up' field must be of type 'str'"
        self._up = value

    @builtins.property
    def right(self):
        """Message field 'right'."""
        return self._right

    @right.setter
    def right(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'right' field must be of type 'str'"
        self._right = value


class Metaclass_MoveCmd(type):
    """Metaclass of service 'MoveCmd'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('cg_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cg_interfaces.srv.MoveCmd')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__move_cmd

            from cg_interfaces.srv import _move_cmd
            if _move_cmd.Metaclass_MoveCmd_Request._TYPE_SUPPORT is None:
                _move_cmd.Metaclass_MoveCmd_Request.__import_type_support__()
            if _move_cmd.Metaclass_MoveCmd_Response._TYPE_SUPPORT is None:
                _move_cmd.Metaclass_MoveCmd_Response.__import_type_support__()


class MoveCmd(metaclass=Metaclass_MoveCmd):
    from cg_interfaces.srv._move_cmd import MoveCmd_Request as Request
    from cg_interfaces.srv._move_cmd import MoveCmd_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
