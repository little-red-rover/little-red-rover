from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class TimeStamp(_message.Message):
    __slots__ = ("sec", "nanosec")
    SEC_FIELD_NUMBER: _ClassVar[int]
    NANOSEC_FIELD_NUMBER: _ClassVar[int]
    sec: int
    nanosec: int
    def __init__(self, sec: _Optional[int] = ..., nanosec: _Optional[int] = ...) -> None: ...

class TwistCmd(_message.Message):
    __slots__ = ("time", "v", "w")
    TIME_FIELD_NUMBER: _ClassVar[int]
    V_FIELD_NUMBER: _ClassVar[int]
    W_FIELD_NUMBER: _ClassVar[int]
    time: TimeStamp
    v: float
    w: float
    def __init__(self, time: _Optional[_Union[TimeStamp, _Mapping]] = ..., v: _Optional[float] = ..., w: _Optional[float] = ...) -> None: ...

class LaserScan(_message.Message):
    __slots__ = ("time", "angle_min", "angle_max", "angle_increment", "time_increment", "scan_time", "range_min", "range_max", "ranges", "intensities")
    TIME_FIELD_NUMBER: _ClassVar[int]
    ANGLE_MIN_FIELD_NUMBER: _ClassVar[int]
    ANGLE_MAX_FIELD_NUMBER: _ClassVar[int]
    ANGLE_INCREMENT_FIELD_NUMBER: _ClassVar[int]
    TIME_INCREMENT_FIELD_NUMBER: _ClassVar[int]
    SCAN_TIME_FIELD_NUMBER: _ClassVar[int]
    RANGE_MIN_FIELD_NUMBER: _ClassVar[int]
    RANGE_MAX_FIELD_NUMBER: _ClassVar[int]
    RANGES_FIELD_NUMBER: _ClassVar[int]
    INTENSITIES_FIELD_NUMBER: _ClassVar[int]
    time: TimeStamp
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    scan_time: float
    range_min: float
    range_max: float
    ranges: _containers.RepeatedScalarFieldContainer[float]
    intensities: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, time: _Optional[_Union[TimeStamp, _Mapping]] = ..., angle_min: _Optional[float] = ..., angle_max: _Optional[float] = ..., angle_increment: _Optional[float] = ..., time_increment: _Optional[float] = ..., scan_time: _Optional[float] = ..., range_min: _Optional[float] = ..., range_max: _Optional[float] = ..., ranges: _Optional[_Iterable[float]] = ..., intensities: _Optional[_Iterable[float]] = ...) -> None: ...

class JointStates(_message.Message):
    __slots__ = ("time", "name", "position", "velocity", "effort")
    TIME_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    POSITION_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_FIELD_NUMBER: _ClassVar[int]
    EFFORT_FIELD_NUMBER: _ClassVar[int]
    time: TimeStamp
    name: _containers.RepeatedScalarFieldContainer[str]
    position: _containers.RepeatedScalarFieldContainer[float]
    velocity: _containers.RepeatedScalarFieldContainer[float]
    effort: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, time: _Optional[_Union[TimeStamp, _Mapping]] = ..., name: _Optional[_Iterable[str]] = ..., position: _Optional[_Iterable[float]] = ..., velocity: _Optional[_Iterable[float]] = ..., effort: _Optional[_Iterable[float]] = ...) -> None: ...

class UdpPacket(_message.Message):
    __slots__ = ("laser", "joint_states", "cmd_vel")
    LASER_FIELD_NUMBER: _ClassVar[int]
    JOINT_STATES_FIELD_NUMBER: _ClassVar[int]
    CMD_VEL_FIELD_NUMBER: _ClassVar[int]
    laser: LaserScan
    joint_states: JointStates
    cmd_vel: TwistCmd
    def __init__(self, laser: _Optional[_Union[LaserScan, _Mapping]] = ..., joint_states: _Optional[_Union[JointStates, _Mapping]] = ..., cmd_vel: _Optional[_Union[TwistCmd, _Mapping]] = ...) -> None: ...
