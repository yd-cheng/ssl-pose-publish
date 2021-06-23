from __future__ import absolute_import
from collections import namedtuple

_detection_frame_fields = u'frame_number, t_capture, t_sent, camera_id, balls, robots_blue, robots_yellow'
_robot_observation_fields = u'confidence, robot_id, x, y, orientation, pixel_x, pixel_y, height'
_ball_observation_fields = u'confidence, area, x, y, z, pixel_x, pixel_y'


class RobotObservation(namedtuple(u'RobotObservation', _robot_observation_fields)):

    __slots__ = ()

    # Add default argument to optional value
    def __new__(cls,
                robot_id=None,
                orientation=None,
                height=None,
                **kwargs):

        return super(RobotObservation, cls).__new__(cls, robot_id=robot_id, orientation=orientation, height=height, **kwargs)


class BallObservation(namedtuple(u'BallObservation', _ball_observation_fields)):

    __slots__ = ()

    # Add default argument to optional value
    def __new__(cls,
                area=None,
                z=None,
                **kwargs):

        return super(BallObservation, cls).__new__(cls, area=area, z=z, **kwargs)


class DetectionFrame(namedtuple(u'DetectionFrame', _detection_frame_fields)):

    __slots__ = ()

    # Add default argument to optional value
    def __new__(cls,
                balls=None,
                robots_blue=None,
                robots_yellow=None,
                **kwargs):

        return super(DetectionFrame, cls).__new__(cls, balls=balls, robots_blue=robots_blue, robots_yellow=robots_yellow, **kwargs)
