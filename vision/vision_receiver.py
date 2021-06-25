import socket
import queue
import threading
import logging
from ipaddress import ip_address

import struct

from observations import DetectionFrame, BallObservation, RobotObservation
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket


class VisionReceiver:

    def __init__(self, server_address):

        self.server_address = server_address

        self.logger = logging.getLogger('VisionReceiver')
        print(server_address)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(server_address)

        if ip_address(server_address[0]).is_multicast:
            self.socket.setsockopt(socket.IPPROTO_IP,
                                   socket.IP_ADD_MEMBERSHIP,
                                   struct.pack("=4sl", socket.inet_aton(server_address[0]), socket.INADDR_ANY))

        self.wait_for_detection = True
        self._detection_frame_queue = queue.Queue()

        self._thread = threading.Thread(target=self.receive_packet)
        self._thread.daemon = True

    def get(self):
        return self._detection_frame_queue.get()

    def start(self):
        self.logger.info('Starting vision receiver thread.')
        self._wait_for_detection()
        self._thread.start()

    def receive_packet(self):
        packet = SSL_WrapperPacket()
        while True:
            data, _ = self.socket.recvfrom(2048)
            packet.ParseFromString(data)
            if packet.HasField('detection'):
                self.create_detection_frame(packet)

    def _wait_for_detection(self):
        self.logger.info('Waiting for detection from {}:{}'.format(*self.server_address))
        packet = SSL_WrapperPacket()
        while self.wait_for_detection:
            data, _ = self.socket.recvfrom(2048)
            packet.ParseFromString(data)
            if packet.HasField('detection'):
                self.logger.info('Detection packet received.')
                self.wait_for_detection = False

    def create_detection_frame(self, packet):
        balls = []
        for ball in packet.detection.balls:
            ball_fields = VisionReceiver.parse_proto(ball)
            balls.append(BallObservation(**ball_fields))

        robots_blue = []
        for robot in packet.detection.robots_blue:
            robot_fields = VisionReceiver.parse_proto(robot)
            robots_blue.append(RobotObservation(**robot_fields))

        robots_yellow = []
        for robot in packet.detection.robots_yellow:
            robot_fields = VisionReceiver.parse_proto(robot)
            robots_yellow.append(RobotObservation(**robot_fields))

        frame_fields = VisionReceiver.parse_proto(packet.detection)
        frame_fields['balls'] = balls
        frame_fields['robots_blue'] = robots_blue
        frame_fields['robots_yellow'] = robots_yellow

        self._detection_frame_queue.put(DetectionFrame(**frame_fields))

    @staticmethod
    def parse_proto(proto_packet):
        return dict(map(lambda f: (f[0].name, f[1]), proto_packet.ListFields()))
