import rospy
import tensorflow as tf

from geometry_msgs.msg import Point, Pose, Quaternion

from vision_receiver import VisionReceiver

class PosePublisher:
  
  def __init__(self, name, vision_address):
    rospy.init_node(name)
    rospy.loginfo('Created ros node {}'.format(name))

    self.pose_dict = {}
    self.pub_dict = {}
    self.vision_receiver = VisionReceiver(vision_address)
    rospy.loginfo('VisionReceiver created. ({}:{})'.format(*vision_address))

    def main_loop(self):
      while not rospy.is_shutdown():
        new_frame = self.vision_receiver.get()
        
        for new_robot in new_frame.robots_blue:
          pose_state = [new_robot.x, new_robot.y, new_robot.orientation]
          pose_dict[new_robot.robot_id] = pose_state

        for new_robot in new_frame.robots_yellow:
          pose_state = [new_robot.x, new_robot.y, new_robot.orientation]
          pose_dict[new_robot.robot_id] = pose_state

        self.publish_poses(self.pose_dict)

    def publish_poses(self, poses):
      for robot_id, pose in poses.items():
        if robot_id not in self.pub_dict:
          topic_name = "robot_pose_" + str(robot_id)
          self.pub_dict[robot_id] = rospy.Publisher(topic_name, Pose, queue_size=10)
          rospy.loginfo('Created new topic {}'.format(topic_name))

        # Get position from robot
        robot_pose = tuple(robot.pose)
        
        pose = Pose()
        pose.header.stamp = rospy.Time.now()
        quat = tf.transformations.quaternion_from_eurler(0, 0, robot.get_orientation)
        pose.point = Point(robot_pose[0] / 1000, robot_pose[1] / 1000, 0)
        pose.orientation = Quaternion(*quat)

        # Publish pose
        self.pub_dict[robot_id].publish(pose)
