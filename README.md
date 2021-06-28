# SSL Vision Pose Publishing
Program that deserializes protobuf objects coming from the SSL Vision program and publishes the Pose (position and quaternion orientation) to a ROS topic. 

Places deserialization into its own ROS node and allows any other functionality to subscribe to the ROS topic and do whatever is needed without worrying about deserializing.
