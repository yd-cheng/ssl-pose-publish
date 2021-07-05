#!/usr/bin/env python2
from vision.publisher import PosePublisher

if __name__ == "__main__":
  vision_address = (u'224.5.23.2', 10006)
  publisher = PosePublisher("pose_tracker", vision_address)
  publisher.start() 
