#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import random
import numpy as np
import os
import os.path
from spencer_tracking_msgs.msg import TrackedPersons

class tracker:
	self._track = False
	self._tracker_ID = 0
	self._tracker_class = None
	self._tracker_alphas = None
	self._tracker_last_detection_time = 0
	self._tracker_last_detection = None
	self._initilaized = False
	self.track_people("/spencer/perception/tracker_persons", TrackedPersons,self.detection_callback)
	
	def detection_callback(self,detections):
		time = detections.header.secs + detections.header.nsecs * 10 **(-9)
		if not self._track:
			return None 
		human_data = []
		
		for person in detectionsi.TrackedPersons:
			if person.detection_id == self._tracker_ID:
				if not self._initilaized:
					self._tracker_last_detection_time = time
					self._tracker_last_detection = np.reshape([person.pose.Point.x,person.pose.Point.y],(2,1))
					break
				if person.is_matched == False and time > self._tracker_last_detection_time + 1:
					break
			else:
				human_data.append(np.reshape([person.pose.Point.x,person.pose.Point.y],(2,1)))
                                self._tracker_class

				
				

