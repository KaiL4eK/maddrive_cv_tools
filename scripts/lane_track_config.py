#!/usr/bin/env python

import rospy
import rosparam
from os.path import expanduser

class LaneTrackConfig:
	filepath = expanduser("~") + '/.maddrive_config/lane_config.yaml'
	roi_param_name = 'lane_roi'

	def __init__(self, ns='lane_config'):
		self.ns = ns

	def save_params(self):
		rospy.loginfo('Saving parameters')
		rosparam.dump_params(LaneTrackConfig.filepath, self.ns, True)

	def load_params(self):
		try:
			rospy.loginfo('Loading parameters')
			rosparam.load_file(LaneTrackConfig.filepath, self.ns, True)
			return True
		except:
			rospy.logwarn('Failed to read data from config')
			return False

	def set_roi_param(self, roi):
		rosparam.set_param(self.ns + '/' + LaneTrackConfig.roi_param_name, str(roi))

	def get_roi_param(self):
		try:
			result = rosparam.get_param(self.ns + '/' + LaneTrackConfig.roi_param_name)
			return result
		except:
			rospy.logwarn('Failed to get param from server')
			return -1
