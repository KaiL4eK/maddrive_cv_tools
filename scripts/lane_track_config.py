#!/usr/bin/env python

import rospy
import rosparam

class LaneTrackConfig:
	filepath = '$HOME/.maddrive_config/lane_config.yaml'
	roi_param_name = 'lane_roi'

	def __init__(self, ns='lane_config'):
		self.ns = ns

	def save_params(self):
		rospy.loginfo('Saving parameters')
		rosparam.dump_params(LaneTrackConfig.filepath, self.ns, True)

	def load_params(self):
		rospy.loginfo('Loading parameters')
		rosparam.load_file(LaneTrackConfig.filepath, self.ns, True)

	def set_roi_param(self, roi):
		rosparam.set_param(roi_param_name, str(roi), True)
