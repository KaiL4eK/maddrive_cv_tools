#!/usr/bin/env python

import os
import errno
import cv2
import rospy
import rosparam
from os.path import expanduser

def get_frame_height_width(frame):
	if len(frame.shape) > 2:
		height, length, _ 	= frame.shape
	else:
		height, length 		= frame.shape

	return height, length

def get_frame_channels(frame):
	if len(frame.shape) > 2:
		height, length, channels 	= frame.shape
	else:
		return 1

	return channels

class ImageROI:
	def __init__(self, roi_height_rel):
		self.__roi_height_rel = roi_height_rel

	def __get_pixel_height(self, frame_height):
		return (1 - self.__roi_height_rel) * frame_height

	def draw_height_border(self, draw_frame, color=(255, 0, 0)):
		height, width = get_frame_height_width(draw_frame)
		pixel_upper_border = self.__get_pixel_height(height)
		cv2.line(draw_frame, (0, int(pixel_upper_border)), 
							 (width, int(pixel_upper_border)), color=color)

	def mask_frame(self, frame):
		height, _ = get_frame_height_width(frame)
		pixel_upper_border = self.__get_pixel_height(height)

		return frame[int(pixel_upper_border):height, :]

	def set_height_rel(self, roi_height_rel):
		if roi_height_rel > 0:
			self.__roi_height_rel = roi_height_rel

	def get_height_rel(self):
		return self.__roi_height_rel

class LaneTrackConfig:
	
	def __init__(self, ns='md_config/lane'):
		self.ns = ns
		self.roi_param_name = 'roi'
		self.config_filepath = expanduser("~") + '/.maddrive_config/lane_config.yaml'

	def save_params(self):
		rospy.loginfo('Saving parameters')
		if not os.path.exists(os.path.dirname(self.config_filepath)):
		    try:
		        os.makedirs(os.path.dirname(self.config_filepath))
		    except OSError as exc: # Guard against race condition
		        if exc.errno != errno.EEXIST:
		            raise

		rosparam.dump_params(self.config_filepath, self.ns, True)

	def load_params(self):
		try:
			rospy.loginfo('Loading parameters')
			rosparam.load_file(self.config_filepath, self.ns, True)
			return True
		except:
			rospy.logwarn('Failed to read data from config')
			return False

	def get_roi_descriptor(self):
		roi_param_value = self.__get_roi_param()
		if roi_param_value < 0:
			roi_param_value = 0.5		# Default
			rospy.loginfo('Using default value: {}'.format(roi_param_value))

		return ImageROI(roi_param_value)

	def set_roi_descriptor(self, roi_desc):
		roi_param_value = roi_desc.get_height_rel()
		self.__set_roi_param(roi_param_value)

	def __set_roi_param(self, roi_height_rel_value):
		rosparam.set_param(self.ns + '/' + self.roi_param_name, str(roi_height_rel_value))

	def __get_roi_param(self):
		try:
			result = rosparam.get_param(self.ns + '/' + self.roi_param_name)
			return result
		except:
			rospy.logwarn('Failed to get param from server')
			return -1
