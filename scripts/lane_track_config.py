#!/usr/bin/env python

import os
import errno
import cv2
import rospy
import rosparam
import rospkg
rospack = rospkg.RosPack()
import numpy as np
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

class ColorFilterMode:
	def __init__(self, range_minmaxs):
		self.range_minmaxs = range_minmaxs

	def get_mins(self):
		return self.range_minmaxs[0:3]

	def get_maxs(self):
		return self.range_minmaxs[3:6]

	def apply_filter(self, frame, transform):
		transformed_image = cv2.cvtColor(frame, transform)

		result_image = cv2.inRange(transformed_image, tuple(self.get_mins()), tuple(self.get_maxs()))
		return result_image

class ColorFilter:
	def __init__(self, transform):
		self.transform = transform
		self.mode_list = []

	def new_mode(self, minmaxs=[0, 0, 0, 180, 255, 255]):
		new_mode = ColorFilterMode(minmaxs)
		self.mode_list.append(new_mode)

	def remove_mode(self, mode):
		self.mode_list.remove(mode)

	def get_mode_list(self):
		return self.mode_list

	def set_mode_list(self, mode_list):
		self.mode_list = mode_list

	def get_minmaxs_list(self):
		result_list = []
		for mode in self.mode_list:
			result_list.append(list(mode.range_minmaxs))

		return result_list

	def estimate_filling (self, bin_frame):
		height, width = get_frame_height_width(bin_frame)
		filling_perc = int(np.sum(bin_frame) / 255. / (height * width) * 100)
		return filling_perc

	def apply_filters(self, frame, confidence):
		height, width = get_frame_height_width(frame)
		result_frame = np.zeros((height, width), dtype=np.uint8)

		for mode in self.mode_list:
			result_mode = mode.apply_filter(frame, self.transform)
			conf = self.estimate_filling(result_mode)

			if conf < confidence:
				result_frame = cv2.bitwise_or(result_mode, result_frame)

		return result_frame, self.estimate_filling(result_frame)

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

	def mask_frame_resize(self, frame):
		height, width = get_frame_height_width(frame)
		pixel_upper_border = self.__get_pixel_height(height)

		result = frame[int(pixel_upper_border):height, :]
		return cv2.resize(result, (width, height))

	def set_height_rel(self, roi_height_rel):
		if roi_height_rel > 0:
			self.__roi_height_rel = roi_height_rel

	def get_height_rel(self):
		return self.__roi_height_rel

class LaneTrackConfig:
	
	def __init__(self, ns='md_config/lane'):
		self.ns = ns

		self.roi_param_name = self.ns + '/roi'

		self.color_filter_transform_name = 'color_filter/transform'
		self.color_filter_minmaxs_name   = 'color_filter/minmaxs'

		self.color_filter_transform_path = self.ns + '/' + self.color_filter_transform_name
		self.color_filter_minmaxs_path   = self.ns + '/' + self.color_filter_minmaxs_name

		self.config_filepath = rospack.get_path('maddrive_cv_tools') + '/config/lane_config.yaml'

	def save_params(self):
		rospy.loginfo('Saving parameters')
		if not os.path.exists(os.path.dirname(self.config_filepath)):
		    try:
		        os.makedirs(os.path.dirname(self.config_filepath))
		    except OSError as exc: # Guard against race condition
		        if exc.errno != errno.EEXIST:
		            raise

		rosparam.dump_params(self.config_filepath, self.ns, True)

	## TODO!!!
	def __load_params(self):
		try:
			rospy.loginfo('Loading parameters')
			res = rosparam.load_file(self.config_filepath, self.ns, True)[0]
			rospy.loginfo('Loading done')
			return True
		except:
			rospy.logwarn('Failed to read data from config')
			return False



	def get_roi_descriptor(self):
		roi_param_value = self.__get_param(self.roi_param_name)
		if roi_param_value is None:
			roi_param_value = 0.5		# Default
			rospy.loginfo('Using default value: {}'.format(roi_param_value))

		return ImageROI(roi_param_value)

	def set_roi_descriptor(self, roi_desc):
		roi_param_value = roi_desc.get_height_rel()
		self.__set_param(self.roi_param_name, roi_param_value)


	def get_color_filter_descriptor(self):
		cf_tranform = self.__get_param(self.color_filter_transform_path)
		if cf_tranform is None:
			cf_tranform = cv2.COLOR_BGR2HSV

		cf_minmaxs = self.__get_param(self.color_filter_minmaxs_path)
		if cf_minmaxs is None:
			cf_minmaxs = []

		print(cf_minmaxs)

		cf_desc = ColorFilter(cf_tranform)
		for minmaxs in cf_minmaxs:
			cf_desc.new_mode(minmaxs)

		return cf_desc

	def set_color_filter_descriptor(self, cf_desc):
		self.__set_param(self.color_filter_transform_path, cf_desc.transform)
		self.__set_param(self.color_filter_minmaxs_path, cf_desc.get_minmaxs_list())

	def __set_param(self, name, value):
		rosparam.set_param(name, str(value))

	def __get_param(self, name):
		try:
			result = rosparam.get_param(name)
			return result
		except:
			rospy.logwarn('Failed to get param %s from server' % name)
			return None
