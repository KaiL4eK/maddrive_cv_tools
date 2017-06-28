#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

from lane_track_config import *

import numpy as np

def image_callback(ros_data):
	global frame
	try:
		frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")
	except CvBridgeError as e:
		rospy.logerror(e)


window_name = 'roi'
roi_trackbar_name = 'roi_height'
frame = None

rospy.init_node('roi_tuner')

bridge = CvBridge()

config = LaneTrackConfig()
config.load_params()

subscriber = rospy.Subscriber('/camera/rgb/image_color', Image, image_callback,  queue_size = 10)

visible_frame_size = (320, 240)

def main():
	global frame

	cv2.namedWindow(window_name)
	
	def nothing(x): pass

	roi_y_perc = config.get_roi_param()
	if roi_y_perc < 0:
		roi_y_perc = 0.5
	cv2.createTrackbar(roi_trackbar_name, window_name, int(roi_y_perc * 100), 100, nothing)

	while not rospy.is_shutdown():
		if frame is not None:
			work_frame = np.copy(frame)

			work_frame = cv2.resize(work_frame, visible_frame_size)
			orig_height, orig_width, orig_chnls = work_frame.shape

			roi_y_perc = (cv2.getTrackbarPos(roi_trackbar_name, window_name) / 100.)
			roi_y = orig_height * roi_y_perc

			tmp_roi_frame = work_frame[int(roi_y):orig_height, :]
			if (tmp_roi_frame.shape[0] * tmp_roi_frame.shape[1]) > 0: 
				roi_frame = tmp_roi_frame
			roi_frame = cv2.resize(roi_frame, visible_frame_size)
			
			cv2.line(work_frame, (0, int(roi_y)), (orig_width, int(roi_y)), color=(255, 0, 0))

			# rospy.loginfo('{} {}'.format(work_frame.shape, roi_frame.shape))

			result_frame = np.hstack( (work_frame, roi_frame) );
			cv2.imshow(window_name, result_frame)
			if cv2.waitKey(1) == ord('s'):
				config.save_params()
				exit(0)

			config.set_roi_param(roi_y_perc)

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
