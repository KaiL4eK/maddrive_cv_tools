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

subscriber = rospy.Subscriber('/camera/rgb/image_color', Image, image_callback,  queue_size = 10)

visible_frame_size = (320, 240)

def main():
	global frame

	cv2.namedWindow(window_name)
	
	def nothing(x): pass

	config = LaneTrackConfig()
	config.load_params()
	roi_desc = config.get_roi_descriptor()

	cv2.createTrackbar(roi_trackbar_name, window_name, int(roi_desc.get_height_rel() * 100), 100, nothing)

	rospy.loginfo('Click "s" to save parameters')

	while not rospy.is_shutdown():
		if frame is not None:
			work_frame = np.copy(frame)

			work_frame = cv2.resize(work_frame, visible_frame_size)
			orig_height, orig_width = get_frame_height_width(work_frame)

			roi_y_rel = (cv2.getTrackbarPos(roi_trackbar_name, window_name) / 100.)
			roi_desc.set_height_rel(roi_y_rel)
			roi_frame = roi_desc.mask_frame_resize(work_frame)
			roi_desc.draw_height_border(work_frame)

			result_frame = np.hstack( (work_frame, roi_frame) );
			cv2.imshow(window_name, result_frame)
			if cv2.waitKey(1) == ord('s'):
				config.set_roi_descriptor(roi_desc)
				config.save_params()
				exit(0)

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
