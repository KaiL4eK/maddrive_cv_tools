#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from lane_track_config import *
import numpy as np
from lane_search import *

rospy.init_node('lane_follower')

#-------------------------------------------------

use_kinect_topic = rospy.get_param('~use_kinect_topic', False)
file_as_video = rospy.get_param('~file_as_video', False)
filepath = rospy.get_param('~filepath', '')

rospy.loginfo('[%s] Param <use_kinect_topic>: %s' % (os.path.basename(__file__), str(use_kinect_topic)))
rospy.loginfo('[%s] Param <file_as_video>: %s' % (os.path.basename(__file__), str(file_as_video)))
rospy.loginfo('[%s] Param <filepath>: %s' % (os.path.basename(__file__), str(filepath)))

#-------------------------------------------------

window_name = 'result'

visible_frame_size = (640, 480)

def main():
	if file_as_video:
		cap = cv2.VideoCapture(filepath)
		if not cap.isOpened():
			rospy.logerr('Unable to open video file')
			exit(1)

	else:
		original_image = cv2.imread(args.filepath)
		if original_image is None:
			rospy.logerr('Unable to open image file')
			exit(1)

	cv2.namedWindow(window_name)

	config = LaneTrackConfig()
	roi_desc = config.get_roi_descriptor()
	cf_desc = config.get_color_filter_descriptor()
	lane = LaneSearch()

	while not rospy.is_shutdown():
		ret, frame = cap.read()

		if frame is not None:
			work_frame = np.copy(frame)

			work_frame = cv2.resize(work_frame, visible_frame_size)
			orig_height, orig_width = get_frame_height_width(work_frame)
			roi_frame = roi_desc.mask_frame_resize(work_frame)
			bin_frame, _ = cf_desc.apply_filters(roi_frame, 10)

			left_fitx, right_fitx, fity = lane.get_lane_polynomials(bin_frame)

			for i in range(len(fity)):
				cv2.circle(roi_frame, (int(left_fitx[i]), int(fity[i])), 1, (0, 255, 0))
				cv2.circle(roi_frame, (int(right_fitx[i]), int(fity[i])), 1, (0, 0, 255))


			result_frame = np.hstack( (work_frame, roi_frame) );
			# cv2.imshow(window_name, result_frame)
			if cv2.waitKey(1) == ord('q'):
				exit(0)
		else:
			rospy.loginfo('Video closed or error')
			break

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
