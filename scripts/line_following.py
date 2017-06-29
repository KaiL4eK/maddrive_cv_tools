#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from lane_track_config import *
import numpy as np
import argparse
from lane_search import *

parser = argparse.ArgumentParser(description='Process picture with different color modes')
# parser.add_argument('mode_prefix', action='store', help='Prefix of mode pack')
parser.add_argument('filepath', action='store', help='Path to image to process')
parser.add_argument('-v', '--video', action='store_true', help='Process video file')
args = parser.parse_args()

rospy.init_node('lane_follower')

window_name = 'result'

visible_frame_size = (640, 480)

def main():
	frame = cv2.imread(args.filepath)
	if frame is None:
		rospy.logerr('Unable to open file')
		exit(1)
	
	cv2.namedWindow(window_name)

	config = LaneTrackConfig()
	config.load_params()
	roi_desc = config.get_roi_descriptor()
	cf_desc = config.get_color_filter_descriptor()
	lane = LaneSearch()

	while not rospy.is_shutdown():
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

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
