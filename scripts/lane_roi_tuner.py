#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from lane_track_config import *
import numpy as np

# parser = argparse.ArgumentParser(description='Process picture with different color modes')
# parser.add_argument('-f', '--filepath', action='store', help='Path to image to process')
# parser.add_argument('-k', '--kinect', action='store_true', help='Get image from kinect camera')
# parser.add_argument('-v', '--video', action='store_true', help='Process video file')
# args = parser.parse_args()

def image_callback(ros_data):
	global frame
	try:
		frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")
	except CvBridgeError as e:
		rospy.logerror(e)

rospy.init_node('lane_roi_tuner')

window_name = 'roi'
roi_trackbar_name = 'roi_height'


#-------------------------------------------------

use_kinect_topic = rospy.get_param('~use_kinect_topic', False)
filepath = rospy.get_param('~filepath', '')

rospy.loginfo('[%s] Param <use_kinect_topic>: %s' % (os.path.basename(__file__), str(use_kinect_topic)))
rospy.loginfo('[%s] Param <filepath>: %s' % (os.path.basename(__file__), str(filepath)))

#-------------------------------------------------

bridge = CvBridge()

if use_kinect_topic:
	rospy.Subscriber('image', Image, image_callback,  queue_size = 10)

visible_frame_size = (320, 240)

def main():
	global frame

	frame = None

	if not use_kinect_topic:
		frame = cv2.imread(filepath)
		if frame is None:
			rospy.logerr('Unable to open file')
			exit(1)
	
	cv2.namedWindow(window_name)
	
	config = LaneTrackConfig()
	# config.load_params()
	roi_desc = config.get_roi_descriptor()

	def nothing(x): pass
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
