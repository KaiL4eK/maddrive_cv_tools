#!/usr/bin/env python

import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from lane_track_config import *

from Tkinter import *
from PIL import Image
from PIL import ImageTk
import cv2

bridge = CvBridge()
confidence = 30
render_image_size = (320, 240)

def image_callback(ros_data):
	global original_image
	try:
		frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")
		original_image = cv2.resize(frame, render_image_size)
	except CvBridgeError as e:
		rospy.logerror(e)

	refresh_filter()

rospy.init_node('color_filter_tuner')

#-------------------------------------------------

use_kinect_topic = rospy.get_param('~use_kinect_topic', False)
file_as_video = rospy.get_param('~file_as_video', False)
filepath = rospy.get_param('~filepath', '')

rospy.loginfo('[%s] Param <use_kinect_topic>: %s' % (os.path.basename(__file__), str(use_kinect_topic)))
rospy.loginfo('[%s] Param <file_as_video>: %s' % (os.path.basename(__file__), str(file_as_video)))
rospy.loginfo('[%s] Param <filepath>: %s' % (os.path.basename(__file__), str(filepath)))

#-------------------------------------------------

config = LaneTrackConfig()
cf_desc = config.get_color_filter_descriptor()
roi_desc = config.get_roi_descriptor()
modeList = cf_desc.get_mode_list()

original_image = None

##############################################################

def refresh_filter():
	# grab a reference to the image panels
	global original_image_widget, thres_image_widget, output_image_widget
	global confidence
	
	if original_image is None:
		return

	roi_height_y = roi_desc.get_pixel_height(original_image.shape[0])
	roi_height_y = int(roi_height_y)

	roi_frame = roi_desc.mask_frame_resize(original_image)

	if current_mode is not None:
		filtered_img = current_mode.apply_filter(roi_frame, cf_desc.transform)
		filling = cf_desc.estimate_filling(filtered_img)
	else:
		filtered_img = np.zeros_like(roi_frame)
		filling = 0

	try:
		confidence = int(confidenceValue.get())
	except:
		pass

	filtered_img_full, filling_full = cf_desc.apply_filters(roi_frame, confidence)

	rgb_original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
	cv2.line(rgb_original_image, (0, roi_height_y), (rgb_original_image.shape[1], roi_height_y), thickness=2, color=(255, 255, 0))

	# convert the images to PIL format and then to ImageTk format
	orig_img = ImageTk.PhotoImage(Image.fromarray(rgb_original_image))
	bin_img = ImageTk.PhotoImage(Image.fromarray(filtered_img))
	res_img = ImageTk.PhotoImage(Image.fromarray(filtered_img_full))
	# if the panels are None, initialize them
	if original_image_widget is None or thres_image_widget is None or output_image_widget is None:
		original_image_widget = Label(image=orig_img)
		original_image_widget.image = orig_img
		original_image_widget.grid(row=1, column=0, columnspan=2, padx=5, pady=5)

		thres_image_widget = Label(image=bin_img)
		thres_image_widget.image = bin_img
		thres_image_widget.grid(row=1, column=2, columnspan=2, padx=5, pady=5)

		output_image_widget = Label(image=res_img)
		output_image_widget.image = res_img
		output_image_widget.grid(row=1, column=4, columnspan=2, padx=5, pady=5)

	# otherwise, update the image panels
	else:
		# update the pannels
		original_image_widget.configure(image=orig_img)
		original_image_widget.image = orig_img
		thres_image_widget.configure(image=bin_img)
		thres_image_widget.image = bin_img
		output_image_widget.configure(image=res_img)
		output_image_widget.image = res_img

	filling_var.set('Filling: {} / {} %'.format(filling, filling_full))

##############################################################

if use_kinect_topic:
	rospy.Subscriber('image', sensor_msgs.msg.Image, image_callback,  queue_size = 10)	

elif file_as_video:
	def video_trackbar(val):
		global original_image
		cap.set(cv2.CAP_PROP_POS_FRAMES, float(val))
		ret, new_frame = cap.read()
		if new_frame is not None:
			original_image = new_frame

		original_image = cv2.resize(original_image, render_image_size, interpolation = cv2.INTER_LINEAR)
		refresh_filter()

	cap = cv2.VideoCapture(filepath)
	frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT);
	video_slider = Scale(root, from_=0, to=frame_count, orient=HORIZONTAL, command = video_trackbar)
	video_slider.pack(side = TOP, fill = "both", expand="yes", padx=5, pady=5)

else:
	original_image = cv2.imread(args.filepath)
	if original_image is None:
		rospy.logerr('Unable to open file')
		exit(1)

	original_image = cv2.resize(original_image, render_image_size, interpolation = cv2.INTER_LINEAR)


# initialize the window toolkit along with the two image panels
root = Tk()
original_image_widget = None
thres_image_widget = None
output_image_widget = None

##############################################################

current_mode = None

modeListControlFrame = Frame(root)
modeListControlFrame.grid(row=0, column=7, rowspan=3)

modeListWidget = Listbox(modeListControlFrame)
modeListWidget.pack(fill = "both")

def updateModeListWidget():
	global current_mode, modeList

	modeListWidget.delete(0, END)

	modeList = cf_desc.get_mode_list()
	for i, mode in enumerate(modeList):
		modeListWidget.insert(i, 'Filter %d' % i)

	current_mode = None

def onSelect(evt):
	global current_mode
	# Note here that Tkinter passes an event object to onselect()
	w = evt.widget
	index = int(w.curselection()[0])
	current_mode = modeList[index]
	print('You selected item {}: {} / {}'.format(index, current_mode.get_mins(), current_mode.get_maxs()))

	first_param_frame_min.set(current_mode.get_mins()[0])
	first_param_frame_max.set(current_mode.get_maxs()[0])

	second_param_frame_min.set(current_mode.get_mins()[1])
	second_param_frame_max.set(current_mode.get_maxs()[1])

	third_param_frame_min.set(current_mode.get_mins()[2])
	third_param_frame_max.set(current_mode.get_maxs()[2])

	refresh_filter()

updateModeListWidget()
modeListWidget.bind('<<ListboxSelect>>', onSelect)	


##############################################################

def addMode_cb():
	cf_desc.new_mode()
	updateModeListWidget()

def removeMode_cb():
	if current_mode is None:
		return

	cf_desc.remove_mode(current_mode)
	updateModeListWidget()

def saveModes_cb():
	config.set_color_filter_descriptor(cf_desc)
	config.save_params()

def saveROI_cb():
	config.set_roi_descriptor(roi_desc)
	config.save_params()

def conf_cb():
	global confidence


addModeBtn = Button(modeListControlFrame, text="Add mode", command=addMode_cb)
addModeBtn.pack(side=BOTTOM, fill="both", expand="yes", padx=5, pady=5)

removeModeBtn = Button(modeListControlFrame, text="Remove mode", command=removeMode_cb)
removeModeBtn.pack(side=BOTTOM, fill="both", expand="yes", padx=5, pady=5)

saveModesBtn = Button(modeListControlFrame, text="Save modes", command=saveModes_cb)
saveModesBtn.pack(side=BOTTOM, fill="both", expand="yes", padx=5, pady=5)

saveROIBtn = Button(modeListControlFrame, text="Save ROI", command=saveROI_cb)
saveROIBtn.pack(side=BOTTOM, fill="both", expand="yes", padx=5, pady=5)

confidenceValue = Entry(modeListControlFrame, text="Confidence value")
confidenceValue.pack(side=BOTTOM, fill="both", expand="yes", padx=5, pady=5)
confidenceValue.insert(0, '0')

##############################################################

def slider_cb(val):
	if current_mode is not None:
		current_mode.range_minmaxs = ( first_param_frame_min.get(), second_param_frame_min.get(), third_param_frame_min.get(),
									   first_param_frame_max.get(), second_param_frame_max.get(), third_param_frame_max.get() )
	
	refresh_filter()

def roi_slider_cb(val):
	roi_desc.set_height_rel(float(val) / 100)
	refresh_filter()

# sliders_widget_frame = Frame(root, highlightbackground='red', highlightthickness=5)
# sliders_widget_frame.grid(row=3, column=0, columnspan=6, sticky='we')

roi_param_scale_y_rel = roi_desc.get_height_rel()
roi_param_scale = Scale(root, label='roi', from_=0, to=100, orient=HORIZONTAL, command = roi_slider_cb)
roi_param_scale.grid(row=2, column=0, columnspan=6, sticky='we', padx=5, pady=5)
roi_param_scale.set(roi_param_scale_y_rel*100)

first_param_frame_min = Scale(root, label='color min1', from_=0, to=180, orient=HORIZONTAL, command = slider_cb)
first_param_frame_min.grid(row=3, column=0, columnspan=3, sticky='we', padx=5, pady=5)

first_param_frame_max = Scale(root, label='color max1', from_=0, to=180, orient=HORIZONTAL, command = slider_cb)
first_param_frame_max.grid(row=3, column=3, columnspan=3, sticky='we', padx=5, pady=5)

second_param_frame_min = Scale(root, label='color min2', from_=0, to=255, orient=HORIZONTAL, command = slider_cb)
second_param_frame_min.grid(row=4, column=0, columnspan=3, sticky='we', padx=5, pady=5)

second_param_frame_max = Scale(root, label='color max2', from_=0, to=255, orient=HORIZONTAL, command = slider_cb)
second_param_frame_max.grid(row=4, column=3, columnspan=3, sticky='we', padx=5, pady=5)

third_param_frame_min = Scale(root, label='color min3', from_=0, to=255, orient=HORIZONTAL, command = slider_cb)
third_param_frame_min.grid(row=5, column=0, columnspan=3, sticky='we', padx=5, pady=5)

third_param_frame_max = Scale(root, label='color max3', from_=0, to=255, orient=HORIZONTAL, command = slider_cb)
third_param_frame_max.grid(row=5, column=3, columnspan=3, sticky='we', padx=5, pady=5)


##############################################################

filling_var = StringVar()
Label(root, textvariable = filling_var, font=("Helvetica", 16)).grid(row=0, column=0, columnspan=6)

##############################################################

# create a button, then when pressed, will trigger a file chooser
# dialog and allow the user to select an input image; then add the
# button the GUI
# btn = Button(root, text="Select an image", command=select_image)
# btn.pack(side="bottom", fill="both", expand="yes", padx=5, pady=5)
 
# kick off the GUI
root.mainloop()
