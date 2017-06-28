#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from lane_track_config import *

from Tkinter import *
from PIL import Image
from PIL import ImageTk
import cv2


import argparse

parser = argparse.ArgumentParser(description='Process picture with different color modes')
# parser.add_argument('mode_prefix', action='store', help='Prefix of mode pack')
parser.add_argument('filepath', action='store', help='Path to image to process')
parser.add_argument('-v', '--video', action='store_true', help='Process video file')
args = parser.parse_args()

rospy.init_node('color_filter_tuner')

render_image_size = (320, 240)

# initialize the window toolkit along with the two image panels
root = Tk()
original_image_widget = None
thres_image_widget = None
output_image_widget = None

##############################################################

config = LaneTrackConfig()
config.load_params()
cf_desc = config.get_color_filter_descriptor()
roi_desc = config.get_roi_descriptor()

modeList = cf_desc.get_mode_list()

# color_filter = ColorFilter()
current_mode = None


modeListControlFrame = Frame(root)
modeListControlFrame.pack(side = RIGHT, fill = "both")

modeListWidget = Listbox(modeListControlFrame)

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
modeListWidget.pack(fill = "both")


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


addModeBtn = Button(modeListControlFrame, text="Add mode", command=addMode_cb)
addModeBtn.pack(side=BOTTOM, fill="both", expand="yes", padx=5, pady=5)

removeModeBtn = Button(modeListControlFrame, text="Remove mode", command=removeMode_cb)
removeModeBtn.pack(side=BOTTOM, fill="both", expand="yes", padx=5, pady=5)

saveModesBtn = Button(modeListControlFrame, text="Save modes", command=saveModes_cb)
saveModesBtn.pack(side=BOTTOM, fill="both", expand="yes", padx=5, pady=5)

##############################################################

def refresh_filter():
	# grab a reference to the image panels
	global original_image_widget, thres_image_widget, output_image_widget
	
	roi_frame = roi_desc.mask_frame_resize(original_image)

	if current_mode is not None:
		filtered_img = current_mode.apply_filter(roi_frame, cf_desc.transform)
		filling = cf_desc.estimate_filling(filtered_img)
	else:
		filtered_img = np.zeros_like(roi_frame)
		filling = 0

	filtered_img_full, filling_full = cf_desc.apply_filters(roi_frame, 30)

	rgb_original_image = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2RGB)

	# convert the images to PIL format and then to ImageTk format
	orig_img = ImageTk.PhotoImage(Image.fromarray(rgb_original_image))
	bin_img = ImageTk.PhotoImage(Image.fromarray(filtered_img))
	res_img = ImageTk.PhotoImage(Image.fromarray(filtered_img_full))
	# if the panels are None, initialize them
	if original_image_widget is None or thres_image_widget is None or output_image_widget is None:
		original_image_widget = Label(image=orig_img)
		original_image_widget.image = orig_img
		original_image_widget.pack(side=LEFT, padx=5, pady=5)

		thres_image_widget = Label(image=bin_img)
		thres_image_widget.image = bin_img
		thres_image_widget.pack(side=LEFT, padx=5, pady=5)

		output_image_widget = Label(image=res_img)
		output_image_widget.image = res_img
		output_image_widget.pack(side=RIGHT, padx=5, pady=5)

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

def slider_cb(val):

	if current_mode is not None:
		current_mode.range_minmaxs = (
										first_param_frame_min.get(), second_param_frame_min.get(), third_param_frame_min.get(),
										first_param_frame_max.get(), second_param_frame_max.get(), third_param_frame_max.get() 
									)
	
	refresh_filter()

sliders_widget_frame = Frame(root)
sliders_widget_frame.pack(side = BOTTOM, fill = "both")

third_param_frame = Frame(sliders_widget_frame)
third_param_frame.pack(side = BOTTOM, fill = "both")

third_param_frame_min = Scale(third_param_frame, from_=0, to=255, orient=HORIZONTAL, command = slider_cb)
third_param_frame_min.pack(side = LEFT, fill = "both", expand="yes", padx=5, pady=5)

third_param_frame_max = Scale(third_param_frame, from_=0, to=255, orient=HORIZONTAL, command = slider_cb)
third_param_frame_max.pack(side = RIGHT, fill = "both", expand="yes", padx=5, pady=5)

second_param_frame = Frame(sliders_widget_frame)
second_param_frame.pack(side = BOTTOM, fill = "both")

second_param_frame_min = Scale(second_param_frame, from_=0, to=255, orient=HORIZONTAL, command = slider_cb)
second_param_frame_min.pack(side = LEFT, fill = "both", expand="yes", padx=5, pady=5)

second_param_frame_max = Scale(second_param_frame, from_=0, to=255, orient=HORIZONTAL, command = slider_cb)
second_param_frame_max.pack(side = RIGHT, fill = "both", expand="yes", padx=5, pady=5)

first_param_frame = Frame(sliders_widget_frame)
first_param_frame.pack(side = BOTTOM, fill = "both")

first_param_frame_min = Scale(first_param_frame, from_=0, to=180, orient=HORIZONTAL, command = slider_cb)
first_param_frame_min.pack(side = LEFT, fill = "both", expand="yes", padx=5, pady=5)

first_param_frame_max = Scale(first_param_frame, from_=0, to=180, orient=HORIZONTAL, command = slider_cb)
first_param_frame_max.pack(side = RIGHT, fill = "both", expand="yes", padx=5, pady=5)

##############################################################

filling_var = StringVar()
Label(root, textvariable = filling_var, font=("Helvetica", 16)).pack(side = TOP)

##############################################################

if args.video:
	def video_trackbar(val):
		global original_image
		cap.set(cv2.CAP_PROP_POS_FRAMES, float(val))
		ret, new_frame = cap.read()
		if new_frame is not None:
			original_image = new_frame

		original_image = cv2.resize(original_image, render_image_size, interpolation = cv2.INTER_LINEAR)
		refresh_filter()

	cap = cv2.VideoCapture(args.filepath)
	frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT);
	video_slider = Scale(root, from_=0, to=frame_count, orient=HORIZONTAL, command = video_trackbar)
	video_slider.pack(side = TOP, fill = "both", expand="yes", padx=5, pady=5)
else:
	original_image = cv2.imread(args.filepath)
	original_image = cv2.resize(original_image, render_image_size, interpolation = cv2.INTER_LINEAR)


# create a button, then when pressed, will trigger a file chooser
# dialog and allow the user to select an input image; then add the
# button the GUI
# btn = Button(root, text="Select an image", command=select_image)
# btn.pack(side="bottom", fill="both", expand="yes", padx=5, pady=5)
 
# kick off the GUI
root.mainloop()
