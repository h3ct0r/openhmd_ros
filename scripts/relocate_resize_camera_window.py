#!/usr/bin/python

import rospy
import Xlib
import Xlib.display

rospy.init_node("relocate_resize_window")

display = Xlib.display.Display()
root = display.screen().root

screen_width = rospy.get_param('current_screen_width', 3840)
cam_w = rospy.get_param('window_config_width', 2146)
cam_h = rospy.get_param('window_config_height', 1062)
openhmd_label = rospy.get_param('window_name', '/openhmd/stereo')

windowIDs = root.get_full_property(display.intern_atom('_NET_CLIENT_LIST'), Xlib.X.AnyPropertyType).value

found = False
for windowID in windowIDs:
	window = display.create_resource_object('window', windowID)
	name = window.get_wm_name()
	pid = window.get_full_property(display.intern_atom('_NET_WM_PID'), Xlib.X.AnyPropertyType) # PID
	if name == openhmd_label:
		found = True
		print "[DEBUG]", 'Window with label', openhmd_label,'found!'  
		window.configure(x=screen_width, y=0, width=cam_w, height=cam_h)
		display.sync()
		wdata = window.get_geometry()._data
		print "[DEBUG]", 'Window config', wdata  

if not found:
	print '[ERROR]', 'Window with label', openhmd_label, 'was not found.' 