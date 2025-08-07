#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Tkinter as tk
import rospy
from sensor_msgs.msg import Joy, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from PIL import Image as PILImage, ImageTk  # Pillow

def rgb(r, g, b):
    return '#%02x%02x%02x' % (r, g, b)

class ContinuousSpeedUI(tk.Tk):
    LEVELS = {
        1: (0.05, 0.2),
        2: (0.2, 0.4),
        3: (0.4, 0.6),
        4: (0.6, 0.8),
        5: (0.8, 1.0)
    }

    def __init__(self):
        tk.Tk.__init__(self)
        self.title("Jackal Interface (Geared Continuous - IPO2)")
        self.configure(bg="#333333")

        rospy.init_node('jackal_continuous_teleop', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Bridge + frame holder for ROS image -> Tk
        self.bridge = CvBridge()
        self.latest_frame = None  # numpy array (RGB)

        # Subscribe to your camera topic
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1)

        self.gear = 1
        self.last_triggers = [0, 0]
        self.latest_axes = [0.0] * 8

        self._build_gui()

        rospy.Timer(rospy.Duration(0.05), self.publish_twist)
        self.after(30, self.video_update)  # UI refresh for camera
        self.after(100, self.tk_loop)

    def _build_gui(self):
        container = tk.Frame(self, bg="#333333")
        container.pack(padx=20, pady=20)

        self.canvas_width = 100
        self.canvas_height = 400

        self.speed_canvas = tk.Canvas(container, width=self.canvas_width, height=self.canvas_height,
                                      bg="lightgray", highlightthickness=1, highlightbackground="#999")
        self.speed_canvas.pack(side="left", padx=(0, 20))

        self._draw_gradient()
        self._draw_interval_lines()

        # Where the ROS camera image will be shown
        self.webcam_label = tk.Label(container, bg="#000000")
        self.webcam_label.pack(side="left")

    def _draw_gradient(self):
        steps = self.canvas_height
        for i in range(steps):
            r = int((i / float(steps)) * 255)
            g = int((1 - i / float(steps)) * 255)
            color = rgb(r, g, 0)
            self.speed_canvas.create_rectangle(0, i, self.canvas_width, i+1, outline=color, fill=color)

    def _draw_interval_lines(self):
        levels = 5
        step = self.canvas_height // levels
        for i in range(1, levels):
            y = i * step
            self.speed_canvas.create_line(0, y, self.canvas_width, y, fill="black", width=2)

    def _set_gear(self, new_gear):
        self.gear = max(1, min(5, new_gear))
        rospy.loginfo("Gear changed to {}".format(self.gear))

    def joy_callback(self, msg):
        axes = msg.axes
        rt = axes[5] < 0.5
        lt = axes[2] < 0.5

        if rt and not self.last_triggers[0] and self.gear < 5:
            self._set_gear(self.gear + 1)
        self.last_triggers[0] = rt

        if lt and not self.last_triggers[1] and self.gear > 1:
            self._set_gear(self.gear - 1)
        self.last_triggers[1] = lt

        self.latest_axes = axes

    def image_callback(self, msg):
        """ROS image -> NumPy (RGB). Keep it lightweight; UI thread will render."""
        try:
            # Convert ROS image to BGR (OpenCV default), then to RGB
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb = bgr[:, :, ::-1]  # BGR -> RGB without needing cv2
            self.latest_frame = rgb
        except CvBridgeError as e:
            rospy.logwarn("cv_bridge error: %s", e)

    def video_update(self):
        """Run in Tk thread: convert latest_frame to ImageTk and display."""
        if self.latest_frame is not None:
            h = self.canvas_height
            # Keep aspect by scaling width based on height
            img = PILImage.fromarray(self.latest_frame)
            w0, h0 = img.size
            w = int( (float(w0) / float(h0)) * h )
            img = img.resize((max(1, w), h), PILImage.BILINEAR)
            imgtk = ImageTk.PhotoImage(image=img)
            self.webcam_label.imgtk = imgtk  # prevent GC
            self.webcam_label.configure(image=imgtk)

        self.after(30, self.video_update)  # ~33 FPS

    def publish_twist(self, event):
        if not hasattr(self, 'latest_axes'):
            return

        twist = Twist()
        y_axis = self.latest_axes[1]

        if abs(y_axis) > 0.05:
            min_speed, max_speed = self.LEVELS[self.gear]
            speed = ((max_speed - min_speed) * abs(y_axis)) + min_speed
            twist.linear.x = speed if y_axis > 0 else -speed
        else:
            twist.linear.x = 0.0

        twist.angular.z = self.latest_axes[3] * 1.0
        self.pub.publish(twist)

    def tk_loop(self):
        if not rospy.is_shutdown():
            self.after(100, self.tk_loop)

if __name__ == '__main__':
    try:
        app = ContinuousSpeedUI()
        app.mainloop()
    except rospy.ROSInterruptException:
        pass
