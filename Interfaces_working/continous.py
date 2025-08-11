#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Tkinter as tk
import rospy
from sensor_msgs.msg import Joy, Image, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage, ImageTk

# Optional deps for compressed decode
try:
    import numpy as np, cv2
except Exception:
    np = None; cv2 = None

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

        # Params
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.use_compressed = rospy.get_param('~use_compressed', False)
        self.flip_h = rospy.get_param('~flip_horizontal', False)
        self.flip_v         = rospy.get_param('~flip_vertical', False)

        # ROS
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.bridge = CvBridge()
        self.latest_frame = None

        if self.use_compressed:
            if (np is None) or (cv2 is None):
                rospy.logwarn("~use_compressed:=true but numpy/cv2 missing; video disabled.")
            rospy.Subscriber(self.image_topic, CompressedImage,
                             self.image_callback_compressed, queue_size=1, buff_size=2**24)
        else:
            rospy.Subscriber(self.image_topic, Image,
                             self.image_callback_raw, queue_size=1, buff_size=2**24)

        # State
        self.gear = 1
        self.last_triggers = [0, 0]
        self.latest_axes = [0.0] * 8
        self._joy_y = 0.0   # forward/back input (-1..1), used for indicator

        self._build_gui()

        rospy.Timer(rospy.Duration(0.05), self.publish_twist)   # 20 Hz control loop (as before)
        self.after(30, self.video_update)                       # ~33 FPS UI refresh
        self.after(100, self.tk_loop)

    def _build_gui(self):
        container = tk.Frame(self, bg="#333333")
        container.pack(padx=20, pady=20)

        # Bar on LEFT
        self.canvas_width = 100
        self.canvas_height = 400
        self.level_count = 5
        self.level_h = self.canvas_height // self.level_count

        self.speed_canvas = tk.Canvas(
            container, width=self.canvas_width, height=self.canvas_height,
            bg="lightgray", highlightthickness=1, highlightbackground="#999"
        )
        self.speed_canvas.pack(side="left", padx=(0, 20))

        self._draw_gradient()
        self._draw_interval_lines()

        # Highlight rectangle for CURRENT gear (ensure it's ABOVE everything)
        self.level_highlight_id = self.speed_canvas.create_rectangle(
            1, 1, self.canvas_width-1, self.level_h-1, outline="#FFFFFF", width=3
        )

        # Moving indicator line (black, thick) inside current gear box
        self.level_indicator_id = self.speed_canvas.create_line(
            0, self.canvas_height-8, self.canvas_width, self.canvas_height-8,
            fill="#000000", width=5
        )

        self._update_level_highlight()
        self._reset_indicator_to_bottom()
        self._raise_overlays()

        # Webcam to the RIGHT (height-matched)
        self.webcam_label = tk.Label(container, bg="#000000")
        self.webcam_label.pack(side="left")

    def _raise_overlays(self):
        # keep highlight + indicator always visible
        self.speed_canvas.tag_raise(self.level_highlight_id)
        self.speed_canvas.tag_raise(self.level_indicator_id)

    def _draw_gradient(self):
        steps = self.canvas_height
        for i in range(steps):
            r = int((i / float(steps)) * 255)
            g = int((1 - i / float(steps)) * 255)
            color = rgb(r, g, 0)  # green(top) → red(bottom)
            self.speed_canvas.create_rectangle(0, i, self.canvas_width, i+1,
                                               outline=color, fill=color)

    def _draw_interval_lines(self):
        for i in range(1, self.level_count):
            y = i * self.level_h
            self.speed_canvas.create_line(0, y, self.canvas_width, y, fill="black", width=2)

    def _gear_to_segment_bounds(self, gear):
        # gear 5 = top segment, gear 1 = bottom segment
        idx_from_top = self.level_count - gear
        y0 = idx_from_top * self.level_h
        y1 = y0 + self.level_h
        return y0, y1

    def _update_level_highlight(self):
        y0, y1 = self._gear_to_segment_bounds(self.gear)
        self.speed_canvas.coords(self.level_highlight_id,
                                 1, y0+1, self.canvas_width-1, y1-1)
        self._raise_overlays()

    def _reset_indicator_to_bottom(self):
        y0, y1 = self._gear_to_segment_bounds(self.gear)
        margin = 6  # offset from borders so line is visible
        y = y1 - margin
        self.speed_canvas.coords(self.level_indicator_id, 0, y, self.canvas_width, y)
        self._raise_overlays()

    def _update_indicator_from_input(self):
        """
        Move indicator based on *forward* joystick only.
        Reverse does NOT move the line (it stays at bottom of the interval).
        """
        y0, y1 = self._gear_to_segment_bounds(self.gear)
        margin = 6
        usable = (y1 - y0) - 2*margin
        # only forward contributes; backward leaves it at bottom
        forward_input = max(0.0, self._joy_y)           # 0..1, reverse -> 0
        frac = max(0.0, min(1.0, forward_input))        # clamp
        y = int((y1 - margin) - (usable * frac))        # bottom -> top within box
        self.speed_canvas.coords(self.level_indicator_id, 0, y, self.canvas_width, y)
        self._raise_overlays()

    def _set_gear(self, new_gear):
        self.gear = max(1, min(5, new_gear))
        self._update_level_highlight()
        self._reset_indicator_to_bottom()
        rospy.loginfo("Gear changed to {}".format(self.gear))

    # ROS callbacks
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
        self._joy_y = axes[1]  # store raw forward/back input (-1..1)

    def image_callback_raw(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.flip_h: bgr = bgr[:, ::-1]
            if self.flip_v: bgr = bgr[::-1, :]
            self.latest_frame = bgr[:, :, ::-1]
        except CvBridgeError as e:
            rospy.logwarn("cv_bridge error: %s", e)

    def image_callback_compressed(self, msg):
        if (np is None) or (cv2 is None):
            return
        try:
            arr = np.fromstring(msg.data, np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None: return
            if self.flip_h: bgr = cv2.flip(bgr, 1)
            if self.flip_v: bgr = cv2.flip(bgr, 0)
            self.latest_frame = bgr[:, :, ::-1]
        except Exception as e:
            rospy.logwarn("compressed decode error: %s", e)

    # Tk/UI loop
    def video_update(self):
        # Move indicator (forward-only) inside current gear box
        self._update_indicator_from_input()

        # Camera image (height-matched to bar)
        if self.latest_frame is not None:
            h = self.canvas_height
            img = PILImage.fromarray(self.latest_frame)
            w0, h0 = img.size
            w = int((float(w0) / float(h0)) * h)
            img = img.resize((max(1, w), h), PILImage.BILINEAR)
            imgtk = ImageTk.PhotoImage(image=img)
            self.webcam_label.imgtk = imgtk
            self.webcam_label.configure(image=imgtk)

        self.after(30, self.video_update)

    def publish_twist(self, event):
        if not hasattr(self, 'latest_axes'): return
        twist = Twist()
        y_axis = self.latest_axes[1]

        # continuous speed within current gear interval
        if abs(y_axis) > 0.05:
            min_s, max_s = self.LEVELS[self.gear]
            speed = ((max_s - min_s) * abs(y_axis)) + min_s
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
