#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Continuous (geared) teleop + camera GUI
# ROS Indigo / Python 2.7  |  Tkinter UI with global scaling
# Set UI_SCALE_DEFAULT below to scale the entire UI.

import Tkinter as tk
import tkFont
import rospy
from sensor_msgs.msg import Joy, Image, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage, ImageTk

# ---- Optional deps for compressed decode ----
try:
    import numpy as np
    import cv2
    try:
        cv2.setNumThreads(1)
    except Exception:
        pass
except Exception:
    np = None
    cv2 = None

# ---- Hard-coded UI scale (adjust here) ----
UI_SCALE_DEFAULT = 4.0    # <--- change this to make the whole UI bigger/smaller


def rgb(r, g, b):
    return '#%02x%02x%02x' % (r, g, b)


class ContinuousSpeedUI(tk.Tk):
    # min/max speed ranges per gear (used for continuous mapping)
    LEVELS = {
        1: (0.05, 0.20),
        2: (0.20, 0.40),
        3: (0.40, 0.60),
        4: (0.60, 0.80),
        5: (0.80, 1.00),
    }

    def __init__(self):
        tk.Tk.__init__(self)
        self.title("Jackal Interface (Geared Continuous)")
        self.configure(bg="#333333")

        # ---- ROS ----
        rospy.init_node('jackal_continuous_teleop', anonymous=True)

        # ---- Params (camera/control behavior) ----
        self.image_topic    = rospy.get_param('~image_topic', '/camera/image_raw')
        self.use_compressed = rospy.get_param('~use_compressed', False)
        self.flip_h         = rospy.get_param('~flip_horizontal', False)
        self.flip_v         = rospy.get_param('~flip_vertical', False)
        self.fast_decode    = rospy.get_param('~fast_decode', True)
        # 1(off), 2(half), 4(quarter) — used only when fast_decode is True
        self.reduced_decode = int(rospy.get_param('~reduced_decode', 2))

        # ---- UI scaling ----
        self.ui_scale = float(UI_SCALE_DEFAULT)
        self._apply_scaling()

        # ---- Pub/Sub ----
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1, tcp_nodelay=True)

        self.bridge = CvBridge()
        self.latest_frame = None

        if self.use_compressed:
            if (np is None) or (cv2 is None):
                rospy.logwarn("~use_compressed:=true but numpy/cv2 missing; video disabled.")
            else:
                rospy.Subscriber(self.image_topic, CompressedImage,
                                 self.image_callback_compressed, queue_size=1, buff_size=2**24)
                rospy.loginfo("Subscribed (compressed): %s", self.image_topic)
        else:
            rospy.Subscriber(self.image_topic, Image,
                             self.image_callback_raw, queue_size=1, buff_size=2**24)
            rospy.loginfo("Subscribed (raw): %s", self.image_topic)

        # ---- State ----
        self.gear = 1
        self.last_triggers = [0, 0]          # [RT, LT] edge-detect
        self.latest_axes = [0.0] * 8
        self._joy_y = 0.0                    # forward/back input (-1..1) for indicator

        # ---- GUI ----
        self._build_gui()

        # ---- Loops ----
        rospy.Timer(rospy.Duration(0.05), self.publish_twist)  # ~20 Hz control
        self.after(33, self.video_update)                      # ~30 FPS display
        self.after(100, self.tk_loop)

    # ---------- UI scaling ----------
    def _apply_scaling(self):
        S = max(0.5, float(self.ui_scale))
        try:
            self.tk.call('tk', 'scaling', S)
        except Exception:
            pass

        # fonts
        self.font_btn  = tkFont.Font(family="Helvetica", size=int(10 * S), weight="bold")
        self.font_misc = tkFont.Font(family="Helvetica", size=int(10 * S))

        # px helper + minimum window
        def _px(v): return int(round(v * S))
        self.px = _px
        self.minsize(self.px(380), self.px(320))

        # geometry derived from scale
        self.canvas_width  = self.px(24) * 4      # ~96 px at S=1; 384 at S=4
        self.canvas_height = self.px(100) * 4     # ~400 px at S=1; 1600 at S=4
        self.pad_outer     = self.px(10)
        self.pad_inner     = self.px(10)

    # ---------- GUI ----------
    def _build_gui(self):
        container = tk.Frame(self, bg="#333333")
        container.pack(padx=self.pad_outer, pady=self.pad_outer)

        # LEFT: vertical speed bar
        self.level_count = 5
        self.level_h = self.canvas_height // self.level_count

        self.speed_canvas = tk.Canvas(
            container, width=self.canvas_width, height=self.canvas_height,
            bg="lightgray", highlightthickness=1, highlightbackground="#999"
        )
        self.speed_canvas.pack(side="left", padx=(0, self.pad_inner))

        self._draw_gradient()
        self._draw_interval_lines()

        # Current-gear highlight
        self.level_highlight_id = self.speed_canvas.create_rectangle(
            1, 1, self.canvas_width - 1, self.level_h - 1, outline="#FFFFFF", width=self.px(1.5)
        )
        # Moving indicator line (forward fill)
        self.level_indicator_id = self.speed_canvas.create_line(
            0, self.canvas_height - self.px(2), self.canvas_width, self.canvas_height - self.px(2),
            fill="#000000", width=self.px(3)
        )

        self._update_level_highlight()
        self._reset_indicator_to_bottom()
        self._raise_overlays()

        # RIGHT: camera view (height-matched to bar)
        self.webcam_label = tk.Label(container, bg="#000000")
        self.webcam_label.pack(side="left")

    def _raise_overlays(self):
        self.speed_canvas.tag_raise(self.level_highlight_id)
        self.speed_canvas.tag_raise(self.level_indicator_id)

    def _draw_gradient(self):
        steps = self.canvas_height
        for i in range(steps):
            r = int((i / float(steps)) * 255)
            g = int((1 - i / float(steps)) * 255)
            color = rgb(r, g, 0)  # green(top) → red(bottom)
            self.speed_canvas.create_rectangle(0, i, self.canvas_width, i + 1,
                                               outline=color, fill=color)

    def _draw_interval_lines(self):
        for i in range(1, self.level_count):
            y = i * self.level_h
            self.speed_canvas.create_line(0, y, self.canvas_width, y, fill="black", width=self.px(1.5))

    def _gear_to_segment_bounds(self, gear):
        # gear 5 = top segment, gear 1 = bottom segment
        idx_from_top = self.level_count - gear
        y0 = idx_from_top * self.level_h
        y1 = y0 + self.level_h
        return y0, y1

    def _update_level_highlight(self):
        y0, y1 = self._gear_to_segment_bounds(self.gear)
        self.speed_canvas.coords(self.level_highlight_id,
                                 1, y0 + 1, self.canvas_width - 1, y1 - 1)
        self._raise_overlays()

    def _reset_indicator_to_bottom(self):
        y0, y1 = self._gear_to_segment_bounds(self.gear)
        margin = self.px(4)
        y = y1 - margin
        self.speed_canvas.coords(self.level_indicator_id, 0, y, self.canvas_width, y)
        self._raise_overlays()

    def _update_indicator_from_input(self):
        """
        Move indicator based on forward joystick only.
        Reverse doesn't move the fill line (stays at bottom of the box).
        """
        y0, y1 = self._gear_to_segment_bounds(self.gear)
        margin = self.px(4)
        usable = (y1 - y0) - 2 * margin
        forward_input = max(0.0, self._joy_y)          # 0..1; reverse -> 0
        frac = max(0.0, min(1.0, forward_input))       # clamp
        y = int((y1 - margin) - (usable * frac))       # bottom -> top
        self.speed_canvas.coords(self.level_indicator_id, 0, y, self.canvas_width, y)
        self._raise_overlays()

    def _set_gear(self, new_gear):
        self.gear = max(1, min(5, new_gear))
        self._update_level_highlight()
        self._reset_indicator_to_bottom()
        rospy.loginfo("Gear -> %d", self.gear)

    # ---------- ROS callbacks ----------
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
        self._joy_y = axes[1]  # store forward/back input (-1..1)

    def image_callback_raw(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.flip_h: bgr = bgr[:, ::-1]
            if self.flip_v: bgr = bgr[::-1, :]
            self.latest_frame = bgr[:, :, ::-1]  # RGB
        except CvBridgeError as e:
            rospy.logwarn("cv_bridge error: %s", e)

    def image_callback_compressed(self, msg):
        if (np is None) or (cv2 is None):
            return
        try:
            arr = np.fromstring(msg.data, np.uint8)
            flag = cv2.IMREAD_COLOR
            if self.fast_decode:
                if self.reduced_decode == 2:
                    flag = cv2.IMREAD_REDUCED_COLOR_2
                elif self.reduced_decode == 4:
                    flag = cv2.IMREAD_REDUCED_COLOR_4
            bgr = cv2.imdecode(arr, flag)
            if bgr is None:
                return
            if self.flip_h: bgr = cv2.flip(bgr, 1)
            if self.flip_v: bgr = cv2.flip(bgr, 0)
            self.latest_frame = bgr[:, :, ::-1]  # RGB
        except Exception as e:
            rospy.logwarn("compressed decode error: %s", e)

    # ---------- UI loops ----------
    def video_update(self):
        # move indicator inside current gear box
        self._update_indicator_from_input()

        # camera image (height-matched to bar)
        if self.latest_frame is not None:
            h = self.canvas_height
            img = PILImage.fromarray(self.latest_frame)
            w0, h0 = img.size
            w = int((float(w0) / float(h0)) * h)
            img = img.resize((max(1, w), h), PILImage.NEAREST)
            imgtk = ImageTk.PhotoImage(image=img)
            self.webcam_label.imgtk = imgtk
            self.webcam_label.configure(image=imgtk)

        self.after(33, self.video_update)

    def publish_twist(self, _evt):
        if not hasattr(self, 'latest_axes'):
            return
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
