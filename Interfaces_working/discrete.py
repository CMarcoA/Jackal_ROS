#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Discrete teleop + camera GUI (ROS Indigo / Python 2.7)
# Video below the gear buttons; camera pipeline optimized for higher FPS.

import Tkinter as tk
import rospy
from sensor_msgs.msg import Joy, Image, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage, ImageTk

# Optional deps for compressed image decode
#
try:
    import numpy as np
    import cv2
    try:
        # Can reduce jitter on some VMs
        cv2.setNumThreads(1)
    except Exception:
        pass
except Exception:
    np = None
    cv2 = None


class XboxTeleopApp(tk.Tk):
    GEAR_SPEEDS = {1: 0.2, 2: 0.4, 3: 0.6, 4: 0.8, 5: 1.0}
    GEAR_COLORS = {
        1: ("#00FF00", "#009900"),
        2: ("#80FF00", "#669900"),
        3: ("#FFFF00", "#CCCC00"),
        4: ("#FF8000", "#CC6600"),
        5: ("#FF0000", "#990000"),
    }

    def __init__(self):
        tk.Tk.__init__(self)
        self.title("Jackal Interface (Teleop + Camera)")
        self.configure(bg="#333333")

        # ---- ROS node ----
        rospy.init_node('xbox_teleop', anonymous=True)

        # ---- Params ----
        self.image_topic     = rospy.get_param('~image_topic', '/camera/image_raw')
        self.use_compressed  = rospy.get_param('~use_compressed', False)
        self.flip_h          = rospy.get_param('~flip_horizontal', False)
        self.flip_v          = rospy.get_param('~flip_vertical', False)

        # Performance toggles (client-side)
        self.fast_decode     = rospy.get_param('~fast_decode', True)   # enable reduced-res JPEG decode
        self.reduced_decode  = int(rospy.get_param('~reduced_decode', 2))  # 1(off), 2(half), 4(quarter)

        # ---- Pub/Sub ----
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1, tcp_nodelay=True)

        self.bridge = CvBridge()
        self.latest_axes = [0.0] * 8
        self.latest_frame = None
        self.gear = 1
        self.last_triggers = [0, 0]  # [RT, LT]

        # Camera subscribe with big buffer, drop old frames (queue_size=1)
        if self.use_compressed:
            if np is None or cv2 is None:
                rospy.logwarn("~use_compressed:=true but numpy/cv2 not found; video disabled.")
            rospy.Subscriber(self.image_topic, CompressedImage,
                             self.image_callback_compressed, queue_size=1, buff_size=2**24)
            rospy.loginfo("Subscribed (compressed): %s", self.image_topic)
        else:
            rospy.Subscriber(self.image_topic, Image,
                             self.image_callback_raw, queue_size=1, buff_size=2**24)
            rospy.loginfo("Subscribed (raw): %s", self.image_topic)

        # ---- GUI ----
        self._build_gui()
        self._refresh_buttons()

        # Loops
        rospy.Timer(rospy.Duration(0.016), self.publish_twist)  # ~60 Hz control
        self.after(33, self.video_tick)                         # ~30 FPS display
        self.after(150, self._sync_video_width)                 # fit video width to gear row
        self.after(100, self.tk_loop)

    # ---------- GUI ----------
    def _build_gui(self):
        # Top bar with STOP
        top = tk.Frame(self, bg="#333333")
        top.pack(fill="x", pady=(10, 0))
        tk.Button(top, text="EMERGENCY STOP", bg="red", fg="white", width=14,
                  command=lambda: self._set_gear(1)).pack(side="right", padx=10)

        # Middle container
        mid = tk.Frame(self, bg="#333333")
        mid.pack(pady=10, padx=10)

        # Row 1: Gear buttons
        self.btn_frame = tk.Frame(mid, bg="#333333")
        self.btn_frame.pack()
        self._buttons = []
        for g in range(1, 6):
            normal, _ = self.GEAR_COLORS[g]
            b = tk.Button(self.btn_frame, text=str(g), width=4, bg=normal,
                          fg="white" if g >= 4 else "black",
                          command=lambda gg=g: self._set_gear(gg))
            b.pack(side="left", padx=5)
            self._buttons.append(b)

        # Row 2: Video display (below)
        self.video_label = tk.Label(mid, bg="#000000")
        self.video_label.pack(pady=(10, 0))

    def _refresh_buttons(self):
        for idx, b in enumerate(self._buttons, start=1):
            normal, pressed = self.GEAR_COLORS[idx]
            if self.gear == idx:
                b.config(bg=pressed, relief="sunken", bd=4)
            else:
                b.config(bg=normal, relief="raised", bd=2)

    def _set_gear(self, g):
        self.gear = max(1, min(5, g))
        self._refresh_buttons()
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

    def image_callback_raw(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.flip_h: bgr = bgr[:, ::-1]
            if self.flip_v: bgr = bgr[::-1, :]
            self.latest_frame = bgr[:, :, ::-1]  # RGB
        except CvBridgeError as e:
            rospy.logwarn("cv_bridge error: %s", e)

    def image_callback_compressed(self, msg):
        if np is None or cv2 is None:
            return
        try:
            arr = np.fromstring(msg.data, np.uint8)

            # Reduced-resolution decode for speed (fast path)
            flag = cv2.IMREAD_COLOR
            if self.fast_decode:
                if self.reduced_decode == 2:
                    flag = cv2.IMREAD_REDUCED_COLOR_2  # half-size
                elif self.reduced_decode == 4:
                    flag = cv2.IMREAD_REDUCED_COLOR_4  # quarter-size

            bgr = cv2.imdecode(arr, flag)
            if bgr is None:
                return
            if self.flip_h: bgr = cv2.flip(bgr, 1)
            if self.flip_v: bgr = cv2.flip(bgr, 0)
            self.latest_frame = bgr[:, :, ::-1]  # RGB
        except Exception as e:
            rospy.logwarn("compressed decode error: %s", e)

    # ---------- Layout helpers ----------
    def _sync_video_width(self):
        self.update_idletasks()
        self.video_target_w = self.btn_frame.winfo_width()

    def _resize_to_target(self, pil_img):
        w0, h0 = pil_img.size
        tw = getattr(self, 'video_target_w', None)
        if not tw or tw <= 1:
            return pil_img
        th = int((float(h0) / float(w0)) * tw)
        # NEAREST is faster than BILINEAR for preview
        return pil_img.resize((tw, th), PILImage.NEAREST)

    # ---------- Loops ----------
    def video_tick(self):
        if self.latest_frame is not None:
            img = PILImage.fromarray(self.latest_frame)  # NumPy (RGB) → PIL
            img = self._resize_to_target(img)           # fit to gear width
            imgtk = ImageTk.PhotoImage(image=img)       # PIL → Tk image
            self.video_label.imgtk = imgtk              # prevent GC
            self.video_label.configure(image=imgtk)
        self.after(33, self.video_tick)

    def publish_twist(self, _evt):
        if not hasattr(self, 'latest_axes'):
            return
        t = Twist()
        # Discrete forward/back on left stick Y
        if self.latest_axes[1] > 0.5:
            t.linear.x = self.GEAR_SPEEDS[self.gear]
        elif self.latest_axes[1] < -0.5:
            t.linear.x = -self.GEAR_SPEEDS[self.gear]
        else:
            t.linear.x = 0.0
        # Analog turn on right stick X
        t.angular.z = self.latest_axes[3] * 1.0
        self.cmd_pub.publish(t)

    def tk_loop(self):
        if not rospy.is_shutdown():
            self.after(100, self.tk_loop)


if __name__ == '__main__':
    try:
        app = XboxTeleopApp()
        app.mainloop()
    except rospy.ROSInterruptException:
        pass
