#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Tkinter as tk
import rospy
from sensor_msgs.msg import Joy, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage, ImageTk

class GearTeleopApp(tk.Tk):
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
        self.title("Jackal Interface (Geared Discrete)")
        self.configure(bg="#333333")

        rospy.init_node('jackal_gear_teleop', anonymous=True)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Camera subscribe
        self.bridge = CvBridge()
        self.latest_frame = None
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1)

        self.gear = 1
        self.last_triggers = [0, 0]  # [RT, LT]
        self.latest_axes = [0.0] * 8

        self._build_gui()
        self._refresh_buttons()

        rospy.Timer(rospy.Duration(0.05), self.publish_twist)  # 20 Hz
        self.after(30, self.video_tick)  # ~33 FPS
        self.after(100, self.tk_loop)

    def _build_gui(self):
        # Header row with STOP button
        top = tk.Frame(self, bg="#333333")
        top.pack(fill="x", pady=(10, 0))
        stop_btn = tk.Button(
            top, text="EMERGENCY STOP", bg="red", fg="white", width=14,
            command=lambda: self._set_gear(1)
        )
        stop_btn.pack(side="right", padx=10)

        # Middle row: left = gears, right = video
        mid = tk.Frame(self, bg="#333333")
        mid.pack(pady=10, padx=10)

        # Gear buttons frame (left)
        btn_frame = tk.Frame(mid, bg="#333333")
        btn_frame.pack(side="left", padx=(0, 20))
        self._buttons = []
        for gear in range(1, 6):
            normal, _pressed = self.GEAR_COLORS[gear]
            btn = tk.Button(
                btn_frame, text=str(gear), width=4, bg=normal,
                fg="white" if gear >= 4 else "black",
                command=lambda g=gear: self._set_gear(g)
            )
            btn.pack(side="left", padx=5)
            self._buttons.append(btn)

        # Video area (right)
        self.video_h = 240
        self.video_w = 320
        self.webcam_label = tk.Label(mid, bg="#000000", width=self.video_w, height=1)
        self.webcam_label.pack(side="left")

        # Footer spacer
        tk.Frame(self, height=10, bg="#333333").pack()

    def _refresh_buttons(self):
        for idx, btn in enumerate(self._buttons, start=1):
            normal, pressed = self.GEAR_COLORS[idx]
            if self.gear == idx:
                btn.config(bg=pressed, relief="sunken", bd=4)
            else:
                btn.config(bg=normal, relief="raised", bd=2)

    def _set_gear(self, new_gear):
        self.gear = max(1, min(5, new_gear))
        self._refresh_buttons()
        rospy.loginfo("Gear changed to {}".format(self.gear))

    # === ROS callbacks ===
    def joy_callback(self, msg):
        axes = msg.axes

        # These indices match many Logitech/Xbox mappings on Indigo; tweak if needed
        rt_pressed = axes[5] < 0.5
        lt_pressed = axes[2] < 0.5

        if rt_pressed and not self.last_triggers[0] and self.gear < 5:
            self._set_gear(self.gear + 1)
        self.last_triggers[0] = rt_pressed

        if lt_pressed and not self.last_triggers[1] and self.gear > 1:
            self._set_gear(self.gear - 1)
        self.last_triggers[1] = lt_pressed

        self.latest_axes = axes

    def image_callback(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # BGR -> RGB (no cv2 import required)
            rgb = bgr[:, :, ::-1]
            self.latest_frame = rgb
        except CvBridgeError as e:
            rospy.logwarn("cv_bridge error: %s", e)

    # === Tk update loops ===
    def video_tick(self):
        if self.latest_frame is not None:
            img = PILImage.fromarray(self.latest_frame)
            # Fit height; keep aspect
            w0, h0 = img.size
            target_h = self.video_h
            target_w = int((float(w0) / float(h0)) * target_h)
            img = img.resize((target_w, target_h), PILImage.BILINEAR)

            imgtk = ImageTk.PhotoImage(image=img)
            self.webcam_label.imgtk = imgtk  # prevent GC
            self.webcam_label.configure(image=imgtk)
        self.after(30, self.video_tick)

    def publish_twist(self, event):
        if not hasattr(self, 'latest_axes'):
            return

        twist = Twist()
        # Inverted linear axis so UP = forward (like your original)
        if self.latest_axes[1] > 0.5:
            twist.linear.x = self.GEAR_SPEEDS[self.gear]
        elif self.latest_axes[1] < -0.5:
            twist.linear.x = -self.GEAR_SPEEDS[self.gear]
        else:
            twist.linear.x = 0.0

        # Analog rotation
        twist.angular.z = self.latest_axes[3] * 1.0
        self.pub.publish(twist)

    def tk_loop(self):
        if not rospy.is_shutdown():
            self.after(100, self.tk_loop)

if __name__ == '__main__':
    try:
        app = GearTeleopApp()
        app.mainloop()
    except rospy.ROSInterruptException:
        pass
