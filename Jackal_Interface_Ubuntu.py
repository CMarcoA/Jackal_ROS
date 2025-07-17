#!/usr/bin/env python3
import time
import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class RobotControlApp:
    GEAR_COLORS = {
        1: ("#00FF00", "#009900"),
        2: ("#80FF00", "#669900"),
        3: ("#FFFF00", "#CCCC00"),
        4: ("#FF8000", "#CC6600"),
        5: ("#FF0000", "#990000"),
    }

    def __init__(self, master):
        # ROS setup
        rospy.init_node('jackal_gui')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # UI & state
        self._gear = 1
        self._last_shift = 0.0
        self._shift_cooldown = 1.0
        self._cap = cv2.VideoCapture(0)

        master.title("Jackal Interface (ROS)")
        master.configure(bg="#333333")
        self._build_widgets(master)
        self._update_video()

    def _build_widgets(self, master):
        self._stop_btn = tk.Button(master, text="EMERGENCY STOP",
                                    bg="red", fg="white", width=14,
                                    command=self._emergency_stop)
        self._stop_btn.pack(side="top", anchor="ne", padx=10, pady=10)

        btn_frame = ttk.Frame(master)
        btn_frame.pack(pady=10)
        for gear in range(1, 6):
            normal, pressed = RobotControlApp.GEAR_COLORS[gear]
            btn = tk.Button(btn_frame, text=str(gear),
                            bg=normal, fg="white" if gear>=4 else "black",
                            width=4, activebackground=pressed,
                            command=lambda g=gear: self._set_gear(g))
            btn.pack(side="left", padx=5)
            setattr(self, f"btn{gear}", btn)
            setattr(self, f"_norm{gear}", normal)
            setattr(self, f"_press{gear}", pressed)
        self._video_label = ttk.Label(master)
        self._video_label.pack(padx=10, pady=10)
        self._refresh_buttons()

    def _refresh_buttons(self):
        for gear in range(1, 6):
            btn = getattr(self, f"btn{gear}")
            if self._gear == gear:
                btn.config(bg=getattr(self, f"_press{gear}"), relief="sunken", bd=4)
            else:
                btn.config(bg=getattr(self, f"_norm{gear}"), relief="raised", bd=2)

    def _set_gear(self, g):
        now = time.monotonic()
        if now - self._last_shift >= self._shift_cooldown:
            self._gear = g
            self._last_shift = now
            self._refresh_buttons()

    def _emergency_stop(self):
        self._gear = 1
        self._refresh_buttons()
        # send zero Twist immediately
        twist = Twist()
        self.cmd_pub.publish(twist)

    def joy_callback(self, msg: Joy):
        # buttons[9] on most Xbox configs = L-stick click
        if msg.buttons[9]:
            self._emergency_stop()

        # bumpers for gear shift (RB=5, LB=4)
        if msg.buttons[5]:
            self._set_gear(min(self._gear+1, 5))
        if msg.buttons[4]:
            self._set_gear(max(self._gear-1, 1))

        # map left stick (axes[1]=forward/back, axes[0]=left/right) to /cmd_vel
        twist = Twist()
        max_speed = 0.2 * self._gear  # adjust scale per gear
        twist.linear.x  = msg.axes[1] * max_speed
        twist.angular.z = msg.axes[0] * max_speed
        self.cmd_pub.publish(twist)

    def _update_video(self):
        ret, frame = self._cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img   = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self._video_label.imgtk = imgtk
            self._video_label.config(image=imgtk)
        if not rospy.is_shutdown():
            self._video_label.after(30, self._update_video)

if __name__ == '__main__':
    root = tk.Tk()
    app  = RobotControlApp(root)d
    root.mainloop()
