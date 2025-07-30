#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import Tkinter as tk

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
        rospy.init_node('jackal_gear_teleop', anonymous=True)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.gear = 1
        self.last_triggers = [0, 0]  # [RT, LT]
        self.latest_axes = [0.0] * 8

        self._build_gui()
        self._refresh_buttons()

        rospy.Timer(rospy.Duration(0.05), self.publish_twist)
        self.after(100, self.tk_loop)

    def _build_gui(self):
        self.title("Jackal Interface (Geared Discrete)")
        self.configure(bg="#333333")

        stop_btn = tk.Button(
            self,
            text="EMERGENCY STOP",
            bg="red",
            fg="white",
            width=14,
            command=lambda: self._set_gear(1)
        )
        stop_btn.pack(side="top", anchor="ne", padx=10, pady=10)

        btn_frame = tk.Frame(self, bg="#333333")
        btn_frame.pack(pady=10)
        self._buttons = []
        for gear in range(1, 6):
            normal, pressed = self.GEAR_COLORS[gear]
            btn = tk.Button(
                btn_frame,
                text=str(gear),
                width=4,
                bg=normal,
                fg="white" if gear >= 4 else "black"
            )
            btn.pack(side="left", padx=5)
            self._buttons.append(btn)

    def _refresh_buttons(self):
        for idx, btn in enumerate(self._buttons, start=1):
            normal, pressed = self.GEAR_COLORS[idx]
            if self.gear == idx:
                btn.config(bg=pressed, relief="sunken", bd=4)
            else:
                btn.config(bg=normal, relief="raised", bd=2)

    def _set_gear(self, new_gear):
        self.gear = new_gear
        self._refresh_buttons()
        rospy.loginfo("Gear changed to {}".format(new_gear))

    def joy_callback(self, msg):
        axes = msg.axes

        rt_pressed = axes[5] < 0.5
        lt_pressed = axes[2] < 0.5

        if rt_pressed and not self.last_triggers[0] and self.gear < 5:
            self._set_gear(self.gear + 1)
        self.last_triggers[0] = rt_pressed

        if lt_pressed and not self.last_triggers[1] and self.gear > 1:
            self._set_gear(self.gear - 1)
        self.last_triggers[1] = lt_pressed

        self.latest_axes = axes

    def publish_twist(self, event):
        if not hasattr(self, 'latest_axes'):
            return

        twist = Twist()

        # Inverted linear axis so UP = forward
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
