#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy, Image
from geometry_msgs.msg import Twist
import Tkinter as tk

# for camera handling
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PIL import Image as PILImg, ImageTk

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
        # Initialize Tk and ROS
        tk.Tk.__init__(self)
        rospy.init_node('jackal_gear_teleop', anonymous=True)

        # Movement publisher & joystick subscriber
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Camera bridge & image subscriber
        self.bridge = CvBridge()
        self.latest_frame = None
        rospy.Subscriber(
            '/camera/image_color',  # debayered BGR8 stream
            Image,
            self.image_callback,
            queue_size=1
        )

        # Gear & joystick state
        self.gear = 1
        self.last_buttons = [0] * 12
        self.latest_axes   = [0.0] * 8

        # Build the GUI and start loops
        self._build_gui()
        self._refresh_buttons()

        rospy.Timer(rospy.Duration(0.05), self.publish_twist)
        self.after(100, self.tk_loop)

    def _build_gui(self):
        self.title("Jackal Interface (Integrated)")
        self.configure(bg="#333333")

        # Emergency stop button
        stop_btn = tk.Button(
            self,
            text="EMERGENCY STOP",
            bg="red",
            fg="white",
            width=14,
            command=lambda: self._set_gear(1)
        )
        stop_btn.pack(side="top", anchor="ne", padx=10, pady=10)

        # Gear buttons
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

        # Camera display area
        self.camera_label = tk.Label(self, bg="#222222")
        self.camera_label.pack(padx=10, pady=10)

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
        buttons = msg.buttons

        # B button (index 1) → gear up; X button (2) → gear down
        if buttons[1] == 1 and self.last_buttons[1] == 0 and self.gear < 5:
            self._set_gear(self.gear + 1)
        elif buttons[2] == 1 and self.last_buttons[2] == 0 and self.gear > 1:
            self._set_gear(self.gear - 1)

        self.latest_axes   = msg.axes
        self.last_buttons  = buttons

    def publish_twist(self, event):
        twist = Twist()
        # forward/backward on left stick Y; turn on right stick X
        twist.linear.x  = self.latest_axes[1] * self.GEAR_SPEEDS[self.gear]
        twist.angular.z = self.latest_axes[3] * 1.0
        self.pub.publish(twist)

    def image_callback(self, msg):
        try:
            # Convert ROS Image → OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Optional: downscale if too wide
            h, w = cv_image.shape[:2]
            if w > 320:
                cv_image = cv2.resize(cv_image, (320, int(h * 320.0 / w)))
            # BGR → RGB → PIL → PhotoImage
            rgb     = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_img = PILImg.fromarray(rgb)
            tk_img  = ImageTk.PhotoImage(pil_img)
            # Keep reference & let tk_loop display it
            self.latest_frame = tk_img
        except CvBridgeError as e:
            rospy.logwarn_throttle(5.0, 'CV bridge error: {}'.format(e))

    def tk_loop(self):
        # Update camera frame if available
        if self.latest_frame is not None:
            self.camera_label.config(image=self.latest_frame)
        # Continue the loop
        if not rospy.is_shutdown():
            self.after(30, self.tk_loop)

if __name__ == '__main__':
    try:
        app = GearTeleopApp()
        app.mainloop()
    except rospy.ROSInterruptException:
        pass
