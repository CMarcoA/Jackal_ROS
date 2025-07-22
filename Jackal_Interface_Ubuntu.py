#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import tkinter as tk
import threading
import time
from evdev import list_devices, InputDevice, ecodes

class RobotControlApp(tk.Tk):
    # Button colors for each gear: (normal, pressed)
    GEAR_COLORS = {
        1: ("#00FF00", "#009900"),
        2: ("#80FF00", "#669900"),
        3: ("#FFFF00", "#CCCC00"),
        4: ("#FF8000", "#CC6600"),
        5: ("#FF0000", "#990000"),
    }
    # Linear speeds (m/s) for each gear
    GEAR_SPEED = {
        1: 0.2,
        2: 0.4,
        3: 0.6,
        4: 0.8,
        5: 1.0,
    }

    def __init__(self):
        super().__init__()
        # Initialize ROS node and publisher
        rospy.init_node('jackal_gui_teleop', anonymous=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # TK window setup
        self.title("Jackal Interface (Standalone)")
        self.configure(bg="#333333")

        # Current gear
        self._gear = 1

        # Build UI and start controller listener
        self._build_widgets()
        self._refresh_buttons()
        self._start_controller_listener()

    def _build_widgets(self):
        # Emergency stop (reset to gear 1)
        stop_btn = tk.Button(
            self,
            text="EMERGENCY STOP",
            bg="red",
            fg="white",
            width=14,
            command=lambda: self._set_gear(1)
        )
        stop_btn.pack(side="top", anchor="ne", padx=10, pady=10)

        # Gear buttons 1–5
        btn_frame = tk.Frame(self, bg="#333333")
        btn_frame.pack(pady=10)
        self._buttons = []
        for gear in range(1, 6):
            normal, pressed = RobotControlApp.GEAR_COLORS[gear]
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
        # Update button colors/relief for current gear
        for idx, btn in enumerate(self._buttons, start=1):
            normal, pressed = RobotControlApp.GEAR_COLORS[idx]
            if self._gear == idx:
                btn.config(bg=pressed, relief="sunken", bd=4)
            else:
                btn.config(bg=normal, relief="raised", bd=2)

    def _set_gear(self, gear):
        # Change gear, update UI, and publish a Twist
        self._gear = gear
        self._refresh_buttons()

        twist = Twist()
        twist.linear.x = RobotControlApp.GEAR_SPEED[gear]
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def _start_controller_listener(self):
        # Locate the Xbox controller among input devices
        dev = None
        for path in list_devices():
            d = InputDevice(path)
            if "X-Box One pad" in d.name or "Controller" in d.name:
                dev = d
                print(f"→ Using controller: {d.name} @ {path}")
                break
        if not dev:
            print("Xbox controller not found. Make sure it's plugged in and the driver is loaded.")
            return

        # Start polling thread
        threading.Thread(target=self._poll_loop, args=(dev,), daemon=True).start()

    def _poll_loop(self, dev: InputDevice):
        # Listen for button presses to change gears
        for event in dev.read_loop():
            if event.type == ecodes.EV_KEY and event.value == 1:  # key down
                if event.code == ecodes.BTN_X and self._gear > 1:
                    self._set_gear(self._gear - 1)
                    time.sleep(0.2)  # debounce
                elif event.code == ecodes.BTN_B and self._gear < 5:
                    self._set_gear(self._gear + 1)
                    time.sleep(0.2)  # debounce

if __name__ == "__main__":
    app = RobotControlApp()
    app.mainloop()
