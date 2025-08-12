#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Unified teleop + camera GUI for ROS Indigo / Python 2.7
# Video panel BELOW the gear buttons, width matches gear row

import Tkinter as tk
import rospy
from sensor_msgs.msg import Joy, Image, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage, ImageTk

# Optional deps for compressed image decode
try:
    import numpy as np, cv2
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

        # ---- ROS Node initializing ----
        rospy.init_node('xbox_teleop', anonymous=True)

        # ---- Read Parameters for Camera stuff FROM ROS ----
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.use_compressed = rospy.get_param('~use_compressed', False)
        self.flip_h = rospy.get_param('~flip_horizontal', False)
        self.flip_v = rospy.get_param('~flip_vertical', False)
        """ 
        Connects to: image_callback_raw() and image_callback_compressed()
        these will use the chosen topic name
        """
        
        # ----  Publisher and Subcriber  ----
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Connects to: publish_twist() - fill a Twist message THEN send it out using this publisher

        rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1, tcp_nodelay=True)
        # Listens to /joy, calls joy_callback() every time a Joy message arrives

        # ---- Setting up image Converter (ROS -> OpenCV) ----
        self.bridge = CvBridge()

        self.latest_axes = [0.0] * 8
        # most recent joystick axes values from /joy (from subscriber)
        # rospy.Subscriber -> joy_callback -> latest_axes

        self.latest_frame = None
        # most recent camera image (from camera callback)

        self.gear = 1

        self.last_triggers = [0, 0]  # [RT, LT]
        # also from joy_callback

        # Subscribe to the camera topic (compressed or raw)
        if self.use_compressed:
            if np is None or cv2 is None:
                rospy.logwarn("~use_compressed:=true but numpy/cv2 not found; video disabled.")
            rospy.Subscriber(self.image_topic,CompressedImage,
                             self.image_callback_compressed,
                             queue_size=1)
            rospy.loginfo("Subscribed (compressed): %s", self.image_topic)
        else:
            rospy.Subscriber(self.image_topic, Image,self.image_callback_raw, queue_size=1)
            rospy.loginfo("Subscribed (raw): %s", self.image_topic)
        """
        1. Listen to the camera topic (name stored in self.image_topic)
        2. If camera sends JPEG style compressed image, use image_callback_compressed
        3. If send raw pixel data, use image_callback_raw
            - Where self.use_compressed come from?
              self.use_compressed = rospy.get_param('~use_compressed', False)
              - which reads a ROS parameter (we set if we want compressed / uncompressed)
              - default is false (raw)
        """
        """
        rospy.Subscribers role : rospy.Subscriber(topic_name, message_type, callback_function, queue_size)
        topic name : where to listen
        message type : what kind of data the topic sends
        - image : raw pixel array
        - CompressedImage : JPEG byte
        callback function : what function to call when a message arrives
        queue size = 1 : keep the latest frame ONLY

        """

        # ---- GUI call ----
        self._build_gui()
        self._refresh_buttons()

        # Loops (publish at ~60 Hz)
        # Start timers / loops for control, video
        rospy.Timer(rospy.Duration(0.016), self.publish_twist) # 60Hz driving loop
        self.after(33, self.video_tick)  # 30 FPS display
        self.after(150, self._sync_video_width)  # adjust video size to match gear row
        self.after(100, self.tk_loop) # keep Tk running with ROS

    # ---------- MAIN GUI ----------
    def _build_gui(self):
        """ Builds all visible widgets: 
            1. STOP button
            2. gear buttons (1–5)
            3. empty video area
            How: 
            Creates frames/labels/buttons each gear button calls _set_gear(gear)
            Connects:
            a.Buttons -> _set_gear() changes current gear -> _refresh_buttons() repaints buttons
            b.self.video_label updated by video_tick()
        """
        # Top bar with STOP
        top = tk.Frame(self, bg="#333333")
        top.pack(fill="x", pady=(10, 0))
        tk.Button(top, # parent container
                  text="EMERGENCY STOP", 
                  bg="red", fg="white", # background & text colours 
                  width=14, # button width to run when clicked
                  command=lambda: self._set_gear(1)).pack(side="right", padx=10)
        """
        a. command= "what function to run when clicked"
        b. lambda: self._set_gear(1) calls your method with 1
            (lambda used to pass arguments, otherwise you’d call it immediately by accident.)
        c. .pack(side="right", padx=10): place the button on the right side of the top frame, with 10px horizontal margin

        Click Flow : press STOP -> Tkinter calls _set_gear(1) -> udpdates self.gear + repaints button using _refresh_buttons()
        """

        # Middle container
        mid = tk.Frame(self, bg="#333333")
        mid.pack(pady=10, padx=10)

        # Row 1: Gear buttons (horizontal)
        self.btn_frame = tk.Frame(mid, bg="#333333") #sub frame within mid
        self.btn_frame.pack()  # centered row
        self._buttons = []
        # build 5 buttons in a loop
        for g in range(1, 6):
            normal, _ = self.GEAR_COLORS[g]
            # self.GEAR_COLORS[g]: your color map per gear
            b = tk.Button(
                self.btn_frame, 
                text=str(g), 
                width=4, bg=normal,
                fg="white" if g >= 4 else "black",
                command=lambda gg=g: self._set_gear(gg))
            b.pack(side="left", padx=5)
            self._buttons.append(b)

        # Row 2: Video display (below)
        # video_label : video panel
        # Label: a display widget that can show text or an image
        self.video_label = tk.Label(mid, bg="#000000")
        self.video_label.pack(pady=(10, 0))
        """
        How is the video being presented?
        First understand that, video_tick() runs every ~33ms and updates self.video_label with the latest camera frame
        1. ROS callback: image_callback receives frame from /camera/...
            converts it from ROS format -> OpenCV (NumPY) -> RGB and stores it in self.latest_frame
        2. Tkinter loop (video_tick) takes self.latest_frame -> resizes it -> converts to a Tk-compatible imaage and displays on self.video label

        Camera -> image_callback -> convert (ROS -> OpenCV -> RGB) -> self.latest_frame (NumPy) -> Tkinter video_tick -> Convert (NumPy -> PIL -> ImageTK) -> self.video_label (UI) 
        """

    def _refresh_buttons(self):
        for idx, b in enumerate(self._buttons, start=1):
            normal, pressed = self.GEAR_COLORS[idx]
            if self.gear == idx:
                b.config(bg=pressed, relief="sunken", bd=4)
            else:
                b.config(bg=normal, relief="raised", bd=2)
        # Visually marks which gear is active
        # How: Loops over buttons, if a button’s index matches self.gear, apply the “pressed” style

    def _set_gear(self, g):
        self.gear = max(1, min(5, g))
        self._refresh_buttons()
        rospy.loginfo("Gear -> %d", self.gear)
        """
        Why: Central place to change gears (from buttons or joystick triggers)
        How: Clamp value, repaint buttons, log to ROS console
        Connects:
        - Called by gear buttons and by joy_callback() (RT/LT).
        - publish_twist() uses self.gear to choose the speed.
        """

    # ---------- ROS callbacks ----------
    def joy_callback(self, msg):
        # msg is the snapshot of the controller
        # msg.axis -> list of stick triggers in [-1.0, 1.0]
        # msg.buttons -> list of ints
        axes = msg.axes
        # Xbox mappings (based on events): 
        # - LT=axes[2]
        # - RT=axes[5]
        rt = axes[5] < 0.5 # when RT is pressed, value becomes smaller than 0.5
        lt = axes[2] < 0.5 # same

        # Edge detection
        # Act only when the state changes from NOT pressed -> PRESSED
        if rt and not self.last_triggers[0] and self.gear < 5:
            self._set_gear(self.gear + 1)
        self.last_triggers[0] = rt

        if lt and not self.last_triggers[1] and self.gear > 1:
            self._set_gear(self.gear - 1)
        self.last_triggers[1] = lt

        self.latest_axes = axes
        """
        How: “Edge detection”: only change gear on the transition from not-pressed to pressed
        - Some drivers map triggers so released ≈ 1.0 and pressed (−1.0 or < 0.5)
        - We treat “pressed” = value < 0.5 as a simple threshold
        ---
        - Change gear only when RT goes from False → True (the edge)
        - We compare the current pressed state (rt) with the previous state (self.last_triggers[0]).
        - Then we store the current state for next time

        Connects:
        - Calls _set_gear() which repaints buttons.
        - publish_twist() reads self.latest_axes to send motion.
        """

    def image_callback_raw(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.flip_h: bgr = bgr[:, ::-1]
            if self.flip_v: bgr = bgr[::-1, :]
            self.latest_frame = bgr[:, :, ::-1]  # RGB
        except CvBridgeError as e:
            rospy.logwarn("cv_bridge error: %s", e)
        """
        What: Turns a ROS sensor_msgs/Image into an RGB NumPy array
        Why: Tkinter/Pillow can’t draw ROS messages, we need a normal image array
        How: cv_bridge → BGR array → optional flips → convert to RGB → store in latest_frame
        Connects: video_tick() reads latest_frame and draws it
        """

    def image_callback_compressed(self, msg):
        if np is None or cv2 is None:
            return
        try:
            arr = np.fromstring(msg.data, np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                return
            if self.flip_h: bgr = cv2.flip(bgr, 1)
            if self.flip_v: bgr = cv2.flip(bgr, 0)
            self.latest_frame = bgr[:, :, ::-1]  # RGB
        except Exception as e:
            rospy.logwarn("compressed decode error: %s", e)
        """
        What: Decodes sensor_msgs/CompressedImage (JPEG) into an RGB array.
        Why: Compressed is lighter over network; needs OpenCV to decode.
        How: np.fromstring → cv2.imdecode to BGR → optional flips → RGB → store in latest_frame.
        Connects: video_tick() draws latest_frame. (If OpenCV/NumPy missing, we gracefully skip drawing.)
        """


    # ---------- Layout helpers ----------
    def _sync_video_width(self):
        # measure the actual rendered width of the gear row
        self.update_idletasks()
        self.video_target_w = self.btn_frame.winfo_width()

    def _resize_to_target(self, pil_img):
        # keep aspect: fit width to gear row
        w0, h0 = pil_img.size
        tw = getattr(self, 'video_target_w', None)
        if not tw or tw <= 1:
            return pil_img  # fallback until width known
        th = int((float(h0) / float(w0)) * tw)
        return pil_img.resize((tw, th), PILImage.BILINEAR)


    # ---------- Loops ----------
    def video_tick(self):
        if self.latest_frame is not None:
            img = PILImage.fromarray(self.latest_frame) # NumPy (RGB) → PIL Image
            img = self._resize_to_target(img)  # fit to gear width
            imgtk = ImageTk.PhotoImage(image=img) # PIL → Tk image object
            self.video_label.imgtk = imgtk  # prevent GC
            self.video_label.configure(image=imgtk)  # show it on the Label
        self.after(33, self.video_tick)  # show it on the Label

    def publish_twist(self, _evt):
        if not hasattr(self, 'latest_axes'):
            return
        t = Twist()
        # Forward/back on left stick Y
        if self.latest_axes[1] > 0.5:
            t.linear.x = self.GEAR_SPEEDS[self.gear]
        elif self.latest_axes[1] < -0.5:
            t.linear.x = -self.GEAR_SPEEDS[self.gear]
        else:
            t.linear.x = 0.0
        # Turn on right stick X
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
