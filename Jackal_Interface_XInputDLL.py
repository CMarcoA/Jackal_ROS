import ctypes
from ctypes import wintypes
import time                                   # for cooldown timing
import tkinter as tk
from tkinter import ttk   # like javax.swing.* in Java
import cv2                # for webcam capture
from PIL import Image, ImageTk  # to convert OpenCV images for Tkinter

# Try to load one of the standard XInput DLLs on Windows
XINPUT_DLLS = ('XInput1_4.dll', 'XInput9_1_0.dll', 'XInput1_3.dll')
for dll in XINPUT_DLLS:
    try:
        xinput = ctypes.windll.LoadLibrary(dll)
        break
    except OSError:
        xinput = None
if not xinput:
    raise RuntimeError("Could not load any XInput DLL – make sure you're on Windows with XInput support")

# XInput button flag for left thumbstick click (Emergency Stop)
XINPUT_GAMEPAD_LEFT_THUMB = 0x0040

# Mirror the C struct definitions for XINPUT_STATE and XINPUT_GAMEPAD -------------------------
class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ('wButtons', wintypes.WORD),  # bitmask of digital buttons
        ('bLeftTrigger', wintypes.BYTE),  # 0–255 analog value for left trigger
        ('bRightTrigger', wintypes.BYTE),  # 0–255 analog value for right trigger
        ('sThumbLX',ctypes.c_short), # analog thumbstick axes
        ('sThumbLY',ctypes.c_short),
        ('sThumbRX',ctypes.c_short),
        ('sThumbRY',ctypes.c_short),
    ]

class XINPUT_STATE(ctypes.Structure):
    _fields_ = [
        ('dwPacketNumber', wintypes.DWORD),
        ('Gamepad',        XINPUT_GAMEPAD),
    ]

# Tell ctypes about the function signature of XInputGetState
xinput.XInputGetState.argtypes = (wintypes.DWORD, ctypes.POINTER(XINPUT_STATE))
xinput.XInputGetState.restype  = wintypes.DWORD  # returns 0 on success

# --------------------------------------------------------------------------------------------------
class RobotControlApp:
    # color mappings for gears 1–5 (normal, pressed)
    GEAR_COLORS = {
        1: ("#00FF00", "#009900"),
        2: ("#80FF00", "#669900"),
        3: ("#FFFF00", "#CCCC00"),
        4: ("#FF8000", "#CC6600"),
        5: ("#FF0000", "#990000"),
    }

    def __init__(self, master):
        # --- core state ---
        self._gear = 1                             # current gear selection
        self._cap = cv2.VideoCapture(0)            # webcam capture
        self._buttons = {}                         # placeholder for refs

        # --- cooldown setup ---
        self._shift_cooldown = 1.0                 # seconds between gear shifts
        self._last_shift_time = 0.0                # timestamp of last shift

        # --- window setup ---
        master.title("Jackal Interface (Python ver.1)")
        master.configure(bg="#333333")

        # Build UI (including emergency stop & gear buttons) and start loops
        self._build_widgets(master)
        master.after(50, self._poll_controller)    # controller polling
        self._update_video()                       # video loop

    def _build_widgets(self, master):
        # --- Emergency Stop Button at top-right ---
        # Clicking this (or pressing L‐stick) invokes emergency stop
        self._stop_button = tk.Button(
            master,
            text="EMERGENCY STOP",
            bg="red",
            fg="white",
            width=14,
            relief="raised",
            bd=3,
            command=self._emergency_stop
        )
        # pack it in the top-right corner
        self._stop_button.pack(side="top", anchor="ne", padx=10, pady=10)

        # --- Container for gear buttons ---
        btn_frame = ttk.Frame(master)
        btn_frame.pack(pady=10)

        # Create one button per gear (1–5)
        for gear in range(1, 6):
            normal, pressed = RobotControlApp.GEAR_COLORS[gear]
            btn = tk.Button(
                btn_frame,
                text=str(gear),
                bg=normal,
                fg="white" if gear >= 4 else "black",
                width=4,
                relief="raised",
                bd=2,
                activebackground=pressed,
                command=lambda g=gear: self._set_gear(g)
            )
            btn.pack(side="left", padx=5)
            # store references for styling
            setattr(self, f"btn{gear}", btn)
            setattr(self, f"_g{gear}_normal", normal)
            setattr(self, f"_g{gear}_pressed", pressed)

        # Apply initial styling for the active gear
        self._update_button_styles()

        # --- Video Display Label ---
        self._video_label = ttk.Label(master)
        self._video_label.pack(padx=10, pady=10)

    def _update_button_styles(self):
        """Recolor & adjust relief on each button to show which gear is active."""
        for gear in range(1, 6):
            btn = getattr(self, f"btn{gear}")
            normal = getattr(self, f"_g{gear}_normal")
            pressed = getattr(self, f"_g{gear}_pressed")
            if self._gear == gear:
                btn.config(bg=pressed, relief="sunken", bd=4)
            else:
                btn.config(bg=normal, relief="raised", bd=2)

    def _set_gear(self, new_gear):
        """Immediately set gear, reset cooldown, and update UI."""
        self._gear = new_gear
        self._last_shift_time = time.monotonic()  # reset cooldown timer
        print(f"[Terminal] Gear set to {self._gear}")
        self._update_button_styles()

    def _emergency_stop(self):
        """
        Called when the Emergency Stop button is clicked
        or when the L‐stick is pressed on the controller.
        """
        # Example behavior: reset to gear 1 and update UI
        self._gear = 1
        print("[EMERGENCY STOP] Gear reset to 1")
        self._update_button_styles()
        # TODO: insert actual robot stop command here

    def _poll_controller(self):
        """
        Polls XInput every 50 ms. Checks thumbstick click for emergency stop
        and applies gear-shift cooldown before responding to RT/LT.
        """
        now = time.monotonic()
        ready = (now - self._last_shift_time) >= self._shift_cooldown

        state = XINPUT_STATE()
        if xinput.XInputGetState(0, ctypes.byref(state)) == 0:
            buttons = state.Gamepad.wButtons
            # --- Emergency stop on left thumbstick click (no cooldown) ---
            if buttons & XINPUT_GAMEPAD_LEFT_THUMB:
                self._emergency_stop()

            # --- Gear shifting with cooldown ---
            if ready:
                lt = state.Gamepad.bLeftTrigger    # 0–255
                rt = state.Gamepad.bRightTrigger   # 0–255
                threshold = 30                     # simple deadzone

                # RT → up, LT → down
                if rt > threshold and self._gear < max(RobotControlApp.GEAR_COLORS):
                    self._set_gear(self._gear + 1)
                elif lt > threshold and self._gear > 1:
                    self._set_gear(self._gear - 1)

        # schedule next poll without blocking GUI
        self._video_label.after(50, self._poll_controller)

    def _update_video(self):
        """Continuously grab frames from webcam and display them."""
        ret, frame = self._cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # BGR → RGB
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self._video_label.imgtk = imgtk                # keep reference
            self._video_label.configure(image=imgtk)
        # schedule next frame display
        self._video_label.after(30, self._update_video)


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlApp(root)
    root.mainloop()
