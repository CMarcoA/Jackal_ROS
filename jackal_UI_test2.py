#!/usr/bin/env python
# -*- coding: utf-8 -*-

import tkinter as tk  # Python 2.7 uses capital 'T'

def rgb(r, g, b):
    """Convert RGB to hex (e.g., #ff0000)."""
    return '#%02x%02x%02x' % (r, g, b)

class ContinuousSpeedUI(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.title("Jackal Interface (Geared Continuous - IPO2)")
        self.configure(bg="#333333")
        self._build_gui()

    def _build_gui(self):
        container = tk.Frame(self, bg="#333333")
        container.pack(padx=20, pady=20)

        canvas_width = 100
        canvas_height = 400

        # Speed gradient canvas
        self.speed_canvas = tk.Canvas(container, width=canvas_width, height=canvas_height,
                                      bg="lightgray", highlightthickness=1, highlightbackground="#999")
        self.speed_canvas.pack(side="left", padx=(0, 20))

        self._draw_gradient(canvas_width, canvas_height)
        self._draw_interval_lines(canvas_width, canvas_height, levels=5)

        # Webcam placeholder (also a Canvas for better height control)
        self.webcam_canvas = tk.Canvas(container, width=300, height=canvas_height,
                                       bg="#cccccc", highlightthickness=2, relief="sunken")
        self.webcam_canvas.pack(side="left")

    def _draw_gradient(self, width, height):
        steps = height
        for i in range(steps):
            # Gradient: green at top, red at bottom
            r = int((i / float(steps)) * 255)
            g = int((1 - i / float(steps)) * 255)
            b = 0
            color = rgb(r, g, b)
            self.speed_canvas.create_rectangle(0, i, width, i+1, outline=color, fill=color)

    def _draw_interval_lines(self, width, height, levels=5):
        step = height // levels
        for i in range(1, levels):
            y = i * step
            self.speed_canvas.create_line(0, y, width, y, fill="black", width=2)

if __name__ == '__main__':
    app = ContinuousSpeedUI()
    app.mainloop()
