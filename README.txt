# HCI Jackal: Gearing-Based Teleoperation Interfaces

Robots are built to take in our every command but does it truly understand the power they hold?
Imagine a robot, a machine that is practically unstopable once it sets a speed its motors.
Now, imagine putting this robot in a crowded area, speeding through it with max speed. Sounds dangerous doesn't it?
We propose to design and find an interface that lets operators control the speed in certain levels, creating a safer way to utilise the power of the robot.

**What this is:** A Human–Computer Interaction (HCI) project at the University of Manitoba exploring **“gearing” controls** for the Clearpath **Jackal UGV**. Gearing lets an operator work within just the part of the robot’s capability they actually need (e.g., **slow, precise motion in tight spaces**) by remapping the joystick range to a **smaller, safer speed band**. This aims to improve **usability, workload, and safety** in cluttered environments. :contentReference[oaicite:0]{index=0}

---

## Why gearing?
Robots expose wide capability ranges (0–100% speed), but operators often only need a fraction. Gearing **rescales the joystick** so its full throw covers only the desired range (e.g., “up to 0.6 m/s”), making fine control easier and reducing overshoot and stress. :contentReference[oaicite:1]{index=1}

> **Our goal:** design, implement, and test interfaces where operators **select a subset of capability** and the joystick is **dynamically remapped** to that band. We evaluate with mock service tasks and collect usability/workload/safety data. :contentReference[oaicite:2]{index=2}

---

## What we built (three teleop variants)

1) **Base (Raw Joystick)**  
Direct mapping from joystick to speed—no scaling. Good baseline, but harder to be precise at low speeds. :contentReference[oaicite:3]{index=3}

2) **Geared — Discrete levels (e.g., 1–5)**  
Each gear has a **fixed speed**; triggers bump gear up/down. Helps novices avoid tiny, fatiguing stick adjustments.  
Example: gear 2 = **0.4 m/s**.  
Controls: **RT=+1 gear**, **LT=−1 gear**, stick up/down = forward/back at the gear’s fixed speed. :contentReference[oaicite:4]{index=4}

3) **Geared — Continuous per-level interval**  
Each gear defines a **[min, max] speed window**. We map joystick input `u ∈ [0,1]` to speed:  
`speed = (max − min) * u + min`.  
This preserves precision within a safe cap and still feels analog. Controls mirror the discrete mode. :contentReference[oaicite:5]{index=5}

---

## How we evaluate
- **Tasks:** mock service/navigation tasks in the HCI lab.  
- **Measures:** perceived usability & workload + safety indicators (e.g., near-collisions).  
- **Procedure:** compare base vs. geared modes to see if gearing reduces workload, improves control, and enhances safety. :contentReference[oaicite:8]{index=8}

---

## What’s included in this repo
- `scripts/` — teleop nodes (Base, Geared-Discrete, Geared-Continuous)  
- `launch/` — quick launch files for joystick + teleop  
- `docs/Jackal-Quickstart-Guide.pdf` — **short guidebook** you authored for new lab members (setup & troubleshooting)  
- `data/`, `notebooks/` — optional logs/analysis for user studies

> Note: Jackal exposes a ROS API with core topics (e.g., `/cmd_vel`) and runs an onboard PC. We integrate at the ROS node level and do not modify Jackal firmware. :contentReference[oaicite:9]{index=9}

---

**Default controls (example mapping):**  
- **LB:** enable motion  
- **Left stick (Y):** forward/back  
- **Right stick (X):** rotate  
- **RT/LT:** gear up/down (discrete or continuous window)  
- **Back:** emergency stop
