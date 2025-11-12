<div align="center">

# HCI Jackal: Gearing-Based Teleoperation Interfaces

Robots execute our commands—but do our interfaces reflect the **power** they control?  
We explore **gearing** (speed ranges you can trust) to make Clearpath **Jackal UGV** driving **safer, calmer, and more precise**.

[![Status](https://img.shields.io/badge/status-active-success)](#)
[![ROS](https://img.shields.io/badge/ROS-Indigo%20|%20Kinetic%20|%20Noetic-blue)](#)
[![Python](https://img.shields.io/badge/Python-2.7%20|%203.x-lightgrey)](#)
[![License](https://img.shields.io/badge/License-MIT-informational)](#license)

</div>

---

## ✨ Overview

**Plain idea.** If you put a fast robot in a crowded space, full throttle is risky. **Gearing** lets operators pick **safe speed bands** (e.g., *slow/precise* vs *normal*), so the joystick’s **full travel controls only the range you actually want**.

**What this is.** A Human–Computer Interaction (HCI) project at the University of Manitoba that designs and evaluates **gearing controls** for the Jackal UGV. By remapping joystick input to **smaller, safer velocity bands**, we aim to improve **usability, workload, and safety** in cluttered environments.

> “Select the capability you need; map the joystick to that subset.”

---

## 🤔 Why Gearing?

- **Robots expose 0–100% speed.** Operators rarely need that full range at once.  
- **Rescaling helps.** Gearing **compresses** the joystick so its full throw covers, say, **0 → 0.6 m/s**, not **0 → 2.0 m/s**.  
- **Result.** Fewer overshoots, smoother micro-movements, calmer operators.

**Tiny tech note (linear mapping)**  
If a gear defines a window `[v_min, v_max]` and the stick input is `u ∈ [0,1]`, we send  
`v = (v_max − v_min) · u + v_min`, then publish `geometry_msgs/Twist` on **`/cmd_vel`**.

---

## 🧪 What We Built (3 Variants)

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
