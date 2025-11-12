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

1. **Base (Raw Joystick)**  
   Direct mapping from stick → speed. Great baseline; **harder to be precise** at low speed.

2. **Geared — Discrete Levels (1–5)**  
   Each gear is a **fixed cap** (e.g., Gear 2 = 0.4 m/s). Bump gears **up/down** with triggers.  
   *Novice-friendly, reduces tiny thumb adjustments.*

3. **Geared — Continuous Window per Gear**  
   Each gear defines a **[min, max]** window; the stick sweeps **inside that band**.  
   *Still analog, but always within a safe ceiling.*

---

## 🎮 Default Controls (example mapping)

- **LB**: enable motion  
- **Left stick (Y)**: forward/back  **Right stick (X)**: rotate  
- **RT / LT**: gear up / gear down  
- **Back**: emergency stop
