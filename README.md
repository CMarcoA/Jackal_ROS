<h1 align="center">HCI Jackal: Gearing-Based Teleoperation Interfaces</h1>

<p align="center">
  Robots execute our commands — but do our interfaces respect the <b>power</b> they control?<br>
  We explore <b>gearing</b> (trustable speed ranges) to make Clearpath <b>Jackal UGV</b> driving
  <b>safer</b>, <b>calmer</b>, and more <b>precise</b>.
</p>

<p align="center">
  <a href="#"><img src="https://img.shields.io/badge/Status-Active-success?style=flat-square" alt="Status"></a>
  <a href="#"><img src="https://img.shields.io/badge/ROS-Indigo%20|%20Kinetic%20|%20Noetic-blue?style=flat-square" alt="ROS"></a>
  <a href="#"><img src="https://img.shields.io/badge/Python-2.7%20|%203.x-lightgrey?style=flat-square" alt="Python"></a>
  <a href="#license"><img src="https://img.shields.io/badge/License-MIT-informational?style=flat-square" alt="License"></a>
</p>

<hr />

## 🔎 Overview

**Plain idea:** fast robots in tight spaces are risky. **Gearing** lets the operator pick a safe speed band (e.g., *Slow* vs *Normal*), so the joystick’s **full travel controls only that band**.

**Technical one-liner:** a ROS teleop node that remaps joystick input into **discrete gears** or **per-gear [min,max] windows** and publishes `geometry_msgs/Twist` to **`/cmd_vel`**.

**Tiny example:** if Gear 1 caps linear speed at ~0.20 m/s and Gear 3 at ~0.60 m/s, the same thumb motion yields **finer control** in Gear 1 for doorway approaches; switch to Gear 3 for open halls.

---

## 🤔 Why Gearing?

- Robots expose a broad capability range (0–100% speed); operators rarely need all of it at once.  
- Gearing **rescales** the joystick so its full throw maps to a **smaller, safer** velocity band (e.g., 0→0.6 m/s).  
- Benefits: fewer overshoots, smoother micro-movements, **lower operator workload**, safer demos.

## 🧪 Interfaces in this Repo

1. **Base (Raw Joystick)** — direct stick→speed mapping (baseline; precise low-speed control is harder).  
2. **Geared — Discrete Levels (1–5)** — each gear is a fixed cap (e.g., Gear 2 = 0.40 m/s); bump gear up/down.  
3. **Geared — Continuous Window per Gear** — each gear defines a `[min,max]` window; stick sweeps inside that band (analog feel, safe ceiling).

---

## 🎮 Default Controls (example mapping)

- **LB**: enable motion  
- **Left stick (Y)**: forward/back  **Right stick (X)**: rotate  
- **RT / LT**: gear up / gear down  
- **Back**: emergency stop

---

## ⚡ Quick Start (ROS)

```bash
# 1) Confirm your gamepad device
ls /dev/input/          # look for js0, js1, js2 ...

# 2) Start the joystick driver
rosrun joy joy_node _dev:=/dev/input/js2

# 3) Run a teleop node (publishes /cmd_vel)
rosrun jackal_teleop xbox_teleop.py