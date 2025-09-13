# FTC Robotics - DECODE Season

Welcome to the official codebase for our FTC Robotics Team **TimberX** during the **DECODE season**.  
This repository contains the teleop, autonomous, and utility code used to control and manage our competition robot.

---

## 📂 Project Structure

- **`StudioTeleop.java`** – Main teleoperated (driver-controlled) mode, including:
  - Field-centric driving
  - Odometry-based turning functions (`turnToAngle`, `turnToHeadingPID`)
  - Mode toggles for scoring vs pickup
  - Telemetry outputs for debugging and live feedback
- **`StudioOdometry.java`** – Custom odometry class used for position and heading tracking.
- Additional classes (to be added later) for autonomous routines, hardware mapping, and subsystems.

---

## 🚀 Features

- **Mecanum drive control** with support for strafing, turning, and speed normalization.
- **Odometry tracking** for real-time X, Y, and heading updates.
- **Telemetry feedback** to aid drivers and programmers in debugging robot state.

---

## 🔧 Setup

1. Clone this repository into your **FTC SDK** `TeamCode` folder.
2. Open the project in **Android Studio**.
3. Connect your FTC Robot Controller phone or Control Hub.
4. Deploy the code using the **FTC Driver Station** app.

---

## 🎮 Controls (TeleOp)

- **Left Stick (Y/X):** Drive forward/backward & strafe
- **Right Stick (X):** Turn
- **Left Stick Button:** Toggle between Pickup Mode ↔ Scoring Mode
- (Additional claw, slides, and automation features can be uncommented or extended later.)

---

## 🧪 Autonomous (Planned)

- Odometry-based path following
- AprilTag/vision integration
- Pre-programmed scoring and parking routines

---

## 👥 Team Notes

- Keep code **modular**: hardware logic in hardware classes, not TeleOp.
- Always test new code on the practice bot before pushing to `main`.
- Use **telemetry** for debugging robot state and controls.

---

## 📜 License

This codebase is developed for FTC competitions by our robotics team.  
You are free to reference and adapt the code for educational purposes.

---
