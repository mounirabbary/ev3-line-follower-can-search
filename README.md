# 🤖 EV3 Line Follower + Can Search (Ultrasonic Arc Scan)

![Python](https://img.shields.io/badge/Python-ev3dev-blue)
![Robot](https://img.shields.io/badge/LEGO-EV3-black)
![Control](https://img.shields.io/badge/Control-PID-green)
![Sensors](https://img.shields.io/badge/Sensors-2x%20Color%20%2B%20Ultrasonic-orange)
![License](https://img.shields.io/badge/License-MIT-brightgreen)

[![Demo video](https://img.youtube.com/vi/Z8Ef_XxjmJM/hqdefault.jpg)](https://youtube.com/shorts/Z8Ef_XxjmJM?feature=share)

A LEGO Mindstorms **EV3** robot programmed in **Python (ev3dev)** that:
- follows a line using a **PID controller** with **two color sensors** (`COL-REFLECT`)
- detects the **end of the line** using a stable *white-white* condition
- runs an **interruptible ultrasonic arc scan** to search for a can/object
- returns to the line using **motor encoders**, then resumes line following

---

## 🎥 Demo
- **YouTube demo (Shorts)**: https://youtube.com/shorts/Z8Ef_XxjmJM?feature=share  
- (Optional) Add more links in `videos/links.md`

---

## ✨ Key Features
- ✅ **PID line following** (P/I/D) with **integral clamping** to avoid windup
- ✅ **Gap vs. end-of-line logic** using:
  - `soglia_bianco` (white threshold)
  - `CONTRAST_EPS` (sensor similarity check)
  - `LINE_LOST_LIMIT` (time confirmation)
- ✅ **Search behavior** at end-of-line:
  - short move away from the end zone
  - **arc scan** collecting `(angle, distance_cm)` samples
  - detection of a **distance window** within a target range
  - aim to the window center + short approach
  - encoder-based return + resume line following
- ✅ **Interruptible scan**: if the robot sees the line during scanning, it aborts and returns immediately to line-follow mode

---

## 🧱 Hardware Setup

### Motors
- Left motor: `outA`
- Right motor: `outD`

### Sensors
- Color sensor (left): `in3` → `COL-REFLECT`
- Color sensor (right): `in4` → `COL-REFLECT`
- Ultrasonic sensor: `in2` → `US-DIST-CM`

> If your wiring/ports differ, update them at the top of `src/main.py`.

---

ev3-line-follower-can-search/
├─ README.md
├─ src/
│ └─ main.py
├─ docs/
│ └─ report-ev3.pdf
└─ videos/
└─ links.md



---

## ⚙️ How to Run (ev3dev)

### 1) Copy the project to the EV3
From your computer:
```bash
  scp -r ev3-line-follower-can-search robot@ev3dev.local:~/
```
### 2) SSH into the EV3
```bash
  ssh robot@ev3dev.local
```
### 3) Run the script
```bash
  cd ev3-line-follower-can-search
  python3 src/main.py
```
If import ev3dev.ev3 fails, verify you are using an ev3dev image with the Python bindings installed.

🧠 System Overview
1) Line Following (PID)

Reflected light is read from both sensors:

left_val = clLeft.value()

right_val = clRight.value()

Error:

error = left_val - right_val

PID correction:

correction = P*error + I*integral + D*derivative

Motors run in run_direct() and are commanded via duty_cycle_sp.

Implementation notes:

Integral clamping (INTEGRAL_LIM) reduces windup

Correction cap (relative to base speed) improves stability at higher speeds

2) End-of-Line Detection (white-white stable)

The robot considers the line ended when:

both sensors read white (> soglia_bianco)

values are similar (abs(left_val - right_val) < CONTRAST_EPS)

stable for ~LINE_LOST_LIMIT * dt seconds

Before confirming end-of-line, the robot performs a pivot recovery based on the last known error direction.

3) Can/Object Search (Ultrasonic Arc Scan)

Triggered after end-of-line is confirmed:

Store start encoder positions

Move slightly away from the end zone

Scan an arc and log (angle, distance_cm) samples

Detect a window of angles where distance falls inside [CAN_MIN_CM, CAN_MAX_CM]

Aim to the window center and approach slightly

Return to the start pose using encoders and resume line following

✅ Interruptibility: during scanning, if any color sensor detects the line again, the scan stops and the robot immediately returns to line-follow mode.

Line Following (list)

baseline: base motor duty cycle (higher = faster, less stable)

P_GAIN: proportional gain (too high → oscillations)

I_GAIN: integral gain (helps drift; too high → slow wobble)

D_GAIN: derivative gain (damping; too high → noisy)

INTEGRAL_LIM: integral clamp (prevents windup)

soglia_bianco: white threshold (depends on light/track)

CONTRAST_EPS: similarity for white-white (lower = stricter)

LINE_LOST_LIMIT: samples to confirm end (higher = fewer false triggers)

Search Behavior (list)

CAN_MIN_CM, CAN_MAX_CM: distance range for target

ARC_WIDE, ARC_WIDE_STEP: scan arc + resolution

K_TURN: turning calibration (deg → ticks)

WHEEL_DIAM_MM: distance conversion

DRIVE_SIGN: forward/backward sign

SCALE (in return): encoder return correction factor

White threshold (soglia_bianco): print sensor values on your track and choose a threshold clearly separating black line vs white floor.

Turn calibration (K_TURN): adjust until turn_deg(90) produces ~90° on your surface.

Direction: if the robot drives backward, invert the sign of baseline (and possibly DRIVE_SIGN for drive_cm).

🩹 Troubleshooting

Oscillations on the line → reduce P_GAIN, increase D_GAIN slightly, reduce baseline

Slow drift → increase I_GAIN slightly (keep INTEGRAL_LIM)

False end-of-line triggers → increase LINE_LOST_LIMIT, adjust soglia_bianco / CONTRAST_EPS

Doesn’t detect the object → check printed ultrasonic values; adjust CAN_MIN_CM / CAN_MAX_CM

Inaccurate turns/return → battery and friction matter; recalibrate K_TURN and SCALE

📄 Documentation

Report (PDF): docs/report-ev3.pdf

👤 Author

Mounir Abbary
GitHub: https://github.com/mounirabbary
Linkedin: https://www.linkedin.com/in/mounirabbary/

🧾 License

This project is licensed under the MIT License.
