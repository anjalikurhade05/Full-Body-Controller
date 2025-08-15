# 🤖 URDF Visualizer

A web-based simulator for controlling robots like **Poppy Humanoid**, **SO101 Arm**, and **Allegro Hand** using MediaPipe tracking and URDF visualization.

This tool lets you load URDF models in a browser, visualize them in 3D, and control them via human motion captured through a webcam.

---

## 📂 Features
- Load and view **URDF** models directly in your browser.
- Real-time robot control using **MediaPipe Pose & Hands**.
- Support for multiple robots:
  - 🦾 **Poppy Humanoid** (with backend)
  - 🦿 **SO101 Arm**
  - ✋ **Allegro Hand**
- Easy switching between different robot configurations.
- No installation of heavy simulation software required — runs in a browser.

---

## 🚀 Getting Started

### 1️⃣ Clone the Repository
```bash
git clone https://github.com/anjalikurhade05/Full-Body-Controller.git
cd Full-Body-Controller
###2️⃣ Install Dependencies
Make sure you have Node.js v16+ installed, then run:
npm install
###3️⃣ Start the Frontend
npm run dev
By default, the development server will be available at:
http://localhost:5173
###🤖 Running for Each Robot
1.Poppy Humanoid (with backend)
Frontend:
Open http://localhost:5173/src/humanoid_viewer/humanoid_test.html in your browser.

Backend:
Requires Python 3.x. In another terminal:
cd src/poppy_backend
python main.py

2.SO101 Arm
Open: http://localhost:5173/src/SO-ARM100-main/Arm_test.html

3.Allegro Hand
Open: http://localhost:5173/src/hand_viewer/test.html

🛠 Requirements
Node.js v16 or later
Python 3.x (only for Poppy Humanoid backend)
Webcam for MediaPipe tracking
Modern browser (Chrome/Edge/Firefox recommended)
