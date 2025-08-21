🤖 URDF Visualizer

A web-based simulator for controlling robots like Poppy Humanoid, SO101 Arm, and Allegro Hand using MediaPipe tracking and URDF visualization.

This tool lets you load URDF models in a browser, visualize them in 3D, and control them via human motion captured through a webcam.

📂 Features

Load and view URDF models directly in your browser.

Real-time robot control using MediaPipe Pose & Hands.

Support for multiple robots:

🦾 Poppy Humanoid (with backend using PyBullet)

🦿 SO101 Arm

✋ Allegro Hand

Easy switching between different robot configurations.

No heavy simulation software required — runs directly in a browser.

⚙️ PyBullet Backend (for Poppy Humanoid)

For the Poppy Humanoid, this project uses PyBullet
 as the physics engine and robot control backend.

PyBullet is a Python library for physics simulation, robotics, and reinforcement learning. In this project, it is responsible for:

Simulation Setup – Creates a 3D world with gravity, floor plane, and objects.

Robot Loading – Loads the Poppy Humanoid’s URDF model into the simulation.

Joint Control – Moves robot joints using p.setJointMotorControl2() based on MediaPipe inputs.

Physics & Collisions – Runs real-time physics with p.stepSimulation().

State Feedback – Sends joint positions and base state back to the frontend for visualization.

Rendering – Streams camera images from the simulation using p.getCameraImage().

👉 Note: PyBullet is only required when running the Poppy Humanoid. Other robots (SO101 Arm, Allegro Hand) run entirely in the browser.

🚀 Getting Started
1️⃣ Clone the Repository
git clone https://github.com/anjalikurhade05/Full-Body-Controller.git
cd Full-Body-Controller

2️⃣ Install Dependencies

Make sure you have Node.js v16+ installed, then run:

npm install

3️⃣ Start the Frontend
npm run dev


By default, the development server will be available at:
👉 http://localhost:5173

🤖 Running for Each Robot
1. Poppy Humanoid (with backend)

Frontend:
Open:

http://localhost:5173/src/humanoid_viewer/humanoid_test.html


Backend (requires Python 3.x):

cd src/poppy_backend
python main.py

2. SO101 Arm

Frontend only:

http://localhost:5173/src/SO-ARM100-main/Arm_test.html

3. Allegro Hand

Frontend only:

http://localhost:5173/src/hand_viewer/test.html

🛠 Requirements

Node.js v16 or later

Python 3.x (only for Poppy Humanoid backend)

Webcam for MediaPipe tracking

Modern browser (Chrome, Edge, or Firefox recommended)

⚠️ Known Issues

Leg movement is currently disabled (logic commented out). You can uncomment it to test.

Table and cube in PyBullet: Enable by uncommenting code at main.py (line ~88). Partial functionality.

Some lag in robot movements due to backend (PyBullet) connection.

✨ With this setup, you can experiment with robot control directly in your browser while leveraging PyBullet physics for realistic simulation when needed.
