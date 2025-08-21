🤖 Poppy Humanoid Controller

A web-based simulator for visualizing and controlling the Poppy Humanoid robot using MediaPipe full-body tracking and URDF visualization.

This tool allows you to load the Poppy Humanoid URDF model in your browser, control its movements with your webcam in real time, and optionally run a backend for advanced control.

📂 Features

Load and view the Poppy Humanoid URDF model directly in the browser.
(Poppy Humanoid is loaded by default)

Real-time humanoid control using MediaPipe Pose tracking.

Full-body movement mapping (arms, legs, torso, head).

Lightweight — runs in a browser without heavy simulation tools.

🚀 Getting Started
1️⃣ Clone the Repository
using git clone command

cd urdf_visualizer


2️⃣ Install Dependencies

Make sure you have Node.js v16+ installed. Then run:

npm install

3️⃣ Start the Frontend
npm run dev


By default, the app will be available at:
👉 http://localhost:5173

🤖 Running Poppy Humanoid
Frontend

Open the following in your browser:

http://localhost:5173/src/humanoid_viewer/humanoid_test.html


🛠 Requirements

Node.js v16 or later

Python 3.x

Webcam for MediaPipe tracking

Modern browser (Chrome, Edge, or Firefox recommended)

Guide 

Every button has info button beside it on clicking it, it will display a pop up with information about the funtion of that respective button

Make sure to use quick guide button below webcam to get a quick guide on how to control the poppy humanoid with webcam (mediapipe)
