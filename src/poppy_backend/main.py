import pybullet as p
import time
import pybullet_data
import asyncio
import websockets
import threading
import json
import numpy as np
import cv2
from flask import Flask, Response
from flask_cors import CORS

# --- Flask App for Video Streaming ---
app = Flask(__name__)
CORS(app)

def generate_frames():
    """Generator function to capture PyBullet camera frames and stream them."""
    while True:
        # Define camera view parameters
        cam_target_pos = [0, 0, 0.5]
        cam_dist = 4
        cam_yaw = 45
        cam_pitch = -30

        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=cam_target_pos,
            distance=cam_dist,
            yaw=cam_yaw,
            pitch=cam_pitch,
            roll=0,
            upAxisIndex=2
        )

        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=float(800)/600,
            nearVal=0.1,
            farVal=100.0
        )

        (w, h, rgb, _, _) = p.getCameraImage(
            width=800,
            height=600,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        rgb_array = np.array(rgb, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (h, w, 4))
        bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGBA2BGR)

        (flag, encoded_image) = cv2.imencode(".jpg", bgr_array)
        if not flag:
            continue

        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
              bytearray(encoded_image) + b'\r\n')

        time.sleep(1/30)  # Stream at ~30 FPS

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# --- PyBullet Simulation Setup ---
def setup_pybullet():
    # We must use p.GUI here for the camera image to be available
    physicsClient = p.connect(p.GUI) 
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf")

    # IMPORTANT: Ensure this path is correct for your system.
    robot_urdf_path = "C:/Users/HP/OneDrive/Desktop/4wd-robot-simulator/urdf_visualizer/public/assets/robot2/URDF/Poppy_Humanoid.URDF"

    try:
        # Load the robot with useFixedBase=True to prevent it from falling
        robot_id = p.loadURDF(robot_urdf_path, basePosition=[0, 0, 0.5], useFixedBase=True)
        print(f"Robot loaded with ID: {robot_id}")
    except p.error as e:
        print(f"Error loading URDF: {e}")
        p.disconnect()
        return None
    
   # Load kitchen environment (additional URDF)

    kitchen_path = "C:/Users/HP/OneDrive/Desktop/4wd-robot-simulator/urdf_visualizer/public/assets/env/kitchen.urdf"

    try:
        p.loadURDF(kitchen_path, basePosition=[0, 0.2, 0], useFixedBase=True)

    except p.error as e:
        print(f"Warning: Could not load kitchen URDF: {e}. Simulation will continue without it.")

    # Create a test cube
    cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
    cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1], rgbaColor=[0, 1, 0, 1])
    cube = p.createMultiBody(baseMass=2, baseCollisionShapeIndex=cube_col, baseVisualShapeIndex=cube_vis, basePosition=[-2, 0, 1])
    p.resetBaseVelocity(cube, linearVelocity=[5, 0, 0])

    # Create a small ball near the robot
    ball_radius = 0.1
    ball_col = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
    ball_vis = p.createVisualShape(p.GEOM_SPHERE, radius=ball_radius, rgbaColor=[1, 0, 0, 1])
    # Place the ball directly above the robot's base to ensure it hits.
    ball_position = [0, 0, 1.5] 
    ball = p.createMultiBody(baseMass=2, baseCollisionShapeIndex=ball_col, baseVisualShapeIndex=ball_vis, basePosition=ball_position)

    return robot_id, cube, ball
# Global variables to store the robot ID and joint information
robot_id = None
joint_info = {}

# --- WebSocket Server ---
is_client_connected = False
websocket_connection = None
joint_commands = {}
websocket_loop = None

async def simulation_handler(websocket, path=None):
    """
    Handles incoming WebSocket messages from the frontend.
    """
    global is_client_connected, websocket_connection, joint_commands
    print("Frontend connected!")
    is_client_connected = True
    websocket_connection = websocket

    try:
        async for message in websocket:
            data = json.loads(message)
            if data['type'] == 'joint_command':
                joint_name = data['joint_name']
                target_position = data['target_position']
                joint_commands[joint_name] = target_position
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")
    finally:
        print("Frontend disconnected.")
        is_client_connected = False
        websocket_connection = None

def simulation_loop():
    """
    The main PyBullet simulation loop running in its own thread.
    """
    global robot_id, joint_info, joint_commands, websocket_connection, is_client_connected, websocket_loop, cube, ball

    while not is_client_connected:
        print("Waiting for frontend connection...")
        time.sleep(1)

    print("Starting simulation loop...")
    while p.isConnected() and is_client_connected:
        for joint_name, target_position in joint_commands.items():
            if joint_name in joint_info:
                joint_index = joint_info[joint_name]['jointIndex']
                p.setJointMotorControl2(
                    bodyIndex=robot_id,
                    jointIndex=joint_index,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_position
                )

        p.stepSimulation()
        
        if websocket_connection and websocket_loop:
            joint_states = {}
            for joint_name, info in joint_info.items():
                joint_states[joint_name] = p.getJointState(robot_id, info['jointIndex'])[0]

            # Get the position and orientation of the cube and ball
            cube_pos, cube_orn = p.getBasePositionAndOrientation(cube)
            ball_pos, ball_orn = p.getBasePositionAndOrientation(ball)

            # Create a list of objects with their state
            objects_state = [
                {
                    "type": "cube",
                    "position": cube_pos,
                    "orientation": cube_orn,
                    "id": "test_cube_1"
                },
                {
                    "type": "ball",
                    "position": ball_pos,
                    "orientation": ball_orn,
                    "id": "test_ball_1"
                }
            ]

            message = {
                "type": "sim_state",
                "joints": joint_states,
                "objects": objects_state
            }

            try:
                asyncio.run_coroutine_threadsafe(websocket_connection.send(json.dumps(message)), websocket_loop)
            except Exception as e:
                print(f"Error sending state to frontend: {e}")

        time.sleep(1./240.) # PyBullet simulation step time

    if p.isConnected():
        p.disconnect()

async def start_websocket_server_async():
    """This function will be run inside the new thread's event loop."""
    global websocket_loop
    websocket_loop = asyncio.get_running_loop() 
    server = await websockets.serve(simulation_handler, "localhost", 8765)
    print("Backend is running. Waiting for frontend connection...")
    await server.wait_closed()

def run_websocket_server():
    """Wrapper to start and run the asyncio event loop in a new thread."""
    asyncio.run(start_websocket_server_async())

# --- NEW: Function to run the Flask app in a separate thread ---
def run_flask_app():
    """Wrapper to run the Flask app."""
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main():
    global robot_id, joint_info, cube, ball
    robot_id, cube, ball = setup_pybullet()
    if robot_id is None:
        return

    for i in range(p.getNumJoints(robot_id)):
        joint_data = p.getJointInfo(robot_id, i)
        joint_name = joint_data[1].decode('utf-8')
        joint_info[joint_name] = {
            'jointIndex': joint_data[0],
            'jointType': joint_data[2],
            'lowerLimit': joint_data[8],
            'upperLimit': joint_data[9]
        }

    # Start the WebSocket server in a separate thread
    websocket_thread = threading.Thread(target=run_websocket_server, daemon=True)
    websocket_thread.start()

    # --- NEW: Start the Flask app in a separate thread ---
    flask_thread = threading.Thread(target=run_flask_app, daemon=True)
    flask_thread.start()

    # Run the PyBullet simulation loop directly
    simulation_loop()

if __name__ == '__main__':
    main()


#C:/Users/HP/OneDrive/Desktop/4wd-robot-simulator/urdf_visualizer/public/assets/robot2/URDF/Poppy_Humanoid.URDF



# --- Load Robot ---

# IMPORTANT: Ensure this path is correct for your system.

#robot_urdf_path = "C:/Users/HP/OneDrive/Desktop/4wd-robot-

# Load kitchen environment (additional URDF)

#kitchen_path = "C:/Users/HP/OneDrive/Desktop/4wd-robot-simulator/urdf_visualizer/public/assets/env/kitchen.urdf"

#try:

#    p.loadURDF(kitchen_path, basePosition=[0, 0, 0], useFixedBase=True)

#except p.error as e:

#    print(f"Warning: Could not load kitchen URDF: {e}. Simulation will continue without it.")



# Create a test cube

#cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])

#cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1], rgbaColor=[0, 1, 0, 1])

#cube = p.createMultiBody(baseMass=2, baseCollisionShapeIndex=cube_col, baseVisualShapeIndex=cube_vis, basePosition=[-2, 0, 1])

#p.resetBaseVelocity(cube, linearVelocity=[5, 0, 0])



# Create a small ball near the robot

#ball_radius = 0.05

#ball_col = p.createCollisionShape(p.GEOM_SPHERE, #radius=ball_radius)

#ball_vis = p.createVisualShape(p.GEOM_SPHERE, #radius=ball_radius, rgbaColor=[1, 0, 0, 1])

#ball_position = [0.5, 0.5, ball_radius]

#ball = p.createMultiBody(baseMass=0.1, #baseCollisionShapeIndex=ball_col, #baseVisualShapeIndex=ball_vis, basePosition=ball_position)

"""
    # Load the robot without useFixedBase=True to allow it to be dynamic
    robot_id = p.loadURDF(robot_urdf_path, basePosition=[0, 0, 0.5])
    print(f"Robot loaded with ID: {robot_id}")

    # NEW/UPDATED: Create a fixed constraint to anchor the robot's base.
    # The base will not move from its position, but the robot's joints can still move
    # in reaction to forces like the falling ball.
    p.createConstraint(robot_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0.5])"""