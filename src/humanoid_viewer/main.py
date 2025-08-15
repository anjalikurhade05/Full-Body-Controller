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

def run_flask_app():
    # Use 0.0.0.0 so it is accessible from other machines if needed.
    app.run(host='0.0.0.0', port=5000, debug=False)

# ---------------- PyBullet setup ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# --- Floor ---
floor_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[5, 5, 0.01])
p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_shape, basePosition=[0, 0, -0.01])

# --- Load Robot ---
robot_urdf_path = "C:/Users/dssau/OneDrive/Desktop/Internship/urdfss-b1/poppy/Poppy_Humanoid.URDF"
robotId = p.loadURDF(robot_urdf_path, [0, 0, 0.7], useFixedBase=True)

# --- Robot Joint Mapping ---
joint_name_to_id = {}
for i in range(p.getNumJoints(robotId)):
    joint_info = p.getJointInfo(robotId, i)
    joint_name = joint_info[1].decode('UTF-8')
    joint_name_to_id[joint_name] = joint_info[0]

# --- Spawn Ball in PyBullet ---
sphere_radius = 0.05
col_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)
visual_shape_id = p.createVisualShape(
    p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[1, 0, 0, 1]
)
ball_id = p.createMultiBody(
    baseMass=0.2,
    baseCollisionShapeIndex=col_shape_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=[0.5, 0, 0.8]
)

# --- Track connected WebSocket clients ---
connected_clients = set()
connected_clients_lock = threading.Lock()

async def send_initial_state(ws):
    """Send a one-time full state to a newly connected client."""
    joints_state = {}
    for jname, jid in joint_name_to_id.items():
        pos = p.getJointState(robotId, jid)[0]
        joints_state[jname] = pos
    ball_pos, ball_orn = p.getBasePositionAndOrientation(ball_id)
    packet = {
        "type": "sim_state",
        "joints": joints_state,
        "objects": [
            {"name": "ball", "position": ball_pos, "orientation": ball_orn}
        ]
    }
    try:
        await ws.send(json.dumps(packet))
    except Exception as e:
        print("Error sending initial state:", e)

async def handle_mediapipe_data(websocket):
    """Handle incoming messages from frontend (MediaPipe -> backend commands)."""
    print("WebSocket client connected!")
    with connected_clients_lock:
        connected_clients.add(websocket)
    # send an initial state snapshot so the client can render immediately
    try:
        await send_initial_state(websocket)
    except Exception as e:
        print("Failed to send initial state:", e)

    try:
        async for message in websocket:
            try:
                received_data = json.loads(message)
                if received_data.get('type') == 'joint_command':
                    joint_name = received_data.get('joint_name')
                    target_position = received_data.get('target_position')
                    if joint_name in joint_name_to_id:
                        p.setJointMotorControl2(
                            bodyIndex=robotId,
                            jointIndex=joint_name_to_id[joint_name],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=target_position,
                            force=500
                        )
            except Exception as e:
                print(f"Error processing message from client: {e}")
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected (normal).")
    except Exception as e:
        print("WebSocket handler error:", e)
    finally:
        with connected_clients_lock:
            if websocket in connected_clients:
                connected_clients.remove(websocket)
        # no explicit close here; the server will clean up

async def broadcast_state():
    """Send robot and ball state to all connected clients at ~60 Hz."""
    while True:
        # Robot joint states
        joints_state = {}
        for jname, jid in joint_name_to_id.items():
            try:
                pos = p.getJointState(robotId, jid)[0]
            except Exception:
                pos = 0.0
            joints_state[jname] = float(pos)

        # Ball state
        ball_pos, ball_orn = p.getBasePositionAndOrientation(ball_id)
        # Ensure types are JSON serializable lists of floats
        ball_pos_list = [float(ball_pos[0]), float(ball_pos[1]), float(ball_pos[2])]
        ball_orn_list = [float(ball_orn[0]), float(ball_orn[1]), float(ball_orn[2]), float(ball_orn[3])]

        packet = {
            "type": "sim_state",
            "joints": joints_state,
            "objects": [
                {"name": "ball", "position": ball_pos_list, "orientation": ball_orn_list}
            ]
        }

        to_remove = []
        data = json.dumps(packet)
        with connected_clients_lock:
            clients = list(connected_clients)
        if clients:
            # Send concurrently and catch failures to drop dead clients
            send_tasks = []
            for c in clients:
                send_tasks.append(_safe_send(c, data))
            # run them concurrently
            results = await asyncio.gather(*send_tasks, return_exceptions=True)
            # _safe_send handles removal on error already
        await asyncio.sleep(1/120)  # 60 updates per second

async def _safe_send(ws, data):
    """Send data to ws, removing it from connected_clients on failure."""
    try:
        await ws.send(data)
    except Exception as e:
        print("Error sending to a client, removing it:", e)
        with connected_clients_lock:
            if ws in connected_clients:
                connected_clients.remove(ws)
        try:
            await ws.close()
        except Exception:
            pass

async def websocket_server():
    print("WebSocket server starting on ws://localhost:8765")
    async with websockets.serve(handle_mediapipe_data, "localhost", 8765):
        await broadcast_state()  # keeps running until interrupted



def run_websocket_server():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(websocket_server())
    except Exception as e:
        print("WebSocket thread exiting with error:", e)
    finally:
        loop.close()

# Start servers in separate threads
ws_server_thread = threading.Thread(target=run_websocket_server, daemon=True)
ws_server_thread.start()

flask_thread = threading.Thread(target=run_flask_app, daemon=True)
flask_thread.start()

# --- Main Simulation Loop ---
print("Starting simulation... Servers are running.")
try:
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    print("\nShutting down simulation...")
finally:
    if p.isConnected():
        p.disconnect()
