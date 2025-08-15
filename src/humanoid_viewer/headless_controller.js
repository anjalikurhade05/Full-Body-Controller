// headless_controller.js

import { MediaPipeHandController } from './mediapipe_humanoid_controller.js';

class PybulletController {
    constructor() {
        this.websocket = null;
        this.websocketConnected = false;

        // Initialize MediaPipe and connect WebSocket
        this.init();
    }

    init() {
        // Connect to the Python WebSocket server
        this.connectWebSocket();

        // Initialize MediaPipe, passing this controller instance
        const webcamVideoElement = document.getElementById('webcam');
        if (webcamVideoElement) {
            // The MediaPipe controller will now call this class's `updateJoint` method
            new MediaPipeHandController(this, webcamVideoElement);
        } else {
            console.error("Webcam video element not found in the DOM.");
        }
    }

    connectWebSocket() {
        const wsUrl = 'ws://localhost:8765';
        this.updateStatus('Attempting to connect to PyBullet...', 'info');
        this.websocket = new WebSocket(wsUrl);

        this.websocket.onopen = () => {
            this.websocketConnected = true;
            this.updateStatus('Connected to PyBullet simulation.', 'success');
        };

        this.websocket.onclose = () => {
            this.websocketConnected = false;
            this.updateStatus('Disconnected. Trying to reconnect...', 'error');
            setTimeout(() => this.connectWebSocket(), 3000);
        };

        this.websocket.onerror = () => {
            this.websocketConnected = false;
            this.updateStatus('WebSocket error. Is the Python script running?', 'error');
        };
    }

    /**
     * This function is called by the MediaPipe controller to send a command.
     * It no longer updates a 3D scene, it just sends the data.
     */
    updateJoint(jointName, value) {
        if (this.websocketConnected) {
            const message = {
                type: 'joint_command',
                joint_name: jointName,
                target_position: value
            };
            this.websocket.send(JSON.stringify(message));
        }
    }

    updateStatus(message, type) {
        const statusDiv = document.getElementById('status');
        if (statusDiv) {
            statusDiv.textContent = message;
            statusDiv.className = `status-${type}`;
        }
    }
}

// Start the controller when the page loads
window.addEventListener('DOMContentLoaded', () => {
    new PybulletController();
});