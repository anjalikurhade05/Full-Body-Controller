// mediapipe_hand_controller.js
import {
  HandLandmarker,
  PoseLandmarker,
  FilesetResolver
} from "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.14/vision_bundle.mjs";

// --- NEW: Define only the connections and points we want to draw ---
// This will only draw the left arm: shoulder -> elbow -> wrist
const CONTROL_POSE_CONNECTIONS = [[11, 13], [13, 15]]; 
// This will only draw the points for the left shoulder, elbow, and wrist
const CONTROL_POSE_INDICES = [11, 13, 15];

export class MediaPipeHandController {
    constructor(viewerInstance, videoElement) {
        this.viewer = viewerInstance;
        this.video = videoElement;
        this.handLandmarker = null;
        this.poseLandmarker = null;
        this.runningMode = "VIDEO";
        this.lastVideoTime = -1;
        this.resetTimer = null;
        this.resetDelay = 2000;
        this.previousAngles = {};
        

        // Helper functions
        this.clamp = (v, min, max) => Math.max(min, Math.min(max, v));
        this.mapRange = (value, inMin, inMax, outMin, outMax) => {
            const val = (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
            return this.clamp(val, Math.min(outMin, outMax), Math.max(outMin, outMax));
        };
        // This is the core smoothing function. Adjust alpha for responsiveness.
        // Lower alpha = smoother but more "drag". Higher alpha = more responsive but more "jitter".
        this.smooth = (name, value, alpha = 0.4) => {
            const prev = this.previousAngles[name] || value;
            const smoothed = prev * (1 - alpha) + value * alpha;
            this.previousAngles[name] = smoothed;
            return smoothed;
        };
        this.getAngle = (p1, p2, p3) => {
            const v1 = new THREE.Vector3().subVectors(p1, p2);
            const v2 = new THREE.Vector3().subVectors(p3, p2);
            return v1.angleTo(v2);
        };

        this.initMediaPipe();
    }

    async initMediaPipe() {
        try {
            const vision = await FilesetResolver.forVisionTasks("https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@latest/wasm");
            this.handLandmarker = await HandLandmarker.createFromOptions(vision, {
                baseOptions: { modelAssetPath: `https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task`, delegate: "GPU" },
                numHands: 1, runningMode: this.runningMode
            });
            this.poseLandmarker = await PoseLandmarker.createFromOptions(vision, {
                baseOptions: { modelAssetPath: `https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task`, delegate: "GPU" },
                runningMode: this.runningMode
            });
            this.setupWebcam();
        } catch (error) {
            console.error("Failed to initialize MediaPipe:", error);
            this.viewer.updateStatus("Error: Failed to load AI models.", 'error');
        }
    }

    setupWebcam() {
        navigator.mediaDevices.getUserMedia({ video: { width: 640, height: 480 } })
            .then(stream => {
                this.video.srcObject = stream;
                this.video.addEventListener("loadeddata", () => {
                    this.overlayCanvas = document.getElementById("overlay");
                    this.overlayCanvas.width = this.video.videoWidth;
                    this.overlayCanvas.height = this.video.videoHeight;
                    this.overlayCtx = this.overlayCanvas.getContext("2d");
                    this.predictWebcam();
                });
            })
            .catch(err => this.viewer.updateStatus("Error: Webcam access denied.", 'error'));
    }

    async predictWebcam() {
        if (this.video.readyState < 2) {
            requestAnimationFrame(() => this.predictWebcam());
            return;
        }

        let startTimeMs = performance.now();
        if (this.lastVideoTime !== this.video.currentTime) {
            this.lastVideoTime = this.video.currentTime;
            const handResult = this.handLandmarker.detectForVideo(this.video, startTimeMs);
            const poseResult = this.poseLandmarker.detectForVideo(this.video, startTimeMs);


            
        // Log entire results for debugging
      //  console.log("Pose Result:", poseResult);
      //  console.log("Hand Result:", handResult);
            const handLandmarks = handResult.landmarks[0];
            const poseLandmarks = poseResult.landmarks[0];
            
            
            this.drawLandmarks(poseLandmarks, handLandmarks);

            if (handLandmarks && poseLandmarks) {
                clearTimeout(this.resetTimer);
                this.resetTimer = null;
                this.mapLandmarksToRobot(poseLandmarks, handLandmarks);
            } else {
                this.startResetTimer();
            }
        }
        requestAnimationFrame(() => this.predictWebcam());
    }

    startResetTimer() {
        if (!this.resetTimer) {
            this.resetTimer = setTimeout(() => this.resetRobotPose(), this.resetDelay);
        }
    }

    resetRobotPose() {
        if (this.viewer?.joints) {
            Object.keys(this.viewer.joints).forEach(jointName => this.viewer.updateJoint(jointName, 0));
        }
        this.previousAngles = {};
    }
    
mapLandmarksToRobot(pose, hand) {
    if (!this.viewer?.robot) return;

    const minVisibility = 0.6;

    // Pose visibility values
    const visLeft = {
        shoulder: pose[11]?.visibility ?? 0,
        elbow: pose[13]?.visibility ?? 0,
        wrist: pose[15]?.visibility ?? 0,
    };
    const visRight = {
        shoulder: pose[12]?.visibility ?? 0,
        elbow: pose[14]?.visibility ?? 0,
        wrist: pose[16]?.visibility ?? 0,
    };

    const leftVisible = visLeft.shoulder > minVisibility && visLeft.elbow > minVisibility && visLeft.wrist > minVisibility;
    const rightVisible = visRight.shoulder > minVisibility && visRight.elbow > minVisibility && visRight.wrist > minVisibility;

    if (rightVisible || leftVisible) {
        // Choose which arm to use — prioritize right
        const useRight = rightVisible;
        const shoulder = new THREE.Vector3(...(useRight ? [pose[12].x, pose[12].y, pose[12].z] : [pose[11].x, pose[11].y, pose[11].z]));
        const elbow = new THREE.Vector3(...(useRight ? [pose[14].x, pose[14].y, pose[14].z] : [pose[13].x, pose[13].y, pose[13].z]));
        const wrist = new THREE.Vector3(...(useRight ? [pose[16].x, pose[16].y, pose[16].z] : [pose[15].x, pose[15].y, pose[15].z]));

        // Hand landmarks
        const thumbTip = new THREE.Vector3(hand[4].x, hand[4].y, hand[4].z);
        const indexTip = new THREE.Vector3(hand[8].x, hand[8].y, hand[8].z);
        const indexBase = new THREE.Vector3(hand[5].x, hand[5].y, hand[5].z);
        const middleMCP = new THREE.Vector3(hand[9].x, hand[9].y, hand[9].z);
        const pinkyBase = new THREE.Vector3(hand[17].x, hand[17].y, hand[17].z);

        // Shoulder pan
        const shoulderPanAngle = this.mapRange(wrist.x, shoulder.x - 0.3, shoulder.x + 0.3, -1.9, 1.9);
        this.viewer.updateJoint("shoulder_pan", this.smooth("shoulder_pan", shoulderPanAngle));

        // Shoulder lift
        const upperArmVec = new THREE.Vector3().subVectors(elbow, shoulder);
        const verticalVec = new THREE.Vector3(0, 1, 0);
        const shoulderLiftAngleRad = upperArmVec.angleTo(verticalVec);
        const shoulderLiftAngle = this.mapRange(shoulderLiftAngleRad, 2.5, 0.5, -1.7, 1.7);
        this.viewer.updateJoint("shoulder_lift", this.smooth("shoulder_lift", shoulderLiftAngle));
/*
       // Elbow
        const elbowAngleRad = this.getAngle(shoulder, elbow, wrist);
       //  console.log("elbowAngleRad (radians):", elbowAngleRad);
        const elbowFlexAngle = this.mapRange(elbowAngleRad, 3.0, 2.2, -1.6, 1.6);
        this.viewer.updateJoint("elbow_flex", this.smooth("elbow_flex", elbowFlexAngle));
       
 // Wrist
const wristFlexAngleRad = this.getAngle(elbow, wrist, middleMCP);
console.log("wristFlexAngleRad (radians):", wristFlexAngleRad);

let wristFlexAngle = 0;


    // Mapping: 0.55 → 0 and 0.60 → 1.66
    wristFlexAngle = this.mapRange(wristFlexAngleRad, 0.55, 0.5, 0, 1.66);
    wristFlexAngle = Math.min(wristFlexAngle, 1.66); // Clamp upper bound


// Apply smoothing and update joint
this.viewer.updateJoint("wrist_flex", this.smooth("wrist_flex", wristFlexAngle));

*/
        // Wrist roll
        const handWidthVec = new THREE.Vector3().subVectors(pinkyBase, indexBase);
        const wristRollAngle = this.mapRange(Math.atan2(handWidthVec.y, handWidthVec.x), -1.5, 1.5, -2.7, 2.8);
        this.viewer.updateJoint("wrist_roll", this.smooth("wrist_roll", wristRollAngle));

        // Gripper
        const gripperDist = thumbTip.distanceTo(indexTip);
        const gripperValue = this.mapRange(gripperDist, 0.02, 0.15, -0.17 ,1.7);
        this.viewer.updateJoint("gripper", this.smooth("gripper", gripperValue));
    } else {
        console.warn("No visible arm — skipping robot update");
    }
}


    // --- UPDATED DRAWING LOGIC ---
    drawLandmarks(poseLandmarks, handLandmarks) {
        if (!this.overlayCtx) return;
        const ctx = this.overlayCtx;
        ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
        ctx.save();
        ctx.translate(ctx.canvas.width, 0);
        ctx.scale(-1, 1);

        // Draw only the left arm pose
        if (poseLandmarks) {
            this.drawConnectors(ctx, poseLandmarks, CONTROL_POSE_CONNECTIONS, { color: '#33aaff', lineWidth: 4 });
            this.drawSpecificPoints(ctx, poseLandmarks, CONTROL_POSE_INDICES, { color: '#FF4136', radius: 5 });
        }
        // Draw the hand
        if (handLandmarks) {
            this.drawConnectors(ctx, handLandmarks, HAND_CONNECTIONS, { color: '#00ff99', lineWidth: 4 });
            this.drawPoints(ctx, handLandmarks, { color: '#FF851B', radius: 5 });
        }
        ctx.restore();
    }
    
    drawConnectors(ctx, landmarks, connections, options) {
        ctx.strokeStyle = options.color;
        ctx.lineWidth = options.lineWidth;
        for (const conn of connections) {
            const start = landmarks[conn[0]];
            const end = landmarks[conn[1]];
            if (start && end && (start.visibility??1) > 0.5 && (end.visibility??1) > 0.5) {
                ctx.beginPath();
                ctx.moveTo(start.x * ctx.canvas.width, start.y * ctx.canvas.height);
                ctx.lineTo(end.x * ctx.canvas.width, end.y * ctx.canvas.height);
                ctx.stroke();
            }
        }
    }

    drawPoints(ctx, landmarks, options) {
        ctx.fillStyle = options.color;
        for (const lm of landmarks) {
            if (lm && (lm.visibility??1) > 0.5) {
                ctx.beginPath();
                ctx.arc(lm.x * ctx.canvas.width, lm.y * ctx.canvas.height, options.radius, 0, 2 * Math.PI);
                ctx.fill();
            }
        }
    }
    
    // New function to draw only specific points from the pose
    drawSpecificPoints(ctx, landmarks, indices, options) {
        ctx.fillStyle = options.color;
        for (const index of indices) {
            const lm = landmarks[index];
            if (lm && (lm.visibility??1) > 0.5) {
                ctx.beginPath();
                ctx.arc(lm.x * ctx.canvas.width, lm.y * ctx.canvas.height, options.radius, 0, 2 * Math.PI);
                ctx.fill();
            }
        }
    }
}

// Pre-defined connections for drawing the hand skeleton
const HAND_CONNECTIONS = [[0, 1], [1, 2], [2, 3], [3, 4], [0, 5], [5, 6], [6, 7], [7, 8], [5, 9], [9, 10], [10, 11], [11, 12], [9, 13], [13, 14], [14, 15], [15, 16], [13, 17], [17, 18], [18, 19], [19, 20], [0, 17]];