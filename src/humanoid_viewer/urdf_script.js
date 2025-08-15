import { MediaPipeHandController } from './mediapipe_pose_controller.js';

// Import BufferGeometryUtils if you plan to use it for merging DAE meshes.
// You might need to add this script tag in your HTML:
// <script src="https://unpkg.com/three@0.165.0/examples/jsm/utils/BufferGeometryUtils.js"></script>
// Then you can import it like:
// import * as BufferGeometryUtils from 'three/examples/jsm/utils/BufferGeometryUtils.js';

class ProfessionalURDFViewer {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
       this.robots = new Map(); // Stores all loaded robot groups
       this.jointData = new Map(); // Stores joints for each robot
        this.linkData = new Map(); // Stores links for each robot
        this.jointControls = {};
        this.axesHelpers = [];
        this.meshFiles = new Map(); // Stores File objects, keyed by base filename
        this.loadedGeometries = new Map(); // Stores loaded THREE.BufferGeometry, keyed by original mesh path
        this.gridHelper = null;
        this.autoRotateEnabled = false;
        this.stats = { links: 0, joints: 0, triangles: 0 };
        this.handController = null; // Ensure this property is declared in the class or constructor
   this.externalObjects = new Map(); 


         
        this.websocket = null;
        this.websocketConnected = false;
         this.debugMode = true; 
        
        this.init();
        this.connectWebSocket();
        this.setupEventListeners();
        this.animate();
    }
    
    // --- UPDATED: Method to add or update external objects for correct position ---
    syncExternalObject(objData) {
        let objMesh = this.externalObjects.get(objData.id);

        if (!objMesh) {
            let geometry, material;
            if (objData.type === 'cube') {
                // PyBullet half-extent of 0.1 means a full side length of 0.2
                geometry = new THREE.BoxGeometry(0.2, 0.2, 0.2); 
                material = new THREE.MeshBasicMaterial({ color: 0xff8c00 });
            } else if (objData.type === 'ball') {
                // PyBullet radius of 0.1 is the same for Three.js
                geometry = new THREE.SphereGeometry(0.1, 32, 32); 
                material = new THREE.MeshBasicMaterial({ color: 0x0000ff });
            }

            if (geometry && material) {
                objMesh = new THREE.Mesh(geometry, material);
                this.scene.add(objMesh);
                this.externalObjects.set(objData.id, objMesh);
            }
        }

        if (objMesh) {
            // Get the PyBullet position directly
            const pybulletPos = new THREE.Vector3(
                objData.position[0], 
                objData.position[1], 
                objData.position[2]
            );
            
            // Correctly convert the PyBullet Z-up position to Three.js Y-up
            // The Z and Y coordinates are swapped and the new Y-axis is inverted
            const threejsPos = new THREE.Vector3(pybulletPos.x, pybulletPos.z, -pybulletPos.y);

            // Correctly convert the PyBullet quaternion to Three.js
            const pybulletQuat = new THREE.Quaternion(objData.orientation[0], objData.orientation[1], objData.orientation[2], objData.orientation[3]);
            const eulerZup = new THREE.Euler().setFromQuaternion(pybulletQuat, 'XYZ');
            const threejsQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(eulerZup.x, eulerZup.z, -eulerZup.y, 'XYZ'));

            objMesh.position.copy(threejsPos);
            objMesh.quaternion.copy(threejsQuat);
        }
    }

connectWebSocket() {
        const wsUrl = 'ws://localhost:8765';
        this.updateStatus('Connecting to PyBullet...', 'info');
        this.websocket = new WebSocket(wsUrl);

        this.websocket.onopen = () => {
            this.websocketConnected = true;
            this.updateStatus('Connected to PyBullet.', 'success');
        };

        this.websocket.onmessage = (event) => {
            try {
                const msg = JSON.parse(event.data);
                if (msg.type === "sim_state" && msg.joints) {
                    // Assuming the message contains a 'joints' object directly for a single robot
                    for (const [jointName, jointValue] of Object.entries(msg.joints)) {
                        this.updateJointVisual(jointName, jointValue);
                    }
                }

                if (msg.objects && Array.isArray(msg.objects)) {
                    msg.objects.forEach(objData => {
                        this.syncExternalObject(objData);
                    });
                }
            } catch (err) {
                console.error("Error processing sim_state message:", err);
            }
        };

        this.websocket.onclose = () => {
            this.websocketConnected = false;
            this.updateStatus('Disconnected. Reconnecting...', 'error');
            setTimeout(() => this.connectWebSocket(), 3000);
        };

        this.websocket.onerror = (err) => {
            this.updateStatus('WebSocket Error. Is main.py running?', 'error');
        };
    }
    
updateJointVisual(jointName, value) {
    // 1. Get the specific joint and links maps for the 'poppy' robot
    const jointsMap = this.jointData.get('poppy');
    const linksMap = this.linkData.get('poppy');

    if (!jointsMap || !linksMap) {
        console.error('Robot data not found for:', 'poppy');
        return;
    }

    const joint = jointsMap[jointName];
    if (!joint || !linksMap[joint.child]) {
        return;
    }

    const childLinkGroup = linksMap[joint.child];
    joint.currentValue = value; // Update the internal state

    // Use the initial rotation as the base
    const initialRotation = joint.initialRotation.clone();

    // Create a new quaternion for the rotation based on the joint value and axis
    const rotationAxis = new THREE.Vector3(...joint.axis).normalize();
    const rotationQuaternion = new THREE.Quaternion().setFromAxisAngle(rotationAxis, value);

    // Apply the new rotation to the child link group
    childLinkGroup.quaternion.copy(initialRotation).multiply(rotationQuaternion);

    // Update the UI slider to reflect the new value
    const slider = this.jointControls[jointName];
    if (slider) {
        slider.value = value;
        slider.nextElementSibling.textContent = value.toFixed(2);
    }
}
    init() {
        // Scene setup with professional lighting
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0a14);
        this.scene.fog = new THREE.Fog(0x0a0a14, 10, 50);

        // Camera setup
        this.camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100);
        this.camera.position.set(4, 4, 4);
        this.camera.lookAt(0, 0, 0);

        // Renderer setup with enhanced quality
        this.renderer = new THREE.WebGLRenderer({
            antialias: true,
            alpha: true,
            powerPreference: "high-performance"
        });
        this.renderer.setSize(window.innerWidth - 350, window.innerHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputEncoding = THREE.sRGBEncoding;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.2;

        const viewportDiv = document.getElementById('viewport');
        if (viewportDiv) {
            viewportDiv.appendChild(this.renderer.domElement);
        }

        // Professional lighting setup
        this.setupLighting();

        // Grid
        this.gridHelper = new THREE.GridHelper(10, 20, 0x444466, 0x222233);
        this.gridHelper.material.transparent = true;
        this.gridHelper.material.opacity = 0.3;
        this.scene.add(this.gridHelper);

        // Mouse controls
        this.setupMouseControls();

        // Keyboard controls
        this.setupKeyboardControls();

        // MediaPipe Hand Controller Initialization
        const webcamVideoElement = document.getElementById('webcam');
        if (webcamVideoElement) {
            this.handController = new MediaPipeHandController(this, webcamVideoElement);
        } else {
            console.error("Webcam video element not found in the DOM.");
        }
    }


    setupLighting() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0x404060, 0.3);
        this.scene.add(ambientLight);

        // Main directional light
        const mainLight = new THREE.DirectionalLight(0xffffff, 1.2);
        mainLight.position.set(10, 15, 5);
        mainLight.castShadow = true;
        mainLight.shadow.mapSize.width = 2048;
        mainLight.shadow.mapSize.height = 2048;
        mainLight.shadow.camera.near = 0.5;
        mainLight.shadow.camera.far = 50;
        mainLight.shadow.camera.left = -10;
        mainLight.shadow.camera.right = 10;
        mainLight.shadow.camera.top = 10;
        mainLight.shadow.camera.bottom = -10;
        mainLight.shadow.bias = -0.0005;
        this.scene.add(mainLight);

        // Fill light
        const fillLight = new THREE.DirectionalLight(0x6366f1, 0.4);
        fillLight.position.set(-5, 5, -5);
        this.scene.add(fillLight);

        // Rim light
        const rimLight = new THREE.DirectionalLight(0xa855f7, 0.3);
        rimLight.position.set(0, -5, 10);
        this.scene.add(rimLight);

        // Point lights for dramatic effect
        const pointLight1 = new THREE.PointLight(0x6366f1, 0.5, 20);
        pointLight1.position.set(5, 3, 5);
        this.scene.add(pointLight1);

        const pointLight2 = new THREE.PointLight(0xa855f7, 0.3, 15);
        pointLight2.position.set(-3, 2, -3);
        this.scene.add(pointLight2);
    }

    setupMouseControls() {
        let isDragging = false;
        let previousMousePosition = { x: 0, y: 0 };
        const element = document.getElementById('viewport');

        element.addEventListener('mousedown', (e) => {
            isDragging = true;
            previousMousePosition = { x: e.clientX, y: e.clientY };
            element.style.cursor = 'grabbing';
        });

        element.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const deltaMove = {
                    x: e.clientX - previousMousePosition.x,
                    y: e.clientY - previousMousePosition.y
                };

                const spherical = new THREE.Spherical();
                spherical.setFromVector3(this.camera.position);

                spherical.theta -= deltaMove.x * 0.01;
                spherical.phi += deltaMove.y * 0.01;
                spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));

                this.camera.position.setFromSpherical(spherical);
                this.camera.lookAt(0, 0, 0);

                previousMousePosition = { x: e.clientX, y: e.clientY };
            }
        });

        element.addEventListener('mouseup', () => {
            isDragging = false;
            element.style.cursor = 'grab';
        });

        element.addEventListener('mouseleave', () => {
            isDragging = false;
            element.style.cursor = 'default';
        });

        element.addEventListener('wheel', (e) => {
            e.preventDefault();
            const scale = e.deltaY > 0 ? 1.1 : 0.9;
            this.camera.position.multiplyScalar(scale);
            const distance = this.camera.position.length();
            if (distance > 30) this.camera.position.normalize().multiplyScalar(30);
            if (distance < 1) this.camera.position.normalize().multiplyScalar(1);
        });

        element.style.cursor = 'grab';
    }

    resetCamera() {
        this.camera.position.set(4, 4, 4);
        this.camera.lookAt(0, 0, 0);
    }

    setupKeyboardControls() {
        document.addEventListener('keydown', (e) => {
            switch(e.key.toLowerCase()) {
                case 'r':
                    this.resetCamera();
                    break;
                case 'g':
                    document.getElementById('showGrid').click();
                    break;
                case 'w':
                    document.getElementById('wireframe').click();
                    break;
            }
        });
    }

   // Updated setupEventListeners function
setupEventListeners() {
    const urdfInput = document.getElementById('urdfInput');
    const meshInput = document.getElementById('meshInput');
    const loadButton = document.getElementById('loadButton');

    const checkInputs = () => {
        loadButton.disabled = !urdfInput.files.length;

        const urdfWrapper = document.getElementById('urdfWrapper');
        if (urdfInput.files.length) {
            urdfWrapper.classList.add('has-files');
            urdfWrapper.querySelector('.file-input-text').innerHTML =
                `<strong>Selected:</strong> ${urdfInput.files[0].name}`;
        } else {
            urdfWrapper.classList.remove('has-files');
            urdfWrapper.querySelector('.file-input-text').innerHTML =
                '<strong>Click to select</strong> or drag URDF file here';
        }
    };

    if (urdfInput) urdfInput.addEventListener('change', checkInputs);
    if (loadButton) {
        loadButton.addEventListener('click', async () => { 
            const files = urdfInput.files;
            if (files.length > 0) {
                for (const file of files) {
                    const urdfContent = await this.readFile(file);
                    // Pass the file.name as the robotName to the loading function
                    const robotName = file.name.split('.')[0].toLowerCase();
                    await this._loadRobotInternal(urdfContent, robotName); 
                }
            }
        });
    }

    if (meshInput) {
        meshInput.addEventListener('change', (e) => {
            this.processMeshFiles(e.target.files);

            const meshWrapper = document.getElementById('meshWrapper');
            if (e.target.files.length) {
                meshWrapper.classList.add('has-files');
                meshWrapper.querySelector('.file-input-text').innerHTML =
                    `<strong>Selected:</strong> ${e.target.files.length} mesh files`;
            } else {
                meshWrapper.classList.remove('has-files');
                meshWrapper.querySelector('.file-input-text').innerHTML =
                    '<strong>Click to select</strong> mesh files or drag here';
            }
        });
    }

    // Event listeners for checkboxes (keep these)
    document.getElementById('showAxes').addEventListener('change', (e) => {
        this.toggleAxes(e.target.checked);
    });

    document.getElementById('wireframe').addEventListener('change', (e) => {
        this.toggleWireframe(e.target.checked);
    });

    document.getElementById('showGrid').addEventListener('change', (e) => {
        this.toggleGrid(e.target.checked);
    });

    document.getElementById('autoRotate').addEventListener('change', (e) => {
        this.autoRotateEnabled = e.target.checked;
    });

    checkInputs();
}

    processMeshFiles(files) {
        this.meshFiles.clear();
        for (let file of files) {
            const baseName = file.name.split('/').pop().split('.')[0];
            this.meshFiles.set(file.name.split('/').pop(), file);
            this.meshFiles.set(baseName, file);
        }
        this.updateStatus(`Loaded ${files.length} potential mesh files for lookup.`, 'success');
    }

    // New method to load robot from file input (if UI is still used)
    async loadRobotFromFileInput() {
        const urdfFile = document.getElementById('urdfInput').files[0];
        if (!urdfFile) {
            this.updateStatus('Please select a URDF file.', 'error');
            return;
        }

        this.updateStatus('Loading robot...', 'loading');
        document.getElementById('loadButtonText').innerHTML =
            '<div class="loading-spinner"></div>Loading...';

        try {
            const urdfContent = await this.readFile(urdfFile);
            await this._loadRobotInternal(urdfContent); // Use internal loading function
            this.updateStatus('Robot loaded successfully!', 'success');
            document.getElementById('loadButtonText').textContent = 'Load Robot';
        } catch (error) {
            console.error('Error loading robot:', error);
            this.updateStatus('Error loading robot: ' + error.message, 'error');
            document.getElementById('loadButtonText').textContent = 'Load Robot';
        }
    }

    // Centralized internal loading logic
   async _loadRobotInternal(urdfContent, robotName) {
    // 1. Create a new group for this specific robot
    const newRobotGroup = new THREE.Group();
    newRobotGroup.name = robotName;
    this.scene.add(newRobotGroup);

    // 2. Store the new robot group in a map
    this.robots.set(robotName, newRobotGroup);

    // 3. Create new maps for this robot's joints and links
    const newJointsMap = {};
    const newLinksMap = {};
    this.jointData.set(robotName, newJointsMap);
    this.linkData.set(robotName, newLinksMap);

    // 4. Clear and reset helper data for just this new robot
    this.axesHelpers = [];
    this.loadedGeometries.clear();

    // 5. Call parseURDF with the new, specific data structures
    await this.parseURDF(urdfContent, newRobotGroup, newLinksMap, newJointsMap);

    // 6. Set an initial position for the new robot to prevent overlapping
if (robotName === 'poppy') {
            newRobotGroup.position.set(0, 0.5, 0);
        } 
        else if (robotName.toLowerCase().includes('kitchen')) {
        // Position Kitchen
        newRobotGroup.position.set(0.4, 0, 0);
    }
        else {
            // Position other robots in a staggered way
            const robotCount = this.robots.size;
            newRobotGroup.position.set((robotCount - 1) * 3, 0, 0);
        }
    newRobotGroup.rotation.x = -Math.PI / 2;

    
    // 7. Rebuild the joint controls UI to include the new robot's joints
    this.createJointControls();

    // 8. Adjust the camera and update the stats
    this.fitCameraToObject();
    this.updateStatsDisplay();
}
    // This is the new function to be called for automatic loading
    async loadPoppyHumanoid() {
        this.updateStatus('Automatically loading Poppy Humanoid...', 'loading');

        // Step 1: Define the path to your URDF file
        // Make sure this path is correct relative to your HTML file.
        const urdfPath = '/assets/robot2/URDF/Poppy_Humanoid.URDF'; // Adjusted for Vite public directory

        // Step 2: Define the paths for your mesh files.
        // It's crucial that these paths match what's in your URDF's <mesh filename="..."> tags.
        // The key in the map should be the filename as it appears in the URDF,
        // and the value should be the actual URL to load the mesh.
        // For local files, this will be relative paths.

        const meshFilesMap = new Map();
        meshFilesMap.set('abdomen_respondable.STL', '/assets/robot2/meshes/abdomen_respondable.STL');
        meshFilesMap.set('abdomen_visual.STL', '/assets/robot2/meshes/abdomen_visual.STL');
        meshFilesMap.set('abs_motors_respondable.stl', '/assets/robot2/meshes/abs_motors_respondable.stl');
        meshFilesMap.set('abs_motors_visual.STL', '/assets/robot2/meshes/abs_motors_visual.STL');
        meshFilesMap.set('bust_motors_respondable.stl', '/assets/robot2/meshes/bust_motors_respondable.stl');
        meshFilesMap.set('bust_motors_visual.STL', '/assets/robot2/meshes/bust_motors_visual.STL');
        meshFilesMap.set('chest_respondable.STL', '/assets/robot2/meshes/chest_respondable.STL');
        meshFilesMap.set('chest_visual.STL', '/assets/robot2/meshes/chest_visual.STL');
        meshFilesMap.set('head_respondable.stl', '/assets/robot2/meshes/head_respondable.stl');
        meshFilesMap.set('head_visual.STL', '/assets/robot2/meshes/head_visual.STL');
        meshFilesMap.set('l_foot_respondable.stl', '/assets/robot2/meshes/l_foot_respondable.stl');
        meshFilesMap.set('l_foot_visual.STL', '/assets/robot2/meshes/l_foot_visual.STL');
        meshFilesMap.set('l_forearm_respondable.stl', '/assets/robot2/meshes/l_forearm_respondable.stl');
        meshFilesMap.set('l_forearm_visual.STL', '/assets/robot2/meshes/l_forearm_visual.STL');
        meshFilesMap.set('l_hip_motor_respondable.stl', '/assets/robot2/meshes/l_hip_motor_respondable.stl');
        meshFilesMap.set('l_hip_motor_visual.STL', '/assets/robot2/meshes/l_hip_motor_visual.STL');
        meshFilesMap.set('l_hip_respondable.stl', '/assets/robot2/meshes/l_hip_respondable.stl');
        meshFilesMap.set('l_hip_visual.STL', '/assets/robot2/meshes/l_hip_visual.STL');
        meshFilesMap.set('l_shin_respondable.stl', '/assets/robot2/meshes/l_shin_respondable.stl');
        meshFilesMap.set('l_shin_visual.STL', '/assets/robot2/meshes/l_shin_visual.STL');
        meshFilesMap.set('l_shoulder_motor_respondable.stl', '/assets/robot2/meshes/l_shoulder_motor_respondable.stl');
        meshFilesMap.set('l_shoulder_motor_visual.STL', '/assets/robot2/meshes/l_shoulder_motor_visual.STL');
        meshFilesMap.set('l_shoulder_respondable.stl', '/assets/robot2/meshes/l_shoulder_respondable.stl');
        meshFilesMap.set('l_shoulder_visual.STL', '/assets/robot2/meshes/l_shoulder_visual.STL');
        meshFilesMap.set('l_thigh_respondable.STL', '/assets/robot2/meshes/l_thigh_respondable.STL');
        meshFilesMap.set('l_thigh_visual.STL', '/assets/robot2/meshes/l_thigh_visual.STL');
        meshFilesMap.set('l_upper_arm_respondable.STL', '/assets/robot2/meshes/l_upper_arm_respondable.STL');
        meshFilesMap.set('l_upper_arm_visual.STL', '/assets/robot2/meshes/l_upper_arm_visual.STL');
        meshFilesMap.set('neck_respondable.STL', '/assets/robot2/meshes/neck_respondable.STL');
        meshFilesMap.set('neck_visual.STL', '/assets/robot2/meshes/neck_visual.STL');
        meshFilesMap.set('pelvis_respondable.STL', '/assets/robot2/meshes/pelvis_respondable.STL');
        meshFilesMap.set('pelvis_visual.STL', '/assets/robot2/meshes/pelvis_visual.STL');
        meshFilesMap.set('r_foot_respondable.STL', '/assets/robot2/meshes/r_foot_respondable.STL');
        meshFilesMap.set('r_foot_visual.STL', '/assets/robot2/meshes/r_foot_visual.STL');
        meshFilesMap.set('r_forearm_respondable.STL', '/assets/robot2/meshes/r_forearm_respondable.STL');
        meshFilesMap.set('r_forearm_visual.STL', '/assets/robot2/meshes/r_forearm_visual.STL');
        meshFilesMap.set('r_hip_motor_respondable.STL', '/assets/robot2/meshes/r_hip_motor_respondable.STL');
        meshFilesMap.set('r_hip_motor_visual.STL', '/assets/robot2/meshes/r_hip_motor_visual.STL');
        meshFilesMap.set('r_hip_respondable.STL', '/assets/robot2/meshes/r_hip_respondable.STL');
        meshFilesMap.set('r_hip_visual.STL', '/assets/robot2/meshes/r_hip_visual.STL');
        meshFilesMap.set('r_shin_respondable.STL', '/assets/robot2/meshes/r_shin_respondable.STL');
        meshFilesMap.set('r_shin_visual.STL', '/assets/robot2/meshes/r_shin_visual.STL');
        meshFilesMap.set('r_shoulder_motor_respondable.STL', '/assets/robot2/meshes/r_shoulder_motor_respondable.STL');
        meshFilesMap.set('r_shoulder_motor_visual.STL', '/assets/robot2/meshes/r_shoulder_motor_visual.STL');
        meshFilesMap.set('r_shoulder_respondable.STL', '/assets/robot2/meshes/r_shoulder_respondable.STL');
        meshFilesMap.set('r_shoulder_visual.STL', '/assets/robot2/meshes/r_shoulder_visual.STL');
        meshFilesMap.set('r_thigh_respondable.STL', '/assets/robot2/meshes/r_thigh_respondable.STL');
        meshFilesMap.set('r_thigh_visual.STL', '/assets/robot2/meshes/r_thigh_visual.STL');
        meshFilesMap.set('r_upper_arm_respondable.STL', '/assets/robot2/meshes/r_upper_arm_respondable.STL');
        meshFilesMap.set('r_upper_arm_visual.STL', '/assets/robot2/meshes/r_upper_arm_visual.STL');
        meshFilesMap.set('spine_respondable.STL', '/assets/robot2/meshes/spine_respondable.STL');
        meshFilesMap.set('spine_visual.STL', '/assets/robot2/meshes/spine_visual.STL');


        // Populate the meshFiles map with dummy File objects that fetch from URLs
        this.meshFiles.clear();
        for (const [filenameInUrdf, url] of meshFilesMap.entries()) {
            // Create a pseudo-File object that our loader can understand
            // We don't have a true File object from an input, so we use a Blob/Response
            // This is a workaround to fit the existing loadMeshFile signature.
            const dummyFile = {
                name: filenameInUrdf, // This is important for the existing logic
                // Instead of a true File object, we'll store the URL
                // The loadMeshFile will need to be updated to handle URLs.
                url: url
            };
            this.meshFiles.set(filenameInUrdf, dummyFile);
            this.meshFiles.set(filenameInUrdf.split('.')[0], dummyFile); // Also store by base name
        }
        this.updateStatus(`Prepared ${this.meshFiles.size / 2} mesh file entries for automatic loading.`, 'info');


        try {
            // Fetch the URDF content directly
            const response = await fetch(urdfPath);
            if (!response.ok) {
                throw new Error(`Failed to fetch URDF file: ${response.statusText}`);
            }
            const urdfContent = await response.text();
 const robotName = 'poppy';
            await this._loadRobotInternal(urdfContent,robotName); // Use internal loading function

            this.updateStatus('Poppy Humanoid loaded successfully!', 'success');
            // If you have a loading indicator for manual load, you might want to hide it here too
            const loadButtonText = document.getElementById('loadButtonText');
            if (loadButtonText) loadButtonText.textContent = 'Load Robot';

            // Set initial pose for Poppy Humanoid (adjust as needed)
            this.setPoppyHumanoidInitialPose();

        } catch (error) {
            console.error('Error automatically loading Poppy Humanoid:', error);
            this.updateStatus('Error automatically loading Poppy Humanoid: ' + error.message, 'error');
            const loadButtonText = document.getElementById('loadButtonText');
            if (loadButtonText) loadButtonText.textContent = 'Load Robot';
        }
    }

    // New function to set the initial pose for Poppy Humanoid
    setPoppyHumanoidInitialPose() {
        // These are example joint names and values.
        // You'll need to consult the Poppy Humanoid URDF to get the exact joint names
        // and experiment with values to get the desired starting pose.
        // Angles are in radians. Math.PI is 180 degrees.

        if (this.joints['r_shoulder_y_joint']) { // Example joint
            this.updateJoint('r_shoulder_y_joint', -Math.PI / 4); // Rotate 45 degrees backward
        }
        if (this.joints['l_shoulder_y_joint']) { // Example joint
            this.updateJoint('l_shoulder_y_joint', Math.PI / 4); // Rotate 45 degrees forward
        }
        if (this.joints['r_elbow_y_joint']) { // Example joint
            this.updateJoint('r_elbow_y_joint', -Math.PI / 2); // Bend elbow 90 degrees
        }
        if (this.joints['l_elbow_y_joint']) { // Example joint
            this.updateJoint('l_elbow_y_joint', Math.PI / 2); // Bend elbow 90 degrees
        }
        if (this.joints['head_y_joint']) { // Example joint
             this.updateJoint('head_y_joint', 0.1); // Slight head turn
        }
        // Add more specific joint adjustments here for Poppy Humanoid
        // ... (e.g., hip, knee, ankle joints)

        // After setting all joint values, ensure UI sliders reflect the changes if they exist
        for (const jointName in this.jointControls) {
            const slider = this.jointControls[jointName];
            const joint = this.joints[jointName];
            if (slider && joint) {
                slider.value = joint.currentValue;
                slider.nextElementSibling.textContent = joint.currentValue.toFixed(2);
            }
        }
    }


    readFile(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = e => resolve(e.target.result);
            reader.onerror = reject;
            reader.readAsText(file);
        });
    }

    async parseURDF(urdfContent, targetGroup, linksMap, jointsMap) {
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(urdfContent, "text/xml");

    const rootElement = xmlDoc.getElementsByTagName('robot')[0];
    if (!rootElement) {
        throw new Error("Invalid URDF: No 'robot' tag found.");
    }

    const links = xmlDoc.getElementsByTagName('link');
    for (let link of links) {
        // Use the linksMap for storing links
        await this.parseLink(link, linksMap); 
    }

    const joints = xmlDoc.getElementsByTagName('joint');
    for (let joint of joints) {
        // Use the jointsMap for storing joints
        this.parseJoint(joint, jointsMap);
    }

    let rootLinkName = null;
    const childLinkNames = new Set(Object.values(jointsMap).map(j => j.child)); // Use jointsMap
    for (const linkName in linksMap) { // Use linksMap
        if (!childLinkNames.has(linkName)) {
            rootLinkName = linkName;
            break;
        }
    }

    // Attach the root link to the targetGroup
    if (rootLinkName && linksMap[rootLinkName]) { // Use linksMap
        targetGroup.add(linksMap[rootLinkName]);
    } else if (Object.keys(linksMap).length > 0) {
        console.warn("Could not determine a clear root link. Adding first link to scene root.");
        targetGroup.add(Object.values(linksMap)[0]);
    }

    // Attach child links to their parents
    for (const jointName in jointsMap) { // Use jointsMap
        const joint = jointsMap[jointName];
        const parentLinkGroup = linksMap[joint.parent]; // Use linksMap
        const childLinkGroup = linksMap[joint.child]; // Use linksMap

        if (parentLinkGroup && childLinkGroup) {
            parentLinkGroup.add(childLinkGroup);

            if (joint.origin) {
                this.applyOrigin(childLinkGroup, joint.origin);
            }

            joint.initialPosition = childLinkGroup.position.clone();
            joint.initialRotation = childLinkGroup.quaternion.clone();

            const axisHelper = new THREE.ArrowHelper(
                new THREE.Vector3(...joint.axis).normalize(),
                new THREE.Vector3(0, 0, 0),
                0.1,
                0xff0000,
                0.05,
                0.02
            );
            childLinkGroup.add(axisHelper);
            this.axesHelpers.push(axisHelper); // axesHelpers can remain class-wide
        } else {
            console.warn(`Missing parent or child link for joint: ${jointName}`);
        }
    }

    // The createJointControls function will be called once after parsing all robots
    // so we can remove this call from here.
}


    updateStatsDisplay() {
    let totalLinkCount = 0;
    let totalJointCount = 0;
    let totalTriangleCount = 0;

    // 1. Sum up link counts from all loaded robots
    for (const linksMap of this.linkData.values()) {
        totalLinkCount += Object.keys(linksMap).length;
    }

    // 2. Sum up joint counts from all loaded robots
    for (const jointsMap of this.jointData.values()) {
        totalJointCount += Object.keys(jointsMap).length;
    }

    // 3. Sum up triangle counts from all loaded robots
    if (this.robots) {
        for (const robotGroup of this.robots.values()) {
            robotGroup.traverse(child => {
                if (child.isMesh && child.geometry) {
                    const positions = child.geometry.attributes.position;
                    if (positions) {
                        totalTriangleCount += positions.count / 3;
                    }
                }
            });
        }
    }
    
    // 4. Update the UI with the new, aggregated counts
    document.getElementById('linkCount').textContent = totalLinkCount;
    document.getElementById('jointCount').textContent = totalJointCount;
    this.stats.triangles = Math.floor(totalTriangleCount);
    document.getElementById('triangleCount').textContent = this.stats.triangles;
}

    async parseLink(linkElement,linksMap) {
        const name = linkElement.getAttribute('name');
        const linkGroup = new THREE.Group();
        linkGroup.name = name;

        const visuals = linkElement.getElementsByTagName('visual');
        for (let visual of visuals) {
            const mesh = await this.createVisualMesh(visual);
            if (mesh) {
                linkGroup.add(mesh);
            }
        }

        linksMap[name] = linkGroup;
    }

    async createVisualMesh(visual) {
        const geometryElement = visual.getElementsByTagName('geometry')[0];
        if (!geometryElement) return null;

        let geometry = null;
        let material = this.parseMaterial(visual.getElementsByTagName('material')[0]);

        const meshElement = geometryElement.getElementsByTagName('mesh')[0];
        if (meshElement) {
            geometry = await this.loadMeshGeometry(meshElement);
        } else {
            geometry = this.parseGeometry(geometryElement);
        }

        if (!geometry) return null;

        const mesh = new THREE.Mesh(geometry, material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;

        const origin = visual.getElementsByTagName('origin')[0];
        if (origin) {
            this.applyOrigin(mesh, origin);
        }

        return mesh;
    }

    async loadMeshGeometry(meshElement) {
        const filename = meshElement.getAttribute('filename');
        if (!filename) return null;

        let meshPath = filename;
        if (meshPath.startsWith('package://')) {
            meshPath = meshPath.substring('package://'.length);
        }

        const meshBaseName = meshPath.split('/').pop();
        const meshExtension = meshBaseName.split('.').pop().toLowerCase();

        if (this.loadedGeometries.has(meshPath)) {
            return this.loadedGeometries.get(meshPath).clone();
        }

        // IMPORTANT CHANGE HERE: Now meshFile can be a dummy object with a 'url' property
        let meshFile = this.meshFiles.get(meshBaseName);
        if (!meshFile) {
            console.warn(`Mesh file not found in provided files/map: ${meshPath}. Using fallback cube.`);
            return new THREE.BoxGeometry(0.1, 0.1, 0.1);
        }

        try {
            // Pass the URL or a true File object
            const geometry = await this.loadMeshFile(meshFile.url || meshFile, meshExtension);
            this.loadedGeometries.set(meshPath, geometry);

            const scale = meshElement.getAttribute('scale');
            if (scale && geometry) {
                const scaleValues = scale.split(' ').map(Number);
                const sx = scaleValues[0] !== undefined ? scaleValues[0] : 1;
                const sy = scaleValues[1] !== undefined ? scaleValues[1] : 1;
                sz = scaleValues[2] !== undefined ? scaleValues[2] : 1;
                geometry.scale(sx, sy, sz);
            }

            return geometry;
        } catch (error) {
            console.error(`Error loading mesh ${meshPath}:`, error);
            this.updateStatus(`Error loading mesh ${meshBaseName}. Check console.`, 'error');
            return new THREE.BoxGeometry(0.1, 0.1, 0.1);
        }
    }

    // IMPORTANT CHANGE HERE: Updated to handle both File objects and URLs
    loadMeshFile(fileOrUrl, extension) {
        return new Promise(async (resolve, reject) => {
            let data;
            if (typeof fileOrUrl === 'string') { // It's a URL
                try {
                    const response = await fetch(fileOrUrl);
                    if (!response.ok) {
                        throw new Error(`Failed to fetch mesh from URL: ${fileOrUrl} - ${response.statusText}`);
                    }
                    if (extension === 'stl') {
                        data = await response.arrayBuffer();
                    } else {
                        data = await response.text();
                    }
                } catch (fetchError) {
                    reject(fetchError);
                    return;
                }
            } else { // It's a File object (from user input)
                const reader = new FileReader();
                reader.onerror = reject;

                reader.onload = (e) => {
                    data = e.target.result;
                    this.processLoadedMeshData(data, extension, fileOrUrl.name, resolve, reject);
                };

                if (extension === 'stl') {
                    reader.readAsArrayBuffer(fileOrUrl);
                } else if (extension === 'dae') {
                    reader.readAsText(fileOrUrl);
                } else {
                    reader.readAsText(fileOrUrl);
                }
                return; // FileReader handles resolution/rejection via its onload/onerror
            }
            // If data was fetched directly from URL, process it now
            this.processLoadedMeshData(data, extension, typeof fileOrUrl === 'string' ? fileOrUrl : fileOrUrl.name, resolve, reject);
        });
    }


    // Helper function to process loaded mesh data
    processLoadedMeshData(data, extension, filename, resolve, reject) {
        try {
            let geometry;
            if (extension === 'stl') {
                const loader = new THREE.STLLoader();
                geometry = loader.parse(data);
            } else if (extension === 'obj') {
                const loader = new THREE.OBJLoader();
                const objGroup = loader.parse(data);
                if (objGroup.children.length > 0 && objGroup.children[0].isMesh) {
                    geometry = objGroup.children[0].geometry;
                } else {
                    throw new Error("OBJ file did not contain a mesh with geometry.");
                }
            } else if (extension === 'dae') {
                const loader = new THREE.ColladaLoader();
                loader.parse(data, (collada) => {
                    const meshes = [];
                    collada.scene.traverse((child) => {
                        if (child.isMesh && child.geometry) {
                            meshes.push(child.geometry.isBufferGeometry ? child.geometry : new THREE.BufferGeometry().fromGeometry(child.geometry));
                        }
                    });

                    if (meshes.length > 0) {
                        if (meshes.length > 1) {
                            // If you imported BufferGeometryUtils, uncomment this:
                            // try {
                            //     const mergedGeometry = THREE.BufferGeometryUtils.mergeBufferGeometries(meshes);
                            //     if (mergedGeometry) {
                            //         resolve(mergedGeometry);
                            //         return;
                            //     }
                            // } catch (mergeError) {
                            //     console.warn(`Could not merge geometries for DAE file "${filename}": ${mergeError.message}. Using first mesh only.`);
                            // }
                        }
                        resolve(meshes[0]);
                    } else {
                        reject(new Error(`DAE file "${filename}" contained no meshes.`));
                    }
                }, (xhr) => { /* progress */ }, (error) => {
                    reject(new Error(`Error loading DAE "${filename}": ${error}`));
                });
                return;
            } else {
                throw new Error(`Unsupported mesh format: .${extension} for file ${filename}`);
            }

            if (geometry && !geometry.attributes.normal) {
                geometry.computeVertexNormals();
            }

            resolve(geometry);
        } catch (error) {
            reject(error);
        }
    }


    parseGeometry(geometryElement) {
        const box = geometryElement.getElementsByTagName('box')[0];
        if (box) {
            const size = box.getAttribute('size').split(' ').map(Number);
            return new THREE.BoxGeometry(size[0], size[1], size[2]);
        }

        const cylinder = geometryElement.getElementsByTagName('cylinder')[0];
        if (cylinder) {
            const radius = parseFloat(cylinder.getAttribute('radius'));
            const length = parseFloat(cylinder.getAttribute('length'));
            const geometry = new THREE.CylinderGeometry(radius, radius, length, 32);
            geometry.rotateX(Math.PI / 2);
            return geometry;
        }

        const sphere = geometryElement.getElementsByTagName('sphere')[0];
        if (sphere) {
            const radius = parseFloat(sphere.getAttribute('radius'));
            return new THREE.SphereGeometry(radius, 32, 32);
        }

        console.warn("Unknown primitive geometry type. Using fallback cube.");
        return new THREE.BoxGeometry(0.1, 0.1, 0.1);
    }

    parseMaterial(materialElement) {
        const material = new THREE.MeshStandardMaterial({
            color: 0x888888,
            metalness: 0.2,
            roughness: 0.7,
            envMapIntensity: 0.9
        });

        if (materialElement) {
            const colorElement = materialElement.getElementsByTagName('color')[0];
            if (colorElement) {
                const rgba = colorElement.getAttribute('rgba').split(' ').map(Number);
                material.color.setRGB(rgba[0], rgba[1], rgba[2]);
                if (rgba[3] < 1) {
                    material.transparent = true;
                    material.opacity = rgba[3];
                }
            }
        }

        return material;
    }

    parseJoint(jointElement,jointsMap) {
        const name = jointElement.getAttribute('name');
        const type = jointElement.getAttribute('type');
        const parent = jointElement.getElementsByTagName('parent')[0]?.getAttribute('link');
        const child = jointElement.getElementsByTagName('child')[0]?.getAttribute('link');

        if (!parent || !child) {
            console.warn(`Joint ${name} is missing parent or child link.`);
            return;
        }

        const joint = {
            name: name,
            type: type,
            parent: parent,
            child: child,
            currentValue: 0,
            origin: {
                xyz: [0, 0, 0],
                rpy: [0, 0, 0]
            }
        };

        const originElement = jointElement.getElementsByTagName('origin')[0];
        if (originElement) {
            if (originElement.hasAttribute('xyz')) {
                joint.origin.xyz = originElement.getAttribute('xyz').split(' ').map(Number);
            }
            if (originElement.hasAttribute('rpy')) {
                joint.origin.rpy = originElement.getAttribute('rpy').split(' ').map(Number);
            }
        }

        const axis = jointElement.getElementsByTagName('axis')[0];
        if (axis) {
            joint.axis = axis.getAttribute('xyz').split(' ').map(Number);
        } else {
            joint.axis = [1, 0, 0];
        }

        const limit = jointElement.getElementsByTagName('limit')[0];
        if (limit) {
            joint.lower = parseFloat(limit.getAttribute('lower') || 0);
            joint.upper = parseFloat(limit.getAttribute('upper') || 0);
            if (type === 'continuous') {
                joint.lower = -Math.PI * 2;
                joint.upper = Math.PI * 2;
            }
        } else {
            if (type === 'revolute') {
                joint.lower = -Math.PI;
                joint.upper = Math.PI;
            } else if (type === 'prismatic') {
                joint.lower = -0.5;
                joint.upper = 0.5;
            }
        }

        jointsMap[name] = joint;
    }

    applyOrigin(object, originData) {
        let xyz = [0, 0, 0];
        let rpy = [0, 0, 0];

        if (originData instanceof Element) {
            if (originData.hasAttribute('xyz')) {
                xyz = originData.getAttribute('xyz').split(' ').map(Number);
            }
            if (originData.hasAttribute('rpy')) {
                rpy = originData.getAttribute('rpy').split(' ').map(Number);
            }
        }
        else if (originData && typeof originData === 'object' && Array.isArray(originData.xyz) && Array.isArray(originData.rpy)) {
            xyz = originData.xyz;
            rpy = originData.rpy;
        } else {
            object.position.set(0, 0, 0);
            object.rotation.set(0, 0, 0, 'ZYX');
            return;
        }

        object.position.set(xyz[0], xyz[1], xyz[2]);
        object.rotation.order = 'ZYX';
        object.rotation.set(rpy[0], rpy[1], rpy[2]);
    }

    createJointControls() {
        const controlsDiv = document.getElementById('jointControls');
        if (!controlsDiv) {
            console.warn("Joint controls div not found.");
            return;
        }
        controlsDiv.innerHTML = '';

        Object.keys(this.joints).forEach(jointName => {
            const joint = this.joints[jointName];

            if (joint.type === 'revolute' || joint.type === 'continuous' || joint.type === 'prismatic') {
                const controlDiv = document.createElement('div');
                controlDiv.className = 'joint-control';

                const label = document.createElement('label');
                label.textContent = jointName;

                const slider = document.createElement('input');
                slider.type = 'range';
                slider.min = joint.lower;
                slider.max = joint.upper;
                slider.step = (joint.upper - joint.lower) / 1000;
                slider.value = 0;

                const valueDisplay = document.createElement('span');
                valueDisplay.className = 'value';
                valueDisplay.textContent = '0.00';

                slider.addEventListener('input', (e) => {
                    const value = parseFloat(e.target.value);
                    this.updateJoint(jointName, value);
                    valueDisplay.textContent = value.toFixed(2);
                });

                controlDiv.appendChild(label);
                controlDiv.appendChild(slider);
                controlDiv.appendChild(valueDisplay);
                controlsDiv.appendChild(controlDiv);

                this.jointControls[jointName] = slider;
                this.updateJoint(jointName, 0);
            }
        });
    }

    updateJoint(jointName, value) {
    console.log('Received joint update:', jointName, value); 
    // Check if the WebSocket exists and is in the OPEN state (readyState === 1)
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        const message = {
            type: 'joint_command',
            joint_name: jointName,
            target_position: value
        };
        this.websocket.send(JSON.stringify(message));
    } else {
        // Optional: Add a debug line to see when the message is not sent
        console.warn('WebSocket not open. Not sending joint command.');
    }
}

    toggleAxes(show) {
        this.axesHelpers.forEach(helper => {
            helper.visible = show;
        });
    }

    toggleWireframe(wireframe) {
        if (this.robots) {
            this.robots.traverse(child => {
                if (child.isMesh && child.material) {
                    if (Array.isArray(child.material)) {
                        child.material.forEach(mat => {
                            if (mat.isMaterial) mat.wireframe = wireframe;
                        });
                    } else if (child.material.isMaterial) {
                        child.material.wireframe = wireframe;
                    }
                }
            });
        }
    }

    toggleGrid(show) {
        if (this.gridHelper) {
            this.gridHelper.visible = show;
        }
    }

    fitCameraToObject() {
        if (!this.robots || !this.robots.children.length) {
            this.resetCamera();
            return;
        }

        const box = new THREE.Box3().setFromObject(this.robots);
        if (box.isEmpty()) {
            this.resetCamera();
            return;
        }

        const size = box.getSize(new THREE.Vector3());
        const center = box.getCenter(new THREE.Vector3());

        const maxDim = Math.max(size.x, size.y, size.z);
        const fov = this.camera.fov * (Math.PI / 180);
        let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));

        cameraZ *= 1.5;

        this.camera.position.set(center.x + cameraZ, center.y + cameraZ, center.z + cameraZ);
        this.camera.lookAt(center);
    }

    updateStatus(message, type) {
        const statusDiv = document.getElementById('status');
        if (statusDiv) { // Check if the element exists before trying to modify it
            statusDiv.textContent = message;
            statusDiv.className = `status-${type}`;
        }
    }

    animate() {
        requestAnimationFrame(() => this.animate());

        if (this.autoRotateEnabled) {
            this.robots.rotation.z += 0.005;
        }

        this.renderer.render(this.scene, this.camera);
    }
}
window.addEventListener('DOMContentLoaded',  () => {
    window.viewer = new ProfessionalURDFViewer();
    // Await both loading functions to ensure they complete
    // The loadPoppyHumanoid and loadKitchen functions will now handle their own URDFs and meshes
     window.viewer.loadPoppyHumanoid();
   // await window.viewer.loadKitchen(); 
});

window.addEventListener('resize', () => {
    if (window.viewer) {
        window.viewer.camera.aspect = (window.innerWidth - 350) / window.innerHeight;
        window.viewer.camera.updateProjectionMatrix();
        window.viewer.renderer.setSize(window.innerWidth - 350, window.innerHeight);
    }
});


/*
import { MediaPipeHandController } from './mediapipe_pose_controller.js';

class HybridViewController {
    constructor() {
        this.joints = {};
        this.objects = {}; // <-- NEW: store dynamic objects like the ball
        this.websocket = null;
        this.websocketConnected = false;
        this.init();
    }

    init() {
        this.connectWebSocket();
        this.setupEventListeners();
        // The MediaPipe controller will be initialized automatically from its own file
    }

    connectWebSocket() {
        const wsUrl = 'ws://localhost:8765';
        this.updateStatus('Connecting to PyBullet...', 'info');
        this.websocket = new WebSocket(wsUrl);

        this.websocket.onopen = () => {
            this.websocketConnected = true;
            this.updateStatus('Connected to PyBullet.', 'success');
        };

        // --- NEW: Listen for state updates from backend ---
        this.websocket.onmessage = (event) => {
            try {
                const msg = JSON.parse(event.data);
                if (msg.type === "sim_state") {
                    // Update robot joints if your viewer supports it
                    for (const [jointName, jointValue] of Object.entries(msg.joints)) {
                        if (typeof this.updateJointVisual === "function") {
                            this.updateJointVisual(jointName, jointValue);
                        }
                    }

                    // Update or create dynamic objects like the ball
                    msg.objects.forEach(obj => {
                        if (!this.objects[obj.name]) {
                            if (obj.name === "ball") {
                                const geometry = new THREE.SphereGeometry(0.05, 32, 32);
                                const material = new THREE.MeshStandardMaterial({ color: 0xff0000 });
                                const sphere = new THREE.Mesh(geometry, material);
                                sphere.castShadow = true;
                                sphere.receiveShadow = true;
                                if (this.scene) {
                                    this.scene.add(sphere); // requires that your viewer has a Three.js scene
                                }
                                this.objects[obj.name] = sphere;
                            }
                        }
                        const mesh = this.objects[obj.name];
                        if (mesh) {
                            mesh.position.set(...obj.position);
                            mesh.quaternion.set(...obj.orientation);
                        }
                    });
                }
            } catch (err) {
                console.error("Error processing sim_state:", err);
            }
        };
        // ---------------------------------------------------

        this.websocket.onclose = () => {
            this.websocketConnected = false;
            this.updateStatus('Disconnected. Reconnecting...', 'error');
            setTimeout(() => this.connectWebSocket(), 3000);
        };

        this.websocket.onerror = (err) => {
            this.updateStatus('WebSocket Error. Is main.py running?', 'error');
        };
    }

    setupEventListeners() {
        // Disable 3D-specific controls that no longer do anything
        document.getElementById('showAxes').disabled = true;
        document.getElementById('wireframe').disabled = true;
        document.getElementById('showGrid').disabled = true;
        document.getElementById('autoRotate').disabled = true;

        // The MediaPipe controller will handle its own buttons now.
        // We just ensure we provide the updateJoint function it needs.
    }

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

    // --- NEW: Optional hook for visual joint updates ---
    updateJointVisual(jointName, value) {
        if (this.joints[jointName]) {
            this.joints[jointName].setJointValue(value);
        }
    }
    // ---------------------------------------------------

    updateStatus(message, type) {
        const statusDiv = document.getElementById('status');
        if (statusDiv) {
            statusDiv.textContent = message;
            statusDiv.className = `status-${type}`;
        }
    }
}

// Initialize the controller and connect it to the MediaPipe module
window.addEventListener('DOMContentLoaded', () => {
    const viewer = new HybridViewController();
    // The MediaPipe controller is now self-initializing but needs a reference to the viewer
    // to call `updateJoint`. We can pass the viewer instance to it.
    const webcamElement = document.getElementById('webcam');
    if (webcamElement) {
        new MediaPipeHandController(viewer, webcamElement);
    }
});*/