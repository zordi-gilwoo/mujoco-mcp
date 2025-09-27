/**
 * MuJoCo Remote Viewer Client Application
 * Handles WebRTC connection, event capture, and UI management
 */

class RemoteViewer {
    constructor() {
        // WebRTC and signaling
        this.peerConnection = null;
        this.websocket = null;
        this.localStream = null;
        
        // State tracking
        this.isConnected = false;
        this.isConnecting = false;
        this.frameCount = 0;
        this.eventCount = 0;
        
        // Scene creation state
        this.currentSceneXML = null;
        
        // Mouse interaction state
        this.mouseState = {
            isDown: false,
            lastX: 0,
            lastY: 0,
            button: 0
        };
        
        // DOM elements
        this.elements = {};
        
        // Configuration
        this.config = {
            stunServers: [{ urls: 'stun:stun.l.google.com:19302' }]
        };
        
        this.init();
    }
    
    /**
     * Initialize the application
     */
    init() {
        this.setupDOM();
        this.setupEventListeners();
        this.updateUI();
        this.loadConfig();
        
        console.log('[RemoteViewer] Initialized');
    }
    
    /**
     * Setup DOM element references
     */
    setupDOM() {
        this.elements = {
            // Status elements
            connectionStatus: document.getElementById('connection-status'),
            frameCounter: document.getElementById('frame-counter'),
            eventCounter: document.getElementById('event-counter'),
            
            // Video elements
            remoteVideo: document.getElementById('remote-video'),
            videoOverlay: document.getElementById('video-overlay'),
            loadingSpinner: document.getElementById('loading-spinner'),
            videoContainer: document.querySelector('.video-container'),
            
            // Control buttons
            connectBtn: document.getElementById('connect-btn'),
            disconnectBtn: document.getElementById('disconnect-btn'),
            fullscreenBtn: document.getElementById('fullscreen-btn'),
            
            // Simulation controls
            startSimBtn: document.getElementById('start-sim-btn'),
            pauseSimBtn: document.getElementById('pause-sim-btn'),
            resetSimBtn: document.getElementById('reset-sim-btn'),
            
            // Free-style command interface (main input)
            freestyleCommandInput: document.getElementById('freestyle-command-input'),
            executeCommandBtn: document.getElementById('execute-command-btn'),
            suggestionBtns: document.querySelectorAll('.suggestion-btn'),
            commandResult: document.getElementById('command-result'),
            
            // XML Editor
            toggleXmlBtn: document.getElementById('toggle-xml-editor'),
            xmlEditorContainer: document.getElementById('xml-editor-container'),
            xmlEditor: document.getElementById('xml-editor'),
            xmlValidationStatus: document.getElementById('xml-validation-status'),
            saveXmlBtn: document.getElementById('save-xml-btn'),
            validateXmlBtn: document.getElementById('validate-xml-btn'),
            
            // RL Editor
            toggleRlBtn: document.getElementById('toggle-rl-editor'),
            rlEditorContainer: document.getElementById('rl-editor-container'),
            rlEditor: document.getElementById('rl-editor'),
            rlValidationStatus: document.getElementById('rl-validation-status'),
            saveRlBtn: document.getElementById('save-rl-btn'),
            runRlBtn: document.getElementById('run-rl-btn'),
            
            // Camera presets
            presetBtns: document.querySelectorAll('.preset-btn'),
            
            // Event log and stats
            eventLog: document.getElementById('event-log'),
            statConnected: document.getElementById('stat-connected'),
            statFrames: document.getElementById('stat-frames'),
            statEvents: document.getElementById('stat-events')
        };
    }
    
    /**
     * Setup event listeners
     */
    setupEventListeners() {
        // Connection controls
        this.elements.connectBtn.addEventListener('click', () => this.connect());
        this.elements.disconnectBtn.addEventListener('click', () => this.disconnect());
        this.elements.fullscreenBtn.addEventListener('click', () => this.toggleFullscreen());
        
        // Simulation controls
        this.elements.startSimBtn.addEventListener('click', () => this.sendCommand('start'));
        this.elements.pauseSimBtn.addEventListener('click', () => this.sendCommand('pause'));
        this.elements.resetSimBtn.addEventListener('click', () => this.sendCommand('reset'));
        
        // Editor controls
        this.elements.toggleXmlBtn.addEventListener('click', () => this.toggleXmlEditor());
        this.elements.toggleRlBtn.addEventListener('click', () => this.toggleRlEditor());
        this.elements.saveXmlBtn.addEventListener('click', () => this.saveXmlContent());
        this.elements.validateXmlBtn.addEventListener('click', () => this.validateXmlContent());
        this.elements.saveRlBtn.addEventListener('click', () => this.saveRlContent());
        this.elements.runRlBtn.addEventListener('click', () => this.runRlScript());
        
        // Editor content change listeners
        this.elements.xmlEditor.addEventListener('input', () => this.handleXmlEditorChange());
        this.elements.rlEditor.addEventListener('input', () => this.handleRlEditorChange());
        
        // Camera presets
        this.elements.presetBtns.forEach(btn => {
            btn.addEventListener('click', (e) => {
                const preset = e.target.dataset.preset;
                if (preset === 'reset') {
                    this.sendCommand('reset_camera');
                } else {
                    this.sendCommand('set_camera_preset', { preset });
                }
            });
        });
        
        // Free-style command interface
        this.elements.executeCommandBtn.addEventListener('click', () => this.executeFreestyleCommand());
        
        this.elements.freestyleCommandInput.addEventListener('keydown', (e) => {
            if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) {
                e.preventDefault();
                this.executeFreestyleCommand();
            }
        });
        
        this.elements.freestyleCommandInput.addEventListener('input', () => {
            this.handleFreestyleInputChange();
        });
        
        this.elements.suggestionBtns.forEach(btn => {
            btn.addEventListener('click', (e) => {
                const command = e.target.dataset.command;
                this.elements.freestyleCommandInput.value = command;
                this.handleFreestyleInputChange();
                this.executeFreestyleCommand();
            });
        });
        
        // Video interaction events
        this.setupVideoInteraction();
        
        // Keyboard events
        document.addEventListener('keydown', (e) => this.handleKeyDown(e));
        document.addEventListener('keyup', (e) => this.handleKeyUp(e));
        
        // Window events
        window.addEventListener('beforeunload', () => this.disconnect());
        
        // Fullscreen change
        document.addEventListener('fullscreenchange', () => this.handleFullscreenChange());
    }
    
    /**
     * Setup video container interaction
     */
    setupVideoInteraction() {
        const video = this.elements.videoContainer;
        
        // Mouse events
        video.addEventListener('mousedown', (e) => this.handleMouseDown(e));
        video.addEventListener('mousemove', (e) => this.handleMouseMove(e));
        video.addEventListener('mouseup', (e) => this.handleMouseUp(e));
        video.addEventListener('wheel', (e) => this.handleWheel(e));
        video.addEventListener('contextmenu', (e) => e.preventDefault()); // Prevent right-click menu
        
        // Touch events for mobile support
        video.addEventListener('touchstart', (e) => this.handleTouchStart(e));
        video.addEventListener('touchmove', (e) => this.handleTouchMove(e));
        video.addEventListener('touchend', (e) => this.handleTouchEnd(e));
    }
    
    /**
     * Load configuration from server
     */
    async loadConfig() {
        try {
            const response = await fetch('/api/config');
            const config = await response.json();
            console.log('[RemoteViewer] Loaded config:', config);
            
            // Update local config
            if (config.stun_server) {
                this.config.stunServers = [{ urls: config.stun_server }];
            }
        } catch (error) {
            console.warn('[RemoteViewer] Failed to load config:', error);
        }
    }
    
    /**
     * Connect to the remote viewer
     */
    async connect() {
        if (this.isConnecting || this.isConnected) return;
        
        this.isConnecting = true;
        this.updateUI();
        this.logEvent('Connection', 'Connecting to server...');
        
        try {
            await this.setupWebSocket();
            await this.setupWebRTC();
            
            console.log('[RemoteViewer] Connection initiated');
        } catch (error) {
            console.error('[RemoteViewer] Connection failed:', error);
            this.logEvent('Error', `Connection failed: ${error.message}`);
            this.disconnect();
        }
    }
    
    /**
     * Disconnect from the remote viewer
     */
    disconnect() {
        console.log('[RemoteViewer] Disconnecting...');
        
        this.isConnected = false;
        this.isConnecting = false;
        
        // Close WebRTC
        if (this.peerConnection) {
            this.peerConnection.close();
            this.peerConnection = null;
        }
        
        // Close WebSocket
        if (this.websocket) {
            this.websocket.close();
            this.websocket = null;
        }
        
        // Reset video
        this.elements.remoteVideo.srcObject = null;
        
        this.updateUI();
        this.logEvent('Connection', 'Disconnected');
    }
    
    /**
     * Setup WebSocket connection for signaling
     */
    async setupWebSocket() {
        return new Promise((resolve, reject) => {
            const wsUrl = `ws://${window.location.host}/ws/signaling`;
            this.websocket = new WebSocket(wsUrl);
            
            this.websocket.onopen = () => {
                console.log('[RemoteViewer] WebSocket connected');
                resolve();
            };
            
            this.websocket.onmessage = (event) => {
                this.handleSignalingMessage(JSON.parse(event.data));
            };
            
            this.websocket.onclose = () => {
                console.log('[RemoteViewer] WebSocket closed');
                if (this.isConnected) {
                    this.disconnect();
                }
            };
            
            this.websocket.onerror = (error) => {
                console.error('[RemoteViewer] WebSocket error:', error);
                reject(error);
            };
            
            // Timeout
            setTimeout(() => {
                if (this.websocket.readyState !== WebSocket.OPEN) {
                    reject(new Error('WebSocket connection timeout'));
                }
            }, 5000);
        });
    }
    
    /**
     * Setup WebRTC peer connection
     */
    async setupWebRTC() {
        this.peerConnection = new RTCPeerConnection({
            iceServers: this.config.stunServers
        });
        
        // Handle incoming stream
        this.peerConnection.ontrack = (event) => {
            console.log('[RemoteViewer] Received remote stream');
            this.elements.remoteVideo.srcObject = event.streams[0];
            this.isConnected = true;
            this.updateUI();
            this.logEvent('WebRTC', 'Video stream connected');
        };
        
        // Handle ICE candidates
        this.peerConnection.onicecandidate = (event) => {
            if (event.candidate) {
                this.sendSignalingMessage({
                    type: 'ice-candidate',
                    candidate: event.candidate.toJSON()
                });
            }
        };
        
        // Handle connection state changes
        this.peerConnection.onconnectionstatechange = () => {
            console.log('[RemoteViewer] Connection state:', this.peerConnection.connectionState);
            
            if (this.peerConnection.connectionState === 'connected') {
                this.isConnected = true;
                this.updateUI();
            } else if (this.peerConnection.connectionState === 'failed' || 
                       this.peerConnection.connectionState === 'closed') {
                this.disconnect();
            }
        };
        
        // Create offer
        const offer = await this.peerConnection.createOffer();
        await this.peerConnection.setLocalDescription(offer);
        
        // Send offer to server
        this.sendSignalingMessage({
            type: 'offer',
            sdp: offer.sdp
        });
    }
    
    /**
     * Handle signaling messages from server
     */
    async handleSignalingMessage(message) {
        try {
            switch (message.type) {
                case 'answer':
                    const answer = new RTCSessionDescription({
                        type: 'answer',
                        sdp: message.sdp
                    });
                    await this.peerConnection.setRemoteDescription(answer);
                    console.log('[RemoteViewer] Received answer from server');
                    break;
                    
                case 'ice-candidate':
                    if (message.candidate) {
                        await this.peerConnection.addIceCandidate(new RTCIceCandidate(message.candidate));
                    }
                    break;
                    
                default:
                    console.warn('[RemoteViewer] Unknown signaling message:', message);
            }
        } catch (error) {
            console.error('[RemoteViewer] Error handling signaling message:', error);
        }
    }
    
    /**
     * Send signaling message to server
     */
    sendSignalingMessage(message) {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify(message));
        }
    }
    
    /**
     * Send event to server
     */
    sendEvent(eventData) {
        if (!this.isConnected) return;
        
        this.sendSignalingMessage({
            type: 'event',
            data: eventData
        });
        
        this.eventCount++;
        this.updateUI();
    }
    
    /**
     * Send command to server
     */
    sendCommand(cmd, params = null) {
        const eventData = {
            type: 'command',
            cmd: cmd
        };
        
        if (params) {
            eventData.params = params;
        }
        
        this.sendEvent(eventData);
        this.logEvent('Command', `Sent: ${cmd}`);
    }
    
    /**
     * Handle mouse down events
     */
    handleMouseDown(e) {
        if (!this.isConnected) return;
        
        this.mouseState.isDown = true;
        this.mouseState.lastX = e.clientX;
        this.mouseState.lastY = e.clientY;
        this.mouseState.button = e.buttons;
        
        const rect = e.target.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        this.sendEvent({
            type: 'mouse_down',
            x: Math.round(x),
            y: Math.round(y),
            buttons: e.buttons
        });
        
        e.preventDefault();
    }
    
    /**
     * Handle mouse move events
     */
    handleMouseMove(e) {
        if (!this.isConnected) return;
        
        const rect = e.target.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        this.sendEvent({
            type: 'mouse_move',
            x: Math.round(x),
            y: Math.round(y),
            buttons: e.buttons
        });
        
        this.mouseState.lastX = e.clientX;
        this.mouseState.lastY = e.clientY;
    }
    
    /**
     * Handle mouse up events
     */
    handleMouseUp(e) {
        if (!this.isConnected) return;
        
        this.mouseState.isDown = false;
        
        const rect = e.target.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        this.sendEvent({
            type: 'mouse_up',
            x: Math.round(x),
            y: Math.round(y),
            buttons: e.buttons
        });
    }
    
    /**
     * Handle wheel/scroll events
     */
    handleWheel(e) {
        if (!this.isConnected) return;
        
        const rect = e.target.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        this.sendEvent({
            type: 'scroll',
            x: Math.round(x),
            y: Math.round(y),
            dx: e.deltaX,
            dy: e.deltaY
        });
        
        e.preventDefault();
    }
    
    /**
     * Handle key down events
     */
    handleKeyDown(e) {
        if (!this.isConnected) return;
        
        // Handle special keys
        if (e.code === 'Space') {
            this.sendCommand('pause');
            e.preventDefault();
            return;
        }
        
        this.sendEvent({
            type: 'key_down',
            code: e.code,
            key: e.key,
            alt: e.altKey,
            ctrl: e.ctrlKey,
            shift: e.shiftKey,
            meta: e.metaKey
        });
        
        // Prevent default for arrow keys and page up/down
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'PageUp', 'PageDown'].includes(e.code)) {
            e.preventDefault();
        }
    }
    
    /**
     * Handle key up events
     */
    handleKeyUp(e) {
        if (!this.isConnected) return;
        
        this.sendEvent({
            type: 'key_up',
            code: e.code,
            key: e.key,
            alt: e.altKey,
            ctrl: e.ctrlKey,
            shift: e.shiftKey,
            meta: e.metaKey
        });
    }
    
    /**
     * Handle touch events (basic mobile support)
     */
    handleTouchStart(e) {
        if (e.touches.length === 1) {
            const touch = e.touches[0];
            // Simulate mouse down
            this.handleMouseDown({
                clientX: touch.clientX,
                clientY: touch.clientY,
                buttons: 1,
                target: e.target,
                preventDefault: () => e.preventDefault()
            });
        }
    }
    
    handleTouchMove(e) {
        if (e.touches.length === 1) {
            const touch = e.touches[0];
            // Simulate mouse move
            this.handleMouseMove({
                clientX: touch.clientX,
                clientY: touch.clientY,
                buttons: 1,
                target: e.target
            });
        }
        e.preventDefault();
    }
    
    handleTouchEnd(e) {
        // Simulate mouse up
        this.handleMouseUp({
            clientX: this.mouseState.lastX,
            clientY: this.mouseState.lastY,
            buttons: 0,
            target: e.target
        });
    }
    
    /**
     * Toggle fullscreen mode
     */
    toggleFullscreen() {
        const container = this.elements.videoContainer;
        
        if (!document.fullscreenElement) {
            container.requestFullscreen().catch(err => {
                console.error('[RemoteViewer] Fullscreen error:', err);
            });
        } else {
            document.exitFullscreen();
        }
    }
    
    /**
     * Handle fullscreen changes
     */
    handleFullscreenChange() {
        const container = this.elements.videoContainer;
        const isFullscreen = document.fullscreenElement === container;
        
        container.classList.toggle('fullscreen', isFullscreen);
        this.elements.fullscreenBtn.textContent = isFullscreen ? 'Exit Fullscreen' : 'Fullscreen';
    }
    
    /**
     * Update UI based on current state
     */
    updateUI() {
        // Connection status
        const statusElement = this.elements.connectionStatus;
        if (this.isConnected) {
            statusElement.textContent = 'Connected';
            statusElement.className = 'status-connected';
        } else if (this.isConnecting) {
            statusElement.textContent = 'Connecting...';
            statusElement.className = 'status-connecting';
        } else {
            statusElement.textContent = 'Disconnected';
            statusElement.className = 'status-disconnected';
        }
        
        // Buttons
        this.elements.connectBtn.disabled = this.isConnecting || this.isConnected;
        this.elements.disconnectBtn.disabled = !this.isConnecting && !this.isConnected;
        
        // Simulation controls
        const simControlsDisabled = !this.isConnected;
        this.elements.startSimBtn.disabled = simControlsDisabled;
        this.elements.pauseSimBtn.disabled = simControlsDisabled;
        this.elements.resetSimBtn.disabled = simControlsDisabled;
        
        // Camera presets
        this.elements.presetBtns.forEach(btn => {
            btn.disabled = simControlsDisabled;
        });
        
        // Video overlay
        this.elements.videoOverlay.classList.toggle('hidden', this.isConnected);
        
        // Free-style command interface (always enabled - works independently of viewer connection)
        this.handleFreestyleInputChange();
        
        // Counters
        this.elements.frameCounter.textContent = `Frames: ${this.frameCount}`;
        this.elements.eventCounter.textContent = `Events: ${this.eventCount}`;
        
        // Stats
        this.elements.statConnected.textContent = this.isConnected ? 'Yes' : 'No';
        this.elements.statFrames.textContent = this.frameCount.toString();
        this.elements.statEvents.textContent = this.eventCount.toString();
    }
    
    /**
     * Log event to the event log
     */
    logEvent(category, message) {
        const log = this.elements.eventLog;
        const timestamp = new Date().toLocaleTimeString();
        
        const eventElement = document.createElement('p');
        eventElement.innerHTML = `<span style="color: #666">[${timestamp}]</span> <strong>${category}:</strong> ${message}`;
        eventElement.className = `event-${category.toLowerCase()}`;
        
        // Remove old events if too many
        while (log.children.length > 50) {
            log.removeChild(log.firstChild);
        }
        
        log.appendChild(eventElement);
        log.scrollTop = log.scrollHeight;
    }
    
    /**
     * Start periodic stats updates
     */
    startStatsUpdates() {
        setInterval(() => {
            if (this.isConnected) {
                // Update frame counter (this would be updated by video track events in a real implementation)
                // this.frameCount++;
                this.updateUI();
            }
        }, 1000);
    }
    
    /**
     * Handle prompt input changes
     */
    handlePromptChange() {
        const prompt = this.elements.scenePromptInput.value.trim();
        this.elements.generateSceneBtn.disabled = !prompt;
        
        if (prompt) {
            // Clear the dropdown selection if user is typing
            this.elements.scenePresetDropdown.value = '';
        }
    }
    
    /**
     * Generate scene XML from prompt
     */
    async generateScene() {
        const prompt = this.elements.scenePromptInput.value.trim();
        if (!prompt) return;
        
        this.elements.generateSceneBtn.disabled = true;
        this.elements.generateSceneBtn.textContent = 'Generating...';
        this.setValidationStatus('checking', 'Generating scene XML...');
        
        try {
            // Map common prompts to scene types
            const sceneMapping = {
                'create a pendulum simulation': 'pendulum',
                'create a double pendulum': 'double_pendulum', 
                'create a cart pole simulation': 'cart_pole'
            };
            
            const normalizedPrompt = prompt.toLowerCase();
            let sceneType = null;
            
            // Find matching scene type
            for (const [key, value] of Object.entries(sceneMapping)) {
                if (normalizedPrompt.includes(key) || normalizedPrompt.includes(key.replace('create a ', ''))) {
                    sceneType = value;
                    break;
                }
            }
            
            if (!sceneType) {
                throw new Error('Scene type not recognized. Try: "Create a pendulum simulation", "Create a double pendulum", or "Create a cart pole simulation"');
            }
            
            // Generate the XML based on scene type
            const xml = this.generateSceneXML(sceneType);
            
            // Update the XML editor
            this.elements.xmlEditor.value = xml;
            this.currentSceneXML = xml;
            
            // Validate the XML
            await this.validateXML(xml);
            
            // Enable load button
            this.elements.loadSceneBtn.disabled = false;
            
            // Show XML editor if collapsed
            if (this.elements.xmlEditorContainer.classList.contains('collapsed')) {
                this.toggleXmlEditor();
            }
            
            this.logEvent('Scene', `Generated ${sceneType} scene from prompt: "${prompt}"`);
            
        } catch (error) {
            this.setValidationStatus('invalid', `Error: ${error.message}`);
            this.logEvent('Error', `Scene generation failed: ${error.message}`);
        } finally {
            this.elements.generateSceneBtn.disabled = false;
            this.elements.generateSceneBtn.textContent = 'Generate Scene';
        }
    }
    
    /**
     * Generate MuJoCo XML for scene type
     */
    generateSceneXML(sceneType) {
        const sceneTemplates = {
            'pendulum': `<mujoco>
    <worldbody>
        <body name="pole" pos="0 0 1">
            <joint name="hinge" type="hinge" axis="1 0 0"/>
            <geom name="pole" type="capsule" size="0.02 0.6" rgba="0.8 0.2 0.2 1"/>
            <body name="mass" pos="0 0 -0.6">
                <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.8 0.2 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>`,
            'double_pendulum': `<mujoco>
    <worldbody>
        <body name="pole1" pos="0 0 1">
            <joint name="hinge1" type="hinge" axis="1 0 0"/>
            <geom name="pole1" type="capsule" size="0.02 0.4" rgba="0.8 0.2 0.2 1"/>
            <body name="pole2" pos="0 0 -0.4">
                <joint name="hinge2" type="hinge" axis="1 0 0"/>
                <geom name="pole2" type="capsule" size="0.02 0.4" rgba="0.2 0.8 0.2 1"/>
                <body name="mass" pos="0 0 -0.4">
                    <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
                </body>
            </body>
        </body>
    </worldbody>
</mujoco>`,
            'cart_pole': `<mujoco>
    <worldbody>
        <body name="cart" pos="0 0 0.1">
            <joint name="slider" type="slide" axis="1 0 0"/>
            <geom name="cart" type="box" size="0.1 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
            <body name="pole" pos="0 0 0.1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom name="pole" type="capsule" size="0.02 0.5" rgba="0.2 0.8 0.2 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>`
        };
        
        return sceneTemplates[sceneType] || '';
    }
    
    /**
     * Load scene into viewer
     */
    async loadScene() {
        if (!this.currentSceneXML) return;
        
        this.elements.loadSceneBtn.disabled = true;
        this.elements.loadSceneBtn.textContent = 'Loading...';
        
        try {
            // Send scene to server via API
            const response = await fetch('/api/scene/load', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    xml: this.currentSceneXML
                })
            });
            
            const result = await response.json();
            
            if (response.ok && result.success) {
                this.logEvent('Scene', 'Scene loaded into viewer successfully');
                
                // Also send through WebRTC for real-time updates if connected
                if (this.isConnected) {
                    this.sendEvent({
                        type: 'load_scene',
                        xml: this.currentSceneXML
                    });
                }
            } else {
                throw new Error(result.error || 'Failed to load scene');
            }
            
        } catch (error) {
            this.logEvent('Error', `Failed to load scene: ${error.message}`);
        } finally {
            this.elements.loadSceneBtn.disabled = false;
            this.elements.loadSceneBtn.textContent = 'Load Scene';
        }
    }
    
    /**
     * Toggle XML editor visibility
     */
    toggleXmlEditor() {
        const container = this.elements.xmlEditorContainer;
        const btn = this.elements.toggleXmlBtn;
        
        if (container.classList.contains('collapsed')) {
            container.classList.remove('collapsed');
            btn.textContent = 'Hide XML';
        } else {
            container.classList.add('collapsed');
            btn.textContent = 'Show XML';
        }
    }
    
    /**
     * Validate XML content
     */
    async validateXML(xml) {
        try {
            // Basic XML structure validation
            const parser = new DOMParser();
            const doc = parser.parseFromString(xml, 'text/xml');
            
            // Check for parsing errors
            const parseError = doc.querySelector('parsererror');
            if (parseError) {
                throw new Error('Invalid XML structure');
            }
            
            // Check if it's a MuJoCo XML (has mujoco root element)
            const mujocoRoot = doc.querySelector('mujoco');
            if (!mujocoRoot) {
                throw new Error('Not a valid MuJoCo XML (missing <mujoco> root element)');
            }
            
            // Check for required worldbody
            const worldbody = doc.querySelector('worldbody');
            if (!worldbody) {
                throw new Error('Missing <worldbody> element');
            }
            
            this.setValidationStatus('valid', 'Valid MuJoCo XML');
            return true;
            
        } catch (error) {
            this.setValidationStatus('invalid', `Validation error: ${error.message}`);
            return false;
        }
    }
    
    /**
     * Set validation status display
     */
    setValidationStatus(status, message) {
        const statusElement = this.elements.xmlValidationStatus;
        statusElement.className = `validation-status ${status}`;
        statusElement.textContent = message;
    }
    
    /**
     * Handle free-style command input changes
     */
    handleFreestyleInputChange() {
        const command = this.elements.freestyleCommandInput.value.trim();
        this.elements.executeCommandBtn.disabled = !command;
    }
    
    /**
     * Execute free-style natural language command
     */
    async executeFreestyleCommand() {
        const command = this.elements.freestyleCommandInput.value.trim();
        if (!command) return;
        
        this.elements.executeCommandBtn.disabled = true;
        this.elements.executeCommandBtn.classList.add('executing');
        this.elements.executeCommandBtn.textContent = 'Executing...';
        
        this.showCommandResult('loading', 'Executing command...');
        this.logEvent('Command', `Executing: "${command}"`);
        
        try {
            // Send command to MCP server
            const response = await fetch('/api/execute-command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    command: command
                })
            });
            
            const result = await response.json();
            
            if (response.ok && result.success) {
                this.showCommandResult('success', result.result || result.message);
                this.logEvent('Command', 'Command executed successfully');
                
                // Check if scene was updated and refresh if connected
                if (this.isConnected && (result.actions_taken?.includes('created_scene') || 
                    result.actions_taken?.includes('created_menagerie_scene'))) {
                    this.sendEvent({
                        type: 'refresh_scene'
                    });
                }
            } else {
                throw new Error(result.error || result.message || 'Command execution failed');
            }
            
        } catch (error) {
            this.showCommandResult('error', `Error: ${error.message}`);
            this.logEvent('Error', `Command failed: ${error.message}`);
        } finally {
            this.elements.executeCommandBtn.disabled = false;
            this.elements.executeCommandBtn.classList.remove('executing');
            this.elements.executeCommandBtn.textContent = 'Execute Command';
            this.handleFreestyleInputChange(); // Re-enable based on input content
        }
    }
    
    /**
     * Show command execution result
     */
    showCommandResult(type, message) {
        const resultElement = this.elements.commandResult;
        resultElement.className = `command-result ${type}`;
        resultElement.textContent = message;
        resultElement.style.display = 'block';
        
        // Auto-hide after success (but keep errors/loading visible)
        if (type === 'success') {
            setTimeout(() => {
                if (resultElement.classList.contains('success')) {
                    resultElement.style.display = 'none';
                }
            }, 5000);
        }
    }
    
    /**
     * Toggle RL editor visibility
     */
    toggleRlEditor() {
        const container = this.elements.rlEditorContainer;
        const btn = this.elements.toggleRlBtn;
        
        if (container.classList.contains('collapsed')) {
            container.classList.remove('collapsed');
            btn.textContent = 'Hide RL Editor';
        } else {
            container.classList.add('collapsed');
            btn.textContent = 'Show RL Editor';
        }
    }
    
    /**
     * Handle XML editor content changes
     */
    handleXmlEditorChange() {
        const hasContent = this.elements.xmlEditor.value.trim().length > 0;
        this.elements.saveXmlBtn.disabled = !hasContent;
        
        // Clear validation status when content changes
        this.setEditorValidationStatus('xml', '', '');
    }
    
    /**
     * Handle RL editor content changes
     */
    handleRlEditorChange() {
        const hasContent = this.elements.rlEditor.value.trim().length > 0;
        this.elements.saveRlBtn.disabled = !hasContent;
        this.elements.runRlBtn.disabled = !hasContent;
        
        // Clear validation status when content changes
        this.setEditorValidationStatus('rl', '', '');
    }
    
    /**
     * Save XML content
     */
    async saveXmlContent() {
        const xml = this.elements.xmlEditor.value.trim();
        if (!xml) return;
        
        this.elements.saveXmlBtn.disabled = true;
        this.elements.saveXmlBtn.textContent = 'Saving...';
        
        try {
            // Validate first
            const isValid = await this.validateXmlContent();
            if (!isValid) {
                throw new Error('XML validation failed');
            }
            
            // Here you could send the XML to the server or save locally
            this.currentSceneXML = xml;
            this.setEditorValidationStatus('xml', 'valid', 'XML saved successfully');
            this.logEvent('XML', 'Scene XML saved successfully');
            
        } catch (error) {
            this.setEditorValidationStatus('xml', 'invalid', `Save failed: ${error.message}`);
            this.logEvent('Error', `XML save failed: ${error.message}`);
        } finally {
            this.elements.saveXmlBtn.disabled = false;
            this.elements.saveXmlBtn.textContent = 'Save XML';
        }
    }
    
    /**
     * Validate XML content
     */
    async validateXmlContent() {
        const xml = this.elements.xmlEditor.value.trim();
        if (!xml) {
            this.setEditorValidationStatus('xml', 'invalid', 'No XML content to validate');
            return false;
        }
        
        this.setEditorValidationStatus('xml', 'checking', 'Validating XML...');
        
        try {
            // Basic XML structure validation
            const parser = new DOMParser();
            const doc = parser.parseFromString(xml, 'text/xml');
            
            // Check for parsing errors
            const parseError = doc.querySelector('parsererror');
            if (parseError) {
                throw new Error('Invalid XML structure');
            }
            
            // Check if it's a MuJoCo XML (has mujoco root element)
            const mujocoRoot = doc.querySelector('mujoco');
            if (!mujocoRoot) {
                throw new Error('Not a valid MuJoCo XML (missing <mujoco> root element)');
            }
            
            // Check for required worldbody
            const worldbody = doc.querySelector('worldbody');
            if (!worldbody) {
                throw new Error('Missing <worldbody> element');
            }
            
            this.setEditorValidationStatus('xml', 'valid', 'Valid MuJoCo XML');
            return true;
            
        } catch (error) {
            this.setEditorValidationStatus('xml', 'invalid', `Validation error: ${error.message}`);
            return false;
        }
    }
    
    /**
     * Save RL script content
     */
    async saveRlContent() {
        const script = this.elements.rlEditor.value.trim();
        if (!script) return;
        
        this.elements.saveRlBtn.disabled = true;
        this.elements.saveRlBtn.textContent = 'Saving...';
        
        try {
            // Here you could send the script to the server or save locally
            this.currentRlScript = script;
            this.setEditorValidationStatus('rl', 'valid', 'RL script saved successfully');
            this.logEvent('RL', 'RL script saved successfully');
            
        } catch (error) {
            this.setEditorValidationStatus('rl', 'invalid', `Save failed: ${error.message}`);
            this.logEvent('Error', `RL script save failed: ${error.message}`);
        } finally {
            this.elements.saveRlBtn.disabled = false;
            this.elements.saveRlBtn.textContent = 'Save Script';
        }
    }
    
    /**
     * Run RL script
     */
    async runRlScript() {
        const script = this.elements.rlEditor.value.trim();
        if (!script) return;
        
        this.elements.runRlBtn.disabled = true;
        this.elements.runRlBtn.textContent = 'Running...';
        
        try {
            // Send RL script execution command via the freestyle command interface
            const response = await fetch('/api/execute-command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    command: `Execute RL script: ${script}`
                })
            });
            
            const result = await response.json();
            
            if (response.ok && result.success) {
                this.setEditorValidationStatus('rl', 'valid', 'RL script executed successfully');
                this.logEvent('RL', 'RL script executed successfully');
            } else {
                throw new Error(result.error || 'RL script execution failed');
            }
            
        } catch (error) {
            this.setEditorValidationStatus('rl', 'invalid', `Execution failed: ${error.message}`);
            this.logEvent('Error', `RL script execution failed: ${error.message}`);
        } finally {
            this.elements.runRlBtn.disabled = false;
            this.elements.runRlBtn.textContent = 'Run Script';
        }
    }
    
    /**
     * Set validation status for editors (separate from the legacy method)
     */
    setEditorValidationStatus(editorType, status, message) {
        let statusElement;
        
        if (editorType === 'xml') {
            statusElement = this.elements.xmlValidationStatus;
        } else if (editorType === 'rl') {
            statusElement = this.elements.rlValidationStatus;
        }
        
        if (statusElement) {
            statusElement.className = `validation-status ${status}`;
            statusElement.textContent = message;
        }
    }
}

// Initialize the application when the page loads
document.addEventListener('DOMContentLoaded', () => {
    window.remoteViewer = new RemoteViewer();
    console.log('[RemoteViewer] Application started');
});