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
        this.currentRLEnvironment = null;
        this.isRLRunning = false;
        
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
            // Scene creation controls
            scenePromptInput: document.getElementById('scene-prompt-input'),
            scenePresetDropdown: document.getElementById('scene-preset-dropdown'),
            generateSceneBtn: document.getElementById('generate-scene-btn'),
            loadSceneBtn: document.getElementById('load-scene-btn'),
            
            
            // RL Environment tab system
            rlTabButtons: document.querySelectorAll('#rl-editor-container .tab-button'),
            rlTabPanes: document.querySelectorAll('#rl-editor-container .tab-pane'),
            rlXmlTab: document.getElementById('rl-xml-tab'),
            rlPythonTab: document.getElementById('rl-python-tab'),
            rlXmlEditor: document.getElementById('rl-xml-editor'),
            rlPythonEditor: document.getElementById('rl-python-editor'),
            
            // RL Environment editor controls
            toggleRlEditorBtn: document.getElementById('toggle-rl-editor'),
            rlEditorContainer: document.getElementById('rl-editor-container'),
            rlValidationStatus: document.getElementById('rl-validation-status'),
            
            // RL Environment controls
            rlPromptInput: document.getElementById('rl-prompt-input'),
            rlPresetDropdown: document.getElementById('rl-preset-dropdown'),
            generateRlEnvBtn: document.getElementById('generate-rl-env-btn'),
            loadRlEnvBtn: document.getElementById('load-rl-env-btn'),
            runRandomActionsBtn: document.getElementById('run-random-actions-btn'),
            stopRlEnvBtn: document.getElementById('stop-rl-env-btn'),
            toggleGuidelinesBtn: document.getElementById('toggle-guidelines'),
            guidelinesContainer: document.getElementById('guidelines-container'),
            
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
        this.elements.rlEditor.addEventListener('input', () => this.handleRlEditorChange())
        // Scene creation controls
        this.elements.scenePresetDropdown.addEventListener('change', (e) => {
            if (e.target.value) {
                this.elements.scenePromptInput.value = e.target.value;
                this.handlePromptChange();
            }
        });
        
        this.elements.scenePromptInput.addEventListener('input', () => {
            this.handlePromptChange();
        });
        
        this.elements.generateSceneBtn.addEventListener('click', () => this.generateScene());
        this.elements.loadSceneBtn.addEventListener('click', () => this.loadScene());
        this.elements.toggleSceneXmlBtn.addEventListener('click', () => this.toggleSceneXmlEditor());
        
        // RL Environment tab system
        this.elements.rlTabButtons.forEach(btn => {
            btn.addEventListener('click', (e) => this.switchRLTab(e.target.dataset.tab));
        });
        
        // RL Environment editor controls
        this.elements.toggleRlEditorBtn.addEventListener('click', () => this.toggleRLEditor());
        
        // RL Environment controls
        this.elements.rlPresetDropdown.addEventListener('change', (e) => {
            if (e.target.value) {
                this.handleRLPresetChange(e.target.value);
            }
        });
        
        this.elements.rlPromptInput.addEventListener('input', () => {
            this.handleRLPromptChange();
        });
        
        this.elements.generateRlEnvBtn.addEventListener('click', () => this.generateRLEnvironment());
        this.elements.loadRlEnvBtn.addEventListener('click', () => this.loadRLEnvironment());
        this.elements.runRandomActionsBtn.addEventListener('click', () => this.runRandomActions());
        this.elements.stopRlEnvBtn.addEventListener('click', () => this.stopRLEnvironment());
        this.elements.toggleGuidelinesBtn.addEventListener('click', () => this.toggleGuidelines());
        
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
        this.setSceneValidationStatus('checking', 'Generating scene XML...');
        
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
            
            // Update the Scene XML editor
            this.elements.sceneXmlEditor.value = xml;
            this.currentSceneXML = xml;
            
            // Validate the XML
            await this.validateSceneXML(xml);
            
            // Enable load button
            this.elements.loadSceneBtn.disabled = false;
            
            // Show Scene XML editor if collapsed
            if (this.elements.sceneXmlEditorContainer.classList.contains('collapsed')) {
                this.toggleSceneXmlEditor();
            }
            
            this.logEvent('Scene', `Generated ${sceneType} scene from prompt: "${prompt}"`);
            
        } catch (error) {
            this.setSceneValidationStatus('invalid', `Error: ${error.message}`);
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
     * Toggle Scene XML editor visibility
     */
    toggleSceneXmlEditor() {
        const container = this.elements.sceneXmlEditorContainer;
        const btn = this.elements.toggleSceneXmlBtn;
        
        if (container.classList.contains('collapsed')) {
            container.classList.remove('collapsed');
            btn.textContent = 'Hide XML';
        } else {
            container.classList.add('collapsed');
            btn.textContent = 'Show XML';
        }
    }
    
    /**
     * Toggle RL Editor visibility
     */
    toggleRLEditor() {
        const container = this.elements.rlEditorContainer;
        const btn = this.elements.toggleRlEditorBtn;
        
        if (container.classList.contains('collapsed')) {
            container.classList.remove('collapsed');
            btn.textContent = 'Hide RL Code';
        } else {
            container.classList.add('collapsed');
            btn.textContent = 'Show RL Code';
        }
    }
    
    /**
     * Switch between RL XML and Python tabs
     */
    switchRLTab(tabName) {
        // Update tab buttons
        this.elements.rlTabButtons.forEach(btn => {
            btn.classList.toggle('active', btn.dataset.tab === tabName);
        });
        
        // Update tab panes
        this.elements.rlTabPanes.forEach(pane => {
            pane.classList.toggle('active', pane.id === `${tabName}-tab`);
        });
    }
    
    /**
     * Handle RL preset dropdown change
     */
    handleRLPresetChange(presetValue) {
        const presetPrompts = {
            'franka_reaching': 'Create a reaching task for Franka Panda to reach random targets with continuous actions',
            'cart_pole_balancing': 'Design a cart-pole balancing environment with discrete actions',
            'quadruped_walking': 'Build a quadruped walking environment with gait rewards',
            'simple_arm_reaching': 'Create a simple 2-DOF arm reaching task with sparse rewards'
        };
        
        if (presetPrompts[presetValue]) {
            this.elements.rlPromptInput.value = presetPrompts[presetValue];
            this.handleRLPromptChange();
        }
    }
    
    /**
     * Handle RL prompt input change
     */
    handleRLPromptChange() {
        const prompt = this.elements.rlPromptInput.value.trim();
        this.elements.generateRlEnvBtn.disabled = !prompt;
    }
    
    /**
     * Generate RL environment from prompt
     */
    async generateRLEnvironment() {
        const prompt = this.elements.rlPromptInput.value.trim();
        if (!prompt) return;
        
        // Check if we have a scene XML to base the RL environment on
        if (!this.currentSceneXML) {
            this.logEvent('Please generate a scene first before creating an RL environment', 'error');
            return;
        }
        
        this.elements.generateRlEnvBtn.disabled = true;
        this.elements.generateRlEnvBtn.textContent = 'Generating...';
        
        try {
            this.logEvent('Generating RL environment...', 'info');
            
            // Parse the prompt to determine environment type
            const envConfig = this.parseRLPrompt(prompt);
            
            // Generate the RL environment XML based on the scene XML
            const rlXml = this.generateRLEnvironmentXML(envConfig, this.currentSceneXML);
            this.elements.rlXmlEditor.value = rlXml;
            
            // Generate the Python gymnasium environment code
            const pythonCode = this.generateRLEnvironmentPython(envConfig);
            this.elements.rlPythonEditor.value = pythonCode;
            this.currentRLEnvironment = { config: envConfig, python: pythonCode, xml: rlXml };
            
            // Update UI
            this.elements.loadRlEnvBtn.disabled = false;
            this.elements.runRandomActionsBtn.disabled = false;
            
            // Switch to Python tab to show the generated code
            this.switchRLTab('rl-python');
            
            // Show the RL editor if not already visible
            if (this.elements.rlEditorContainer.classList.contains('collapsed')) {
                this.toggleRLEditor();
            }
            
            this.logEvent(`RL environment generated: ${envConfig.task_type} with ${envConfig.robot_type}`, 'success');
            
        } catch (error) {
            this.logEvent(`Failed to generate RL environment: ${error.message}`, 'error');
        } finally {
            this.elements.generateRlEnvBtn.disabled = false;
            this.elements.generateRlEnvBtn.textContent = 'Generate RL Env';
        }
    }
    
    /**
     * Parse RL prompt to extract environment configuration
     */
    parseRLPrompt(prompt) {
        const config = {
            task_type: 'reaching',
            robot_type: 'simple_arm',
            action_space_type: 'continuous',
            max_episode_steps: 1000,
            reward_scale: 1.0
        };
        
        const promptLower = prompt.toLowerCase();
        
        // Detect task type
        if (promptLower.includes('reach')) config.task_type = 'reaching';
        else if (promptLower.includes('balanc')) config.task_type = 'balancing';
        else if (promptLower.includes('walk')) config.task_type = 'walking';
        else if (promptLower.includes('manipulat')) config.task_type = 'manipulation';
        
        // Detect robot type
        if (promptLower.includes('franka') || promptLower.includes('panda')) config.robot_type = 'franka_panda';
        else if (promptLower.includes('cart') && promptLower.includes('pole')) config.robot_type = 'cart_pole';
        else if (promptLower.includes('quadruped')) config.robot_type = 'quadruped';
        else if (promptLower.includes('simple') && promptLower.includes('arm')) config.robot_type = 'simple_arm';
        
        // Detect action space type
        if (promptLower.includes('discrete')) config.action_space_type = 'discrete';
        else if (promptLower.includes('continuous')) config.action_space_type = 'continuous';
        
        return config;
    }
    
    /**
     * Generate XML for RL environment based on scene XML
     */
    generateRLEnvironmentXML(config, baseSceneXML) {
        // If we have a base scene XML, modify it for RL
        if (baseSceneXML) {
            // Add RL-specific elements to the base scene
            let rlXml = baseSceneXML;
            
            // Add targets for reaching tasks
            if (config.task_type === 'reaching') {
                // Look for closing </worldbody> tag and add target before it
                if (rlXml.includes('</worldbody>')) {
                    const targetXml = `
        <!-- RL Target -->
        <body name="target" pos="0.5 0.0 0.5">
            <geom name="target_geom" type="sphere" size="0.05" rgba="0 1 0 0.7"/>
        </body>`;
                    rlXml = rlXml.replace('</worldbody>', targetXml + '\n    </worldbody>');
                }
            }
            
            // Add actuators section for control
            if (!rlXml.includes('<actuator>')) {
                const actuatorXml = `
    <actuator>
        <!-- RL Actuators will be dynamically added based on joints -->
    </actuator>`;
                rlXml = rlXml.replace('</mujoco>', actuatorXml + '\n</mujoco>');
            }
            
            return rlXml;
        }
        
        // Fallback to predefined templates if no base scene
        const xmlTemplates = {
            franka_reaching: `<mujoco model="franka_reaching">
    <option timestep="0.002"/>
    <worldbody>
        <body name="base" pos="0 0 0">
            <geom name="base_geom" type="cylinder" size="0.1 0.1" rgba="0.5 0.5 0.5 1"/>
            <body name="link1" pos="0 0 0.1">
                <joint name="joint1" type="hinge" axis="0 0 1"/>
                <geom name="link1_geom" type="capsule" size="0.05 0.2" rgba="0.8 0.2 0.2 1"/>
                <body name="link2" pos="0 0 0.2">
                    <joint name="joint2" type="hinge" axis="0 1 0"/>
                    <geom name="link2_geom" type="capsule" size="0.04 0.15" rgba="0.2 0.8 0.2 1"/>
                    <body name="end_effector" pos="0 0 0.15">
                        <geom name="ee_geom" type="sphere" size="0.03" rgba="1 0 0 1"/>
                    </body>
                </body>
            </body>
        </body>
        <body name="target" pos="0.5 0.0 0.5">
            <geom name="target_geom" type="sphere" size="0.05" rgba="0 1 0 0.7"/>
        </body>
    </worldbody>
</mujoco>`,
            cart_pole: `<mujoco model="cartpole">
    <option timestep="0.002"/>
    <worldbody>
        <body name="cart" pos="0 0 0.1">
            <joint name="slider" type="slide" axis="1 0 0" range="-2 2"/>
            <geom name="cart_geom" type="box" size="0.1 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
            <body name="pole" pos="0 0 0.1">
                <joint name="hinge" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
                <geom name="pole_geom" type="capsule" size="0.02 0.5" rgba="0.2 0.8 0.2 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>`,
            quadruped: `<mujoco model="quadruped">
    <option timestep="0.002"/>
    <worldbody>
        <body name="torso" pos="0 0 0.3">
            <geom name="torso_geom" type="box" size="0.2 0.1 0.05" rgba="0.8 0.2 0.2 1"/>
            <body name="leg1" pos="0.15 0.08 -0.05">
                <joint name="hip1" type="hinge" axis="0 1 0"/>
                <geom name="leg1_geom" type="capsule" size="0.02 0.1" rgba="0.2 0.8 0.2 1"/>
            </body>
            <body name="leg2" pos="0.15 -0.08 -0.05">
                <joint name="hip2" type="hinge" axis="0 1 0"/>
                <geom name="leg2_geom" type="capsule" size="0.02 0.1" rgba="0.2 0.8 0.2 1"/>
            </body>
            <body name="leg3" pos="-0.15 0.08 -0.05">
                <joint name="hip3" type="hinge" axis="0 1 0"/>
                <geom name="leg3_geom" type="capsule" size="0.02 0.1" rgba="0.2 0.8 0.2 1"/>
            </body>
            <body name="leg4" pos="-0.15 -0.08 -0.05">
                <joint name="hip4" type="hinge" axis="0 1 0"/>
                <geom name="leg4_geom" type="capsule" size="0.02 0.1" rgba="0.2 0.8 0.2 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>`,
            simple_arm: `<mujoco model="simple_arm">
    <option timestep="0.002"/>
    <worldbody>
        <body name="base" pos="0 0 0">
            <geom name="base_geom" type="cylinder" size="0.1 0.1" rgba="0.5 0.5 0.5 1"/>
            <body name="link1" pos="0 0 0.1">
                <joint name="joint1" type="hinge" axis="0 0 1"/>
                <geom name="link1_geom" type="capsule" size="0.05 0.2" rgba="0.8 0.2 0.2 1"/>
                <body name="link2" pos="0 0 0.2">
                    <joint name="joint2" type="hinge" axis="0 1 0"/>
                    <geom name="link2_geom" type="capsule" size="0.04 0.15" rgba="0.2 0.8 0.2 1"/>
                    <body name="end_effector" pos="0 0 0.15">
                        <geom name="ee_geom" type="sphere" size="0.03" rgba="1 0 0 1"/>
                    </body>
                </body>
            </body>
        </body>
        <body name="target" pos="0.3 0.0 0.3">
            <geom name="target_geom" type="sphere" size="0.05" rgba="0 1 0 0.7"/>
        </body>
    </worldbody>
</mujoco>`
        };
        
        if (config.robot_type === 'franka_panda') return xmlTemplates.franka_reaching;
        if (config.robot_type === 'cart_pole') return xmlTemplates.cart_pole;
        if (config.robot_type === 'quadruped') return xmlTemplates.quadruped;
        return xmlTemplates.simple_arm;
    }
    
    /**
     * Generate Python gymnasium environment code
     */
    generateRLEnvironmentPython(config) {
        return `#!/usr/bin/env python3
"""
Auto-generated RL Environment for ${config.task_type} task with ${config.robot_type}
Generated by MuJoCo MCP Remote Viewer
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import time
from mujoco_mcp.rl_integration import MuJoCoRLEnvironment, RLConfig

def create_environment():
    """Create and return the RL environment"""
    config = RLConfig(
        robot_type="${config.robot_type}",
        task_type="${config.task_type}",
        max_episode_steps=${config.max_episode_steps},
        action_space_type="${config.action_space_type}",
        reward_scale=${config.reward_scale}
    )
    return MuJoCoRLEnvironment(config)

def run_random_actions(env, num_steps=1000):
    """Run the environment with random actions"""
    print(f"🤖 Running ${config.task_type} task with random actions...")
    print(f"📊 Environment: ${config.robot_type}")
    print(f"🎮 Action space: ${config.action_space_type}")
    print("=" * 50)
    
    obs, info = env.reset()
    total_reward = 0
    
    for step in range(num_steps):
        # Sample random action
        action = env.action_space.sample()
        
        # Execute action
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        
        # Print progress every 100 steps
        if step % 100 == 0:
            print(f"Step {step:4d}: reward={reward:.3f}, total={total_reward:.3f}")
        
        # Reset if episode ends
        if terminated or truncated:
            print(f"Episode finished at step {step}")
            obs, info = env.reset()
            total_reward = 0
        
        # Small delay for visualization
        time.sleep(0.01)
    
    print(f"✅ Completed {num_steps} steps")

if __name__ == "__main__":
    # Create environment
    env = create_environment()
    
    print(f"Environment created successfully!")
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    
    try:
        # Run with random actions
        run_random_actions(env, num_steps=1000)
    finally:
        env.close()
`;
    }
    
    /**
     * Load RL environment to viewer
     */
    async loadRLEnvironment() {
        if (!this.currentRLEnvironment || !this.currentRLEnvironment.xml) return;
        
        this.elements.loadRlEnvBtn.disabled = true;
        this.elements.loadRlEnvBtn.textContent = 'Loading...';
        
        try {
            // Load the RL XML to the viewer
            const success = await this.sendCommand('load_model', {
                model_xml: this.currentRLEnvironment.xml
            });
            
            if (success) {
                this.logEvent('RL environment loaded to viewer', 'success');
            } else {
                throw new Error('Failed to load RL environment to viewer');
            }
        } catch (error) {
            this.logEvent(`Failed to load RL environment: ${error.message}`, 'error');
        } finally {
            this.elements.loadRlEnvBtn.disabled = false;
            this.elements.loadRlEnvBtn.textContent = 'Load RL Env';
        }
    }
    
    /**
     * Run RL environment with random actions
     */
    async runRandomActions() {
        if (!this.currentRLEnvironment || this.isRLRunning) return;
        
        this.isRLRunning = true;
        this.elements.runRandomActionsBtn.disabled = true;
        this.elements.stopRlEnvBtn.disabled = false;
        this.elements.runRandomActionsBtn.textContent = '⏳ Running...';
        
        try {
            this.logEvent('Starting RL environment with random actions...', 'info');
            
            // Send command to backend to start RL environment
            const result = await this.sendRLCommand('start_random_actions', {
                config: this.currentRLEnvironment.config,
                xml: this.currentRLEnvironment.xml
            });
            
            if (result.success) {
                this.logEvent('RL environment running with random actions', 'success');
                this.startRLMonitoring();
            } else {
                throw new Error(result.error || 'Failed to start RL environment');
            }
            
        } catch (error) {
            this.logEvent(`Failed to run RL environment: ${error.message}`, 'error');
            this.stopRLEnvironment();
        }
    }
    
    /**
     * Stop RL environment
     */
    async stopRLEnvironment() {
        if (!this.isRLRunning) return;
        
        try {
            await this.sendRLCommand('stop_rl_environment');
            this.logEvent('RL environment stopped', 'info');
        } catch (error) {
            this.logEvent(`Error stopping RL environment: ${error.message}`, 'error');
        }
        
        this.isRLRunning = false;
        this.elements.runRandomActionsBtn.disabled = false;
        this.elements.stopRlEnvBtn.disabled = true;
        this.elements.runRandomActionsBtn.textContent = '▶ Run Random Actions';
    }
    
    /**
     * Start monitoring RL environment performance
     */
    startRLMonitoring() {
        // This would periodically check RL environment status
        // For now, just update the UI to show it's running
        this.logEvent('RL monitoring started', 'info');
    }
    
    /**
     * Send RL-specific command to backend
     */
    async sendRLCommand(command, data = {}) {
        // This would send commands to a backend RL service
        // For now, simulate the command
        return new Promise((resolve) => {
            setTimeout(() => {
                resolve({ success: true, data: {} });
            }, 1000);
        });
    }
    
    /**
     * Toggle guidelines visibility
     */
    toggleGuidelines() {
        const container = this.elements.guidelinesContainer;
        const btn = this.elements.toggleGuidelinesBtn;
        
        if (container.classList.contains('collapsed')) {
            container.classList.remove('collapsed');
            btn.textContent = 'Hide Guidelines';
        } else {
            container.classList.add('collapsed');
            btn.textContent = 'Show Guidelines';
        }
    }
    
    /**
     * Validate Scene XML content
     */
    async validateSceneXML(xml) {
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
            
            this.setSceneValidationStatus('valid', 'Valid MuJoCo XML');
            return true;
            
        } catch (error) {
            this.setSceneValidationStatus('invalid', error.message);
            return false;
        }
    }
    
    /**
     * Set scene validation status
     */
    setSceneValidationStatus(type, message) {
        const status = this.elements.sceneXmlValidationStatus;
        if (status) {
            status.className = `validation-status ${type}`;
            status.textContent = message;
        }
    }
    
    /**
     * Validate XML content (legacy method for compatibility)
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