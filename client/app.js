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
}

// Initialize the application when the page loads
document.addEventListener('DOMContentLoaded', () => {
    window.remoteViewer = new RemoteViewer();
    console.log('[RemoteViewer] Application started');
});