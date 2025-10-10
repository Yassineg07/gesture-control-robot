const WebSocket = require('ws');
const http = require('http');
const sharp = require('sharp');

// Image processing configuration - optimized for speed and quality
const IMAGE_PROCESSING = {
    enabled: true,
    median: 2,              // Light median filter for noise reduction
    brightness: 1.03,       // Minimal brightness boost
    saturation: 1.05,       // Minimal color enhancement
    jpegQuality: 75         // Balanced quality/speed
};

// Create HTTP server for web page
const server = http.createServer((req, res) => {
    if (req.url === '/') {
        const html = `<!DOCTYPE html>
        <html>
        <head>
            <title>ESP32-CAM Live Stream</title>
            <style>
                * { margin: 0; padding: 0; box-sizing: border-box; }
                body {
                    background: linear-gradient(135deg, #0a0a0a 0%, #1a1a1a 50%, #0f0f0f 100%);
                    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;
                    color: #fff;
                    min-height: 100vh;
                    padding: 10px;
                }
                .container {
                    max-width: 1400px;
                    width: 100%;
                    margin: 0 auto;
                }
                header {
                    text-align: center;
                    margin-bottom: 10px;
                }
                h1 {
                    font-size: 26px;
                    font-weight: 600;
                    background: linear-gradient(135deg, #2563eb 0%, #8b0000 100%);
                    -webkit-background-clip: text;
                    -webkit-text-fill-color: transparent;
                    background-clip: text;
                }
                .layout {
                    display: grid;
                    grid-template-columns: 1fr 320px;
                    gap: 20px;
                    width: 100%;
                }
                @media (max-width: 1024px) {
                    .layout {
                        grid-template-columns: 1fr;
                    }
                }
                .stream-section {
                    background: rgba(30, 30, 30, 0.8);
                    backdrop-filter: blur(10px);
                    border-radius: 12px;
                    padding: 15px;
                    box-shadow: 0 20px 60px rgba(0, 0, 0, 0.5);
                    border: 1px solid rgba(100, 100, 100, 0.2);
                }
                .stream-container {
                    position: relative;
                    background: #000;
                    border-radius: 12px;
                    overflow: hidden;
                    aspect-ratio: 4/3;
                    max-height: 750px;
                }
                #stream {
                    width: 100%;
                    height: 100%;
                    object-fit: contain;
                    display: none;
                }
                .stream-placeholder {
                    position: absolute;
                    inset: 0;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    background: rgba(0, 0, 0, 0.5);
                    backdrop-filter: blur(5px);
                }
                .spinner {
                    width: 50px;
                    height: 50px;
                    border: 4px solid rgba(100, 100, 100, 0.2);
                    border-top-color: #2563eb;
                    border-radius: 50%;
                    animation: spin 1s linear infinite;
                }
                @keyframes spin {
                    to { transform: rotate(360deg); }
                }
                #status {
                    text-align: center;
                    padding: 8px;
                    margin-top: 10px;
                    background: rgba(30, 30, 30, 0.6);
                    border-radius: 6px;
                    font-size: 18px;
                    font-weight: 500;
                }
                .connected { color: #2563eb; }
                .disconnected { color: #8b0000; }
                
                .control-panel {
                    background: rgba(30, 30, 30, 0.8);
                    backdrop-filter: blur(10px);
                    border-radius: 12px;
                    padding: 15px;
                    box-shadow: 0 20px 60px rgba(0, 0, 0, 0.5);
                    overflow-y: auto;
                    max-height: calc(100vh - 100px);
                    border: 1px solid rgba(100, 100, 100, 0.2);
                }
                .control-panel::-webkit-scrollbar {
                    width: 8px;
                }
                .control-panel::-webkit-scrollbar-track {
                    background: rgba(50, 50, 50, 0.3);
                    border-radius: 4px;
                }
                .control-panel::-webkit-scrollbar-thumb {
                    background: rgba(100, 100, 100, 0.5);
                    border-radius: 4px;
                }
                .control-panel::-webkit-scrollbar-thumb:hover {
                    background: rgba(100, 100, 100, 0.7);
                }
                .panel-section {
                    background: rgba(40, 40, 40, 0.5);
                    border-radius: 8px;
                    padding: 8px;
                    margin-bottom: 8px;
                    border: 1px solid rgba(100, 100, 100, 0.15);
                }
                .panel-section h3 {
                    margin-bottom: 8px;
                    color: #d0d0d0;
                    font-size: 16px;
                    font-weight: 600;
                    display: flex;
                    align-items: center;
                    gap: 6px;
                }
                .panel-section h3::before {
                    content: '';
                    width: 2px;
                    height: 12px;
                    background: linear-gradient(135deg, #2563eb 0%, #8b0000 100%);
                    border-radius: 1px;
                }
                .control-item {
                    margin-bottom: 8px;
                }
                .control-item:last-child {
                    margin-bottom: 0;
                }
                .control-item label {
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                    margin-bottom: 4px;
                    font-size: 15px;
                    color: #b0b0b0;
                    font-weight: 500;
                }
                .control-item input[type="range"] {
                    width: 100%;
                    height: 5px;
                    border-radius: 3px;
                    background: rgba(100, 100, 100, 0.3);
                    outline: none;
                    -webkit-appearance: none;
                    transition: background 0.2s;
                }
                .control-item input[type="range"]:hover {
                    background: rgba(100, 100, 100, 0.4);
                }
                .control-item input[type="range"]::-webkit-slider-thumb {
                    -webkit-appearance: none;
                    width: 15px;
                    height: 15px;
                    border-radius: 50%;
                    background: linear-gradient(135deg, #2563eb 0%, #8b0000 100%);
                    cursor: pointer;
                    box-shadow: 0 2px 8px rgba(139, 0, 0, 0.4);
                    transition: transform 0.2s;
                }
                .control-item input[type="range"]::-webkit-slider-thumb:hover {
                    transform: scale(1.1);
                }
                .control-item input[type="range"]::-moz-range-thumb {
                    width: 15px;
                    height: 15px;
                    border-radius: 50%;
                    background: linear-gradient(135deg, #2563eb 0%, #8b0000 100%);
                    cursor: pointer;
                    border: none;
                    box-shadow: 0 2px 8px rgba(139, 0, 0, 0.4);
                }
                .control-item input[type="checkbox"] {
                    width: 14px;
                    height: 14px;
                    cursor: pointer;
                    accent-color: #2563eb;
                }
                .control-item input[type="number"] {
                    width: 60px;
                    padding: 4px 6px;
                    background: rgba(50, 50, 50, 0.5);
                    border: 1px solid rgba(100, 100, 100, 0.3);
                    border-radius: 4px;
                    color: #d0d0d0;
                    font-size: 15px;
                    text-align: center;
                    outline: none;
                    transition: all 0.2s;
                }
                .control-item input[type="number"]:focus {
                    background: rgba(50, 50, 50, 0.7);
                    border-color: #2563eb;
                }
                .value-display {
                    color: #2563eb;
                    font-weight: 600;
                    font-size: 15px;
                    min-width: 30px;
                    text-align: right;
                }
                .hint {
                    font-size: 11px;
                    color: #707070;
                    margin-top: 2px;
                }
                .stat-item {
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                    padding: 6px 0;
                    border-bottom: 1px solid rgba(100, 100, 100, 0.15);
                }
                .stat-item:last-child {
                    border-bottom: none;
                }
                .stat-label {
                    color: #909090;
                    font-size: 15px;
                    font-weight: 500;
                }
                .stat-value {
                    color: #d0d0d0;
                    font-weight: 600;
                    font-size: 15px;
                }
                .progress-bar {
                    width: 100%;
                    height: 5px;
                    background: rgba(50, 50, 50, 0.5);
                    border-radius: 3px;
                    overflow: hidden;
                    margin-top: 4px;
                }
                .progress-fill {
                    height: 100%;
                    background: linear-gradient(90deg, #2563eb 0%, #8b0000 100%);
                    transition: width 0.3s ease;
                    border-radius: 3px;
                }
                .toggle-label {
                    display: flex;
                    align-items: center;
                    gap: 8px;
                    cursor: pointer;
                }
            </style>
        </head>
        <body>
            <div class="container">
                <header>
                    <h1>ESP32-CAM Live Stream</h1>
                </header>
                <div class="layout">
                    <div class="stream-section">
                        <div class="stream-container">
                            <img id="stream" alt="Camera Stream">
                            <div class="stream-placeholder" id="placeholder">
                                <div class="spinner"></div>
                            </div>
                        </div>
                        <div id="status" class="disconnected">Connecting to server...</div>
                    </div>
                    
                    <div class="control-panel">
                        <div class="panel-section">
                            <h3>System Monitor</h3>
                            <div class="stat-item">
                                <span class="stat-label">Frame Rate</span>
                                <span class="stat-value" id="fps">--</span>
                            </div>
                            <div class="stat-item">
                                <span class="stat-label">WiFi Signal</span>
                                <span class="stat-value" id="wifi">--</span>
                            </div>
                            <div class="stat-item">
                                <span class="stat-label">Motor Mode</span>
                                <span class="stat-value" id="motor_mode" style="color: #2563eb;">--</span>
                            </div>
                            <div class="stat-item">
                                <span class="stat-label">Right Motor</span>
                                <span class="stat-value" id="motor_right" style="color: #10b981;">--</span>
                            </div>
                            <div class="stat-item">
                                <span class="stat-label">Left Motor</span>
                                <span class="stat-value" id="motor_left" style="color: #10b981;">--</span>
                            </div>
                            <div class="stat-item">
                                <span class="stat-label">SRAM</span>
                                <span class="stat-value" id="sram">--</span>
                            </div>
                            <div class="progress-bar">
                                <div class="progress-fill" id="sram-bar" style="width:0%"></div>
                            </div>
                            <div class="stat-item">
                                <span class="stat-label">PSRAM</span>
                                <span class="stat-value" id="psram">--</span>
                            </div>
                            <div class="progress-bar">
                                <div class="progress-fill" id="psram-bar" style="width:0%"></div>
                            </div>
                        </div>
                        
                        <div class="panel-section">
                            <h3>Image Quality</h3>
                            <div class="control-item">
                                <label><span>Brightness</span> <span class="value-display" id="brightness-val">0</span></label>
                                <input type="range" id="brightness" min="-2" max="2" value="0" step="1">
                            </div>
                            <div class="control-item">
                                <label><span>Contrast</span> <span class="value-display" id="contrast-val">0</span></label>
                                <input type="range" id="contrast" min="-2" max="2" value="0" step="1">
                            </div>
                            <div class="control-item">
                                <label><span>Saturation</span> <span class="value-display" id="saturation-val">-1</span></label>
                                <input type="range" id="saturation" min="-2" max="2" value="-1" step="1">
                            </div>
                            <div class="control-item">
                                <label><span>Quality</span> <span class="value-display" id="quality-val">8</span></label>
                                <input type="range" id="quality" min="0" max="63" value="8" step="1">
                                <div class="hint">Lower = better quality, higher = smaller file</div>
                            </div>
                        </div>
                        
                        <div class="panel-section">
                            <h3>Exposure</h3>
                            <div class="control-item">
                                <label class="toggle-label">
                                    <input type="checkbox" id="aec" checked> 
                                    <span>Auto Exposure</span>
                                </label>
                            </div>
                            <div class="control-item" id="ae-level-control">
                                <label><span>AE Level</span> <span class="value-display" id="ae_level-val">1</span></label>
                                <input type="range" id="ae_level" min="-2" max="2" value="1" step="1">
                                <div class="hint">Brightness compensation</div>
                            </div>
                            <div class="control-item" id="aec-value-control" style="display:none;">
                                <label>
                                    <span>Manual Exposure</span>
                                    <div style="display: flex; align-items: center; gap: 8px;">
                                        <span class="value-display" id="aec_value-val">300</span>
                                        <input type="number" id="aec_value_num" min="0" max="1200" value="300" step="10">
                                    </div>
                                </label>
                                <input type="range" id="aec_value" min="0" max="1200" value="300" step="10">
                                <div class="hint">0 = darkest, 1200 = brightest</div>
                            </div>
                        </div>
                        
                        <div class="panel-section">
                            <h3>LED Flash</h3>
                            <div class="control-item">
                                <label class="toggle-label">
                                    <input type="checkbox" id="led">
                                    <span>Enable Flash</span>
                                </label>
                            </div>
                            <div class="control-item">
                                <label><span>Intensity</span> <span class="value-display" id="led_intensity-val">0</span></label>
                                <input type="range" id="led_intensity" min="0" max="255" value="0" step="1">
                                <div class="hint">0 = off, 255 = maximum brightness</div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            
            <script>
                // Dynamically connect to the same host and port as the web page
                const wsUrl = 'ws://' + window.location.host;
                const ws = new WebSocket(wsUrl);
                const img = document.getElementById('stream');
                const status = document.getElementById('status');
                let frameCount = 0;
                let lastFrameTime = Date.now();
                
                console.log('Connecting to WebSocket:', wsUrl);
                
                // Camera control sliders and toggles
                const controls = {
                    brightness: { elem: document.getElementById('brightness'), val: document.getElementById('brightness-val') },
                    contrast: { elem: document.getElementById('contrast'), val: document.getElementById('contrast-val') },
                    saturation: { elem: document.getElementById('saturation'), val: document.getElementById('saturation-val') },
                    quality: { elem: document.getElementById('quality'), val: document.getElementById('quality-val') },
                    ae_level: { elem: document.getElementById('ae_level'), val: document.getElementById('ae_level-val') },
                    aec_value: { elem: document.getElementById('aec_value'), val: document.getElementById('aec_value-val') },
                    led_intensity: { elem: document.getElementById('led_intensity'), val: document.getElementById('led_intensity-val') }
                };
                const toggles = {
                    aec: document.getElementById('aec'),
                    led: document.getElementById('led')
                };
                // Show/hide manual controls based on auto toggles
                function updateExposureUI() {
                    if (toggles.aec.checked) {
                        document.getElementById('ae-level-control').style.display = '';
                        document.getElementById('aec-value-control').style.display = 'none';
                    } else {
                        document.getElementById('ae-level-control').style.display = 'none';
                        document.getElementById('aec-value-control').style.display = '';
                    }
                }
                toggles.aec.addEventListener('change', updateExposureUI);
                updateExposureUI();

                // Sync number inputs with sliders for manual controls
                const aecValueSlider = document.getElementById('aec_value');
                const aecValueNum = document.getElementById('aec_value_num');
                
                aecValueSlider.addEventListener('input', (e) => {
                    aecValueNum.value = e.target.value;
                    controls.aec_value.val.textContent = e.target.value;
                });
                aecValueNum.addEventListener('input', (e) => {
                    aecValueSlider.value = e.target.value;
                    controls.aec_value.val.textContent = e.target.value;
                });
                aecValueNum.addEventListener('change', sendAllSettings);

                // Send all settings to ESP32
                function sendAllSettings() {
                    const cmd = { cmd: 'set_camera' };
                    // Sliders
                    Object.keys(controls).forEach(key => {
                        let value = parseInt(controls[key].elem.value);
                        cmd[key] = value;
                    });
                    // Toggles
                    cmd.aec = toggles.aec.checked ? 1 : 0;
                    cmd.led = toggles.led.checked ? 1 : 0;
                    if (ws.readyState === WebSocket.OPEN) {
                        ws.send(JSON.stringify(cmd));
                        console.log('Sent:', cmd);
                    }
                }

                // Update display and send on change
                Object.keys(controls).forEach(key => {
                    const slider = controls[key].elem;
                    const display = controls[key].val;
                    slider.addEventListener('input', (e) => {
                        const value = parseInt(e.target.value);
                        display.textContent = value;
                    });
                    slider.addEventListener('change', sendAllSettings);
                });
                Object.keys(toggles).forEach(key => {
                    toggles[key].addEventListener('change', sendAllSettings);
                });
                
                ws.onopen = () => {
                    status.textContent = 'Connected, waiting for ESP32...';
                    status.className = 'connected';
                    
                    // Send initial camera settings
                    setTimeout(sendAllSettings, 1000);
                };
                
                ws.onmessage = (event) => {
                    if (event.data instanceof Blob || event.data instanceof ArrayBuffer) {
                        // Image frame
                        const blob = event.data instanceof Blob ? event.data : new Blob([event.data], {type: 'image/jpeg'});
                        
                        // Update stream
                        const oldSrc = img.src;
                        img.src = URL.createObjectURL(blob);
                        img.style.display = 'block';
                        placeholder.style.display = 'none';
                        
                        // Cleanup old blob
                        if (oldSrc && oldSrc.startsWith('blob:')) {
                            setTimeout(() => URL.revokeObjectURL(oldSrc), 100);
                        }
                        
                        // Update FPS
                        frameCount++;
                        const now = Date.now();
                        if (now - lastFrameTime >= 1000) {
                            const fps = Math.round(frameCount * 1000 / (now - lastFrameTime));
                            status.textContent = \`Live - \${fps} FPS (\${Math.round(blob.size/1024)}KB)\`;
                            frameCount = 0;
                            lastFrameTime = now;
                        }
                    } else if (typeof event.data === 'string') {
                        // JSON stats
                        try {
                            const data = JSON.parse(event.data);
                            if (data.type === 'stats') {
                                document.getElementById('fps').textContent = data.fps.toFixed(1) + ' FPS';
                                document.getElementById('wifi').textContent = data.wifi_rssi + ' dBm';
                                
                                // Update motor status from ESP-NOW
                                if (data.motor_mode !== undefined) {
                                    const modeNames = ['FORWARD', 'REVERSE', 'RIGHT', 'LEFT', 'STOP'];
                                    const modeName = modeNames[data.motor_mode] || 'UNKNOWN';
                                    document.getElementById('motor_mode').textContent = modeName;
                                }
                                if (data.motor_right !== undefined) {
                                    document.getElementById('motor_right').textContent = data.motor_right + ' / 255';
                                }
                                if (data.motor_left !== undefined) {
                                    document.getElementById('motor_left').textContent = data.motor_left + ' / 255';
                                }
                                
                                const sramPct = (data.sram_used / data.sram_total * 100).toFixed(1);
                                document.getElementById('sram').textContent = 
                                    \`\${(data.sram_used/1024).toFixed(0)}KB / \${(data.sram_total/1024).toFixed(0)}KB\`;
                                document.getElementById('sram-bar').style.width = sramPct + '%';
                                
                                const psramPct = (data.psram_used / data.psram_total * 100).toFixed(1);
                                document.getElementById('psram').textContent = 
                                    \`\${(data.psram_used/1024).toFixed(0)}KB / \${(data.psram_total/1024).toFixed(0)}KB\`;
                                document.getElementById('psram-bar').style.width = psramPct + '%';
                            }
                        } catch(e) {}
                    }
                };
                
                ws.onclose = () => {
                    status.textContent = 'Disconnected';
                    status.className = 'disconnected';
                    placeholder.style.display = 'flex';
                };
                
                ws.onerror = () => {
                    status.textContent = 'Connection Error';
                    status.className = 'disconnected';
                };
            </script>
        </body>
        </html>`;
        
        res.writeHead(200, {'Content-Type': 'text/html'});
        res.end(html);
    } else {
        res.writeHead(404, {'Content-Type': 'text/plain'});
        res.end('404 Not Found');
    }
});

// Create WebSocket server with optimized settings
const wss = new WebSocket.Server({ 
    server,
    perMessageDeflate: false, // Disable compression for speed
    maxPayload: 100 * 1024,   // 100KB max payload for images
    backlog: 511,             // Increase backlog for better connection handling
    clientTracking: true      // Track clients for broadcasting
});

let esp32Connected = false;
let webClients = new Set();
let esp32Client = null;

wss.on('connection', (ws, req) => {
    const clientIP = req.socket.remoteAddress;
    const userAgent = req.headers['user-agent'] || '';
    
    // Detect if this is ESP32-CAM or web browser
    const isESP32 = !userAgent.includes('Mozilla') && !userAgent.includes('Chrome');
    
    if (isESP32) {
        console.log('[ESP32] Connected');
        esp32Connected = true;
        esp32Client = ws;
        
        // Notify all web clients that ESP32 is connected
        webClients.forEach(client => {
            if (client.readyState === WebSocket.OPEN) {
                client.send('ESP32_CONNECTED');
            }
        });
        
        ws.on('message', async (data) => {
            if (Buffer.isBuffer(data)) {
                if (data.length < 1000) {
                    // JSON stats
                    try {
                        const text = data.toString('utf8');
                        const json = JSON.parse(text);
                        if (json.type === 'stats') {
                            // Broadcast to web clients
                            webClients.forEach(client => {
                                if (client.readyState === WebSocket.OPEN) {
                                    client.send(text);
                                }
                            });
                        }
                    } catch (e) {
                        // Not JSON, ignore
                    }
                } else {
                    // JPEG frame - process and broadcast to web clients
                    try {
                        let processedFrame = data;
                        
                        if (IMAGE_PROCESSING.enabled) {
                            processedFrame = await sharp(data)
                                .blur(0.3)
                                .median(IMAGE_PROCESSING.median)
                                .modulate({
                                    brightness: IMAGE_PROCESSING.brightness,
                                    saturation: IMAGE_PROCESSING.saturation
                                })
                                .jpeg({ quality: IMAGE_PROCESSING.jpegQuality })
                                .toBuffer();
                        }
                        
                        // Broadcast to web clients
                        webClients.forEach(client => {
                            if (client.readyState === WebSocket.OPEN) {
                                try {
                                    client.send(processedFrame);
                                } catch (error) {
                                    console.error('[ERROR] Failed to send frame');
                                    webClients.delete(client);
                                }
                            }
                        });
                    } catch (error) {
                        console.error('[ERROR] Image processing failed:', error.message);
                    }
                }
            }
        });
        
        ws.on('close', () => {
            console.log('[ESP32] Disconnected');
            esp32Connected = false;
            esp32Client = null;
            
            // Notify web clients that ESP32 disconnected
            webClients.forEach(client => {
                if (client.readyState === WebSocket.OPEN) {
                    client.send('ESP32_DISCONNECTED');
                }
            });
        });
        
    } else {
        // This is a web browser client
        console.log('[BROWSER] Connected');
        webClients.add(ws);
        
        // Send current ESP32 status
        if (esp32Connected) {
            ws.send('ESP32_CONNECTED');
        }
        
        // Handle control commands from web interface
        ws.on('message', (data) => {
            // Forward control commands to ESP32
            if (esp32Client && esp32Client.readyState === WebSocket.OPEN) {
                try {
                    const dataStr = typeof data === 'string' ? data : data.toString();
                    const cmd = JSON.parse(dataStr);
                    if (cmd.cmd === 'set_camera') {
                        esp32Client.send(dataStr);
                    }
                } catch (e) {
                    console.error('[ERROR] Invalid control command');
                }
            }
        });
        
        ws.on('close', () => {
            console.log('[BROWSER] Disconnected');
            webClients.delete(ws);
        });
    }
    
    ws.on('error', (error) => {
        console.error('[ERROR] WebSocket error:', error);
    });
});

// Function to find available port
function findAvailablePort(startPort, callback) {
    const net = require('net');
    const testServer = net.createServer();
    
    testServer.listen(startPort, (err) => {
        if (err) {
            testServer.close();
            findAvailablePort(startPort + 1, callback);
        } else {
            const port = testServer.address().port;
            testServer.close();
            callback(port);
        }
    });
    
    testServer.on('error', (err) => {
        findAvailablePort(startPort + 1, callback);
    });
}

// Get local IP helper function
function getLocalIP() {
    const { networkInterfaces } = require('os');
    const nets = networkInterfaces();
    
    // Prefer WiFi interfaces (192.168.1.x) over ethernet (192.168.56.x)
    let wifiIP = null;
    let otherIP = null;
    
    for (const name of Object.keys(nets)) {
        for (const net of nets[name]) {
            if (net.family === 'IPv4' && !net.internal) {
                if (net.address.startsWith('192.168.1.')) {
                    wifiIP = net.address;
                } else if (!otherIP) {
                    otherIP = net.address;
                }
            }
        }
    }
    
    return wifiIP || otherIP || 'localhost';
}

// Start server on available port
findAvailablePort(8080, (port) => {
    server.listen(port, () => {
        console.log('\n' + '='.repeat(50));
        console.log(' ESP32-CAM Server Started');
        console.log('='.repeat(50));
        console.log(`Server: http://localhost:${port}`);
        console.log(`Network: http://${getLocalIP()}:${port}`);
        console.log(`ESP32 target: ws://${getLocalIP()}:${port}`);
        console.log('='.repeat(50) + '\n');
    });
});

// Graceful shutdown
process.on('SIGINT', () => {
    console.log('\n[INFO] Shutting down server...');
    wss.clients.forEach(client => {
        client.close();
    });
    server.close(() => {
        console.log('[INFO] Server closed');
        process.exit(0);
    });
    // Force exit after 1 second if graceful shutdown fails
    setTimeout(() => {
        console.log('[INFO] Force closing...');
        process.exit(0);
    }, 1000);
});

// Handle Windows Ctrl+C specifically
if (process.platform === "win32") {
    const readline = require("readline");
    const rl = readline.createInterface({
        input: process.stdin,
        output: process.stdout
    });
    rl.on("SIGINT", () => {
        process.emit("SIGINT");
    });
}