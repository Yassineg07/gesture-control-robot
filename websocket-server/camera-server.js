const WebSocket = require('ws');
const http = require('http');
const sharp = require('sharp');

const IMAGE_PROCESSING = {
    enabled: true,
    median: 2,
    brightness: 1.05,
    saturation: 1.05,
    jpegQuality: 75
};

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
                    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
                    color: #fff;
                    min-height: 100vh;
                    padding: 10px;
                }
                .container { max-width: 1400px; width: 100%; margin: 0 auto; }
                header { text-align: center; margin-bottom: 18px; }
                h1 {
                    font-size: 30px;
                    font-weight: 700;
                    background: linear-gradient(135deg, #2563eb 0%, #8b0000 100%);
                    -webkit-background-clip: text;
                    -webkit-text-fill-color: transparent;
                    background-clip: text;
                }
                .layout {
                    display: grid;
                    grid-template-columns: 1fr 400px;
                    gap: 32px;
                    height: 92vh;
                    min-height: 700px;
                }
                @media (max-width: 1024px) {
                    .layout { grid-template-columns: 1fr; height: auto; }
                }
                .stream-container {
                    position: relative;
                    background: #000;
                    border-radius: 12px;
                    overflow: hidden;
                    display: flex;
                }
                #stream { width: 100%; height: 100%; object-fit: contain; display: none; }
                .stream-placeholder {
                    position: absolute;
                    inset: 0;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    background: rgba(0, 0, 0, 0.5);
                }
                #status {
                    text-align: center;
                    padding: 8px;
                    margin-top: 10px;
                    background: rgba(30, 30, 30, 0.6);
                    border-radius: 6px;
                    font-size: 18px;
                }
                .connected { color: #2563eb; }
                .disconnected { color: #8b0000; }
                .control-panel {
                    background: rgba(30, 30, 30, 0.8);
                    border-radius: 8px;
                    padding: 6px;
                    overflow-y: auto;
                    max-height: calc(100vh - 40px);
                }
                .panel-section {
                    background: rgba(40, 40, 40, 0.5);
                    border-radius: 7px;
                    padding: 7px;
                    margin-bottom: 7px;
                }
                .panel-section h3 {
                    margin-bottom: 6px;
                    color: #d0d0d0;
                    font-size: 17px;
                    font-weight: 700;
                }
                .control-item { margin-bottom: 6px; }
                .control-item label {
                    display: flex;
                    justify-content: space-between;
                    margin-bottom: 4px;
                    font-size: 17px;
                    color: #b0b0b0;
                }
                .control-item input[type="range"] {
                    width: 100%;
                    height: 7px;
                    background: rgba(100, 100, 100, 0.3);
                    -webkit-appearance: none;
                }
                .control-item input[type="range"]::-webkit-slider-thumb {
                    -webkit-appearance: none;
                    width: 15px;
                    height: 15px;
                    border-radius: 50%;
                    background: linear-gradient(135deg, #2563eb 0%, #8b0000 100%);
                    cursor: pointer;
                }
                .value-display {
                    color: #2563eb;
                    font-weight: 700;
                    min-width: 20px;
                }
                .stat-item {
                    display: flex;
                    justify-content: space-between;
                    padding: 6px 0;
                    border-bottom: 1px solid rgba(100, 100, 100, 0.15);
                }
                .stat-label { color: #909090; font-size: 15px; }
                .stat-value { color: #d0d0d0; font-size: 15px; font-weight: 600; }
                .progress-bar {
                    width: 100%;
                    height: 5px;
                    background: rgba(50, 50, 50, 0.5);
                    border-radius: 3px;
                    margin-top: 4px;
                }
                .progress-fill {
                    height: 100%;
                    background: linear-gradient(90deg, #2563eb 0%, #8b0000 100%);
                    transition: width 0.3s;
                }
            </style>
        </head>
        <body>
            <div class="container">
                <header><h1>ESP32-CAM Live Stream</h1></header>
                <div class="layout">
                    <div class="stream-section">
                        <div class="stream-container">
                            <img id="stream" alt="Stream">
                            <div class="stream-placeholder" id="placeholder"></div>
                        </div>
                        <div id="status" class="disconnected">Connecting...</div>
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
                                <span class="stat-value" id="motor_mode">--</span>
                            </div>
                            <div class="stat-item">
                                <span class="stat-label">Right Motor</span>
                                <span class="stat-value" id="motor_right">--</span>
                            </div>
                            <div class="stat-item">
                                <span class="stat-label">Left Motor</span>
                                <span class="stat-value" id="motor_left">--</span>
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
                            <h3>Camera Controls</h3>
                            <div class="control-item">
                                <label><span>Brightness</span> <span class="value-display" id="brightness-val">0</span></label>
                                <input type="range" id="brightness" min="-2" max="2" value="0">
                            </div>
                            <div class="control-item">
                                <label><span>Contrast</span> <span class="value-display" id="contrast-val">0</span></label>
                                <input type="range" id="contrast" min="-2" max="2" value="0">
                            </div>
                            <div class="control-item">
                                <label><span>Saturation</span> <span class="value-display" id="saturation-val">-1</span></label>
                                <input type="range" id="saturation" min="-2" max="2" value="-1">
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            
            <script>
                const wsUrl = 'ws://' + window.location.host;
                const ws = new WebSocket(wsUrl);
                const img = document.getElementById('stream');
                const status = document.getElementById('status');
                
                const controls = {
                    brightness: { elem: document.getElementById('brightness'), val: document.getElementById('brightness-val') },
                    contrast: { elem: document.getElementById('contrast'), val: document.getElementById('contrast-val') },
                    saturation: { elem: document.getElementById('saturation'), val: document.getElementById('saturation-val') }
                };

                function sendAllSettings() {
                    const cmd = { cmd: 'set_camera' };
                    Object.keys(controls).forEach(key => {
                        cmd[key] = parseInt(controls[key].elem.value);
                    });
                    if (ws.readyState === WebSocket.OPEN) {
                        ws.send(JSON.stringify(cmd));
                    }
                }

                Object.keys(controls).forEach(key => {
                    const slider = controls[key].elem;
                    const display = controls[key].val;
                    slider.addEventListener('input', (e) => {
                        display.textContent = e.target.value;
                    });
                    slider.addEventListener('change', sendAllSettings);
                });
                
                ws.onopen = () => {
                    status.textContent = 'Connected';
                    status.className = 'connected';
                    setTimeout(sendAllSettings, 1000);
                };
                
                ws.onmessage = (event) => {
                    if (event.data instanceof Blob || event.data instanceof ArrayBuffer) {
                        const blob = event.data instanceof Blob ? event.data : new Blob([event.data]);
                        const oldSrc = img.src;
                        img.src = URL.createObjectURL(blob);
                        img.style.display = 'block';
                        placeholder.style.display = 'none';
                        if (oldSrc && oldSrc.startsWith('blob:')) {
                            setTimeout(() => URL.revokeObjectURL(oldSrc), 100);
                        }
                    } else if (typeof event.data === 'string') {
                        try {
                            const data = JSON.parse(event.data);
                            if (data.type === 'stats') {
                                document.getElementById('fps').textContent = data.fps.toFixed(1) + ' FPS';
                                document.getElementById('wifi').textContent = data.wifi_rssi + ' dBm';
                                
                                const modes = ['FORWARD', 'REVERSE', 'RIGHT', 'LEFT', 'STOP'];
                                document.getElementById('motor_mode').textContent = modes[data.motor_mode] || 'UNKNOWN';
                                document.getElementById('motor_right').textContent = data.motor_right + ' / 255';
                                document.getElementById('motor_left').textContent = data.motor_left + ' / 255';
                                
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
                };
            </script>
        </body>
        </html>`;
        
        res.writeHead(200, {'Content-Type': 'text/html'});
        res.end(html);
    } else {
        res.writeHead(404);
        res.end('404');
    }
});

const wss = new WebSocket.Server({ 
    server,
    perMessageDeflate: false,
    maxPayload: 100 * 1024
});

let esp32Client = null;
let webClients = new Set();

wss.on('connection', (ws, req) => {
    const userAgent = req.headers['user-agent'] || '';
    const isESP32 = !userAgent.includes('Mozilla');
    
    if (isESP32) {
        console.log('[ESP32] Connected');
        esp32Client = ws;
        
        ws.on('message', async (data) => {
            if (Buffer.isBuffer(data)) {
                if (data.length < 1000) {
                    try {
                        const text = data.toString('utf8');
                        const json = JSON.parse(text);
                        if (json.type === 'stats') {
                            webClients.forEach(client => {
                                if (client.readyState === WebSocket.OPEN) {
                                    client.send(text);
                                }
                            });
                        }
                    } catch (e) {}
                } else {
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
                        
                        webClients.forEach(client => {
                            if (client.readyState === WebSocket.OPEN) {
                                try {
                                    client.send(processedFrame);
                                } catch (error) {
                                    webClients.delete(client);
                                }
                            }
                        });
                    } catch (error) {
                        console.error('[ERROR] Image processing failed');
                    }
                }
            }
        });
        
        ws.on('close', () => {
            console.log('[ESP32] Disconnected');
            esp32Client = null;
        });
        
    } else {
        console.log('[BROWSER] Connected');
        webClients.add(ws);
        
        ws.on('message', (data) => {
            if (esp32Client && esp32Client.readyState === WebSocket.OPEN) {
                try {
                    const dataStr = typeof data === 'string' ? data : data.toString();
                    const cmd = JSON.parse(dataStr);
                    if (cmd.cmd === 'set_camera') {
                        esp32Client.send(dataStr);
                    }
                } catch (e) {}
            }
        });
        
        ws.on('close', () => {
            console.log('[BROWSER] Disconnected');
            webClients.delete(ws);
        });
    }
});

function getLocalIP() {
    const { networkInterfaces } = require('os');
    const nets = networkInterfaces();
    
    for (const name of Object.keys(nets)) {
        for (const net of nets[name]) {
            if (net.family === 'IPv4' && !net.internal) {
                return net.address;
            }
        }
    }
    return 'localhost';
}

const port = 8080;
server.listen(port, () => {
    console.log('\nESP32-CAM Server Started');
    console.log(`Local: http://localhost:${port}`);
    console.log(`Network: http://${getLocalIP()}:${port}`);
});

process.on('SIGINT', () => {
    wss.clients.forEach(client => client.close());
    server.close(() => process.exit(0));
    setTimeout(() => process.exit(0), 1000);
});