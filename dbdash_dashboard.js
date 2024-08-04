const socket = io();
let heightmapData = null;
let startPoint = null;
let endPoint = null;
let pathData = null;
let isPlanning = false;
let map = null;
let heatmapLayer = null;
let startMarker = null;
let endMarker = null;
let pathLayer = null;

const testingModeToggle = document.getElementById('testingModeToggle');
const logConsole = document.getElementById('log-console');
const statusIndicator = document.getElementById('status-indicator');
const executePathBtn = document.getElementById('execute-path-btn');
const estopStatus = document.getElementById('estop-status');

executePathBtn.addEventListener('click', function() {
    socket.emit('step_forward', {});
    logMessage("Executing path...");
    statusIndicator.textContent = "Executing path...";
});

testingModeToggle.addEventListener('change', function() {
    socket.emit('toggle_testing_mode', {testing_mode: this.checked});
});

function logMessage(message, level = 'info') {
    console.log(`[${level.toUpperCase()}] ${message}`);
    const logEntry = document.createElement('div');
    logEntry.className = `log-entry log-${level}`;
    logEntry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
    logConsole.appendChild(logEntry);
    logConsole.scrollTop = logConsole.scrollHeight;
}

function initMap() {
    map = L.map('height-map', {
        crs: L.CRS.Simple,
        minZoom: -2
    });

    map.on('click', function(e) {
        if (isPlanning) {
            logMessage("Path planning in progress. Please wait.", "warning");
            return;
        }

        const y = Math.floor(e.latlng.lat);
        const x = Math.floor(e.latlng.lng);

        if (x >= 0 && x < heightmapData.width && y >= 0 && y < heightmapData.height) {
            if (!startPoint) {
                startPoint = [y, x];
                startMarker = L.marker([y, x], {icon: L.divIcon({className: 'start-marker', html: 'S'})}).addTo(map);
                statusIndicator.textContent = "Start point selected. Click to set end point.";
                logMessage(`Start point set at (${y}, ${x})`);
            } else if (!endPoint) {
                endPoint = [y, x];
                endMarker = L.marker([y, x], {icon: L.divIcon({className: 'end-marker', html: 'E'})}).addTo(map);
                statusIndicator.textContent = "End point selected. Planning path...";
                logMessage(`End point set at (${y}, ${x}). Planning path...`);

                isPlanning = true;
                socket.emit('plan_path', { start: startPoint, end: endPoint });
            }
        }
    });
}

function updateHeightMap() {
    if (!heightmapData || !map) return;

    if (heatmapLayer) {
        map.removeLayer(heatmapLayer);
    }

    const bounds = [[0, 0], [heightmapData.height, heightmapData.width]];
    heatmapLayer = L.imageOverlay(heightmapDataToImage(heightmapData), bounds).addTo(map);
    map.fitBounds(bounds);
}

function heightmapDataToImage(data) {
    const canvas = document.createElement('canvas');
    canvas.width = data.width;
    canvas.height = data.height;
    const ctx = canvas.getContext('2d');
    const imageData = ctx.createImageData(data.width, data.height);

    for (let i = 0; i < data.data.length; i++) {
        const value = Math.floor(data.data[i] * 255);
        imageData.data[i * 4] = value;
        imageData.data[i * 4 + 1] = value;
        imageData.data[i * 4 + 2] = value;
        imageData.data[i * 4 + 3] = 255;
    }

    ctx.putImageData(imageData, 0, 0);
    return canvas.toDataURL();
}

socket.on('sportmode_update', function(data) {
    const parsedData = JSON.parse(data);
    document.getElementById('velocity-x').textContent = parsedData.velocity_x.toFixed(3);
    document.getElementById('velocity-y').textContent = parsedData.velocity_y.toFixed(3);
    document.getElementById('yaw-speed').textContent = parsedData.yaw_speed.toFixed(3);
    document.getElementById('position-x').textContent = parsedData.position_x.toFixed(3);
    document.getElementById('position-y').textContent = parsedData.position_y.toFixed(3);
    document.getElementById('yaw-angle').textContent = parsedData.yaw.toFixed(3);
});

socket.on('pointcloud_update', function(data) {
    const parsedData = JSON.parse(data);
    const x = parsedData.points.map(p => p[0] / 2);
    const y = parsedData.points.map(p => p[1] / 2);
    const z = parsedData.points.map(p => p[2]);

    const pointsTrace = {
        x: x,
        y: y,
        mode: 'markers',
        type: 'scatter',
        marker: {
            size: 5,
            color: z,
            colorscale: 'Viridis',
            colorbar: {title: 'Height (m)'},
            cmin: -0.4,
            cmax: 0.0,
        }
    };

    Plotly.newPlot('point-cloud', [pointsTrace], {
        title: 'LiDAR Point Cloud (Top-Down View)',
        width: 350,
        height: 300,
        margin: { t: 30, b: 30, l: 30, r: 30 },
        xaxis: { title: 'X (m)', range: [-1.0, 1.0], scaleanchor: 'y', scaleratio: 1 },
        yaxis: { title: 'Y (m)', range: [-1.0, 1.0] },
        shapes: [{
            type: 'path',
            path: 'M 0 0 L 0 0.3 L -0.05 0.25 M 0 0.3 L 0.05 0.25',
            line: {
                color: 'red',
                width: 3
            }
        }],
        annotations: [{
            x: 0,
            y: -0.05,
            xref: 'x',
            yref: 'y',
            text: 'Go2',
            showarrow: false,
            font: {
                color: 'red'
            }
        }]
    });
});

socket.on('heightmap_update', function(data) {
    heightmapData = JSON.parse(data);
    if (!map) {
        initMap();
    }
    updateHeightMap();
});

socket.on('path_result', function(data) {
    pathData = JSON.parse(data);
    if (pathLayer) {
        map.removeLayer(pathLayer);
    }
    pathLayer = L.polyline(pathData, {color: 'blue'}).addTo(map);
    isPlanning = false;
    statusIndicator.textContent = "Path planning complete. Click 'Execute Path' to start.";
    logMessage(`Path planning complete. Path length: ${pathData.length} points`);
    executePathBtn.disabled = false;
});

socket.on('estop_status', function(data) {
    estopStatus.className = data.is_estopped ? 'estop-active' : 'estop-inactive';
    if (data.is_estopped) {
        statusIndicator.textContent = "E-Stop activated! Robot stopped.";
        logMessage("E-Stop activated! Robot stopped.", "warning");
    } else {
        statusIndicator.textContent = "";
        logMessage("E-Stop deactivated. Robot ready to move.");
    }
});

socket.on('log', function(data) {
    logMessage(data.message, data.level);
});

window.addEventListener('load', function() {
    logMessage("Dashboard loaded");
    statusIndicator.textContent = "Click on the height map to set start point.";
});