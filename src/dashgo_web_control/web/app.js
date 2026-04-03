const state = {
  mapVersion: -1,
  pathVersion: -1,
  status: null,
  map: null,
  path: [],
  pendingGoal: null,
  goalPickArmed: false,
  mapView: {
    scale: 1,
    minScale: 1,
    maxScale: 1,
    centerX: 0,
    centerY: 0,
    headingReferenceYaw: null,
    initialized: false,
  },
};

const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");
const statusLine = document.getElementById("status-line");
const poseText = document.getElementById("pose-text");
const velocityText = document.getElementById("velocity-text");
const mapText = document.getElementById("map-text");
const goalText = document.getElementById("goal-text");
const manualPoseText = document.getElementById("manual-pose-text");
const manualVelocityText = document.getElementById("manual-velocity-text");
const manualRadarStatus = document.getElementById("manual-radar-status");
const manualRadarCanvas = document.getElementById("manual-radar-canvas");
const manualRadarCaption = document.getElementById("manual-radar-caption");
const radarCtx = manualRadarCanvas ? manualRadarCanvas.getContext("2d") : null;
const pageTitle = document.getElementById("page-title");
const navPage = document.getElementById("nav-page");
const manualPage = document.getElementById("manual-page");

const joystickBase = document.getElementById("joystick-base");
const joystickKnob = document.getElementById("joystick-knob");
const stopButton = document.getElementById("stop-button");
const refreshMapButton = document.getElementById("refresh-map-button");
const manualModeButton = document.getElementById("manual-mode-button");
const navModeButton = document.getElementById("nav-mode-button");
const fitMapButton = document.getElementById("fit-map-button");
const pickGoalButton = document.getElementById("pick-goal-button");
const clearGoalButton = document.getElementById("clear-goal-button");
const confirmGoalButton = document.getElementById("confirm-goal-button");
const modeText = document.getElementById("mode-text");
const modeDot = document.getElementById("mode-dot");
const radarDot = document.getElementById("radar-dot");
const baseDot = document.getElementById("base-dot");
const DEFAULT_LOCAL_VIEW_SCALE_FACTOR = 2.4;
const JOYSTICK_SNAP_MIN_MAGNITUDE = 0.18;
const JOYSTICK_SNAP_COS_THRESHOLD = Math.cos(Math.PI / 10);
const JOYSTICK_SNAP_STRENGTH = 0.38;
let joystickPointerId = null;
let joystickCenter = null;
let joystickRadius = 0;
let cmdVelTimer = null;
let cmdVelLoop = null;
let cmdVelRequestInFlight = false;
let lastCmd = { linear: 0, angular: 0 };
let mapGesture = null;
let mapPointers = new Map();
let mapImage = null;
let statusOverrideUntil = 0;
let renderQueued = false;

function clamp(value, low, high) {
  return Math.max(low, Math.min(high, value));
}

function formatNumber(value) {
  return Number.isFinite(value) ? value.toFixed(2) : "--";
}

function formatDistance(value) {
  return Number.isFinite(value) ? `${value.toFixed(2)} m` : "--";
}

function drawRadarPanel() {
  if (!manualRadarCanvas || !radarCtx) {
    return;
  }

  const size = Math.max(220, Math.floor(manualRadarCanvas.clientWidth || manualRadarCanvas.width || 320));
  if (manualRadarCanvas.width !== size || manualRadarCanvas.height !== size) {
    manualRadarCanvas.width = size;
    manualRadarCanvas.height = size;
  }

  const width = manualRadarCanvas.width;
  const height = manualRadarCanvas.height;
  const centerX = width / 2;
  const centerY = height / 2;
  const radius = Math.min(width, height) * 0.42;
  const radar = state.status?.scan || {};
  const points = Array.isArray(radar.points) ? radar.points : [];
  const rangeMax = Number.isFinite(radar.range_max) && radar.range_max > 0.1 ? radar.range_max : 6.0;

  radarCtx.clearRect(0, 0, width, height);

  const bg = radarCtx.createRadialGradient(centerX, centerY, radius * 0.1, centerX, centerY, radius);
  bg.addColorStop(0, 'rgba(255,255,255,0.98)');
  bg.addColorStop(1, 'rgba(220,233,247,0.98)');
  radarCtx.fillStyle = bg;
  radarCtx.fillRect(0, 0, width, height);

  radarCtx.save();
  radarCtx.translate(centerX, centerY);

  radarCtx.strokeStyle = 'rgba(17, 51, 85, 0.10)';
  radarCtx.lineWidth = 1;
  for (let i = 1; i <= 4; i += 1) {
    radarCtx.beginPath();
    radarCtx.arc(0, 0, radius * i / 4, 0, Math.PI * 2);
    radarCtx.stroke();
  }
  for (let i = 0; i < 8; i += 1) {
    const angle = (Math.PI / 4) * i;
    radarCtx.beginPath();
    radarCtx.moveTo(0, 0);
    radarCtx.lineTo(Math.cos(angle) * radius, Math.sin(angle) * radius);
    radarCtx.stroke();
  }

  radarCtx.fillStyle = 'rgba(15, 118, 214, 0.08)';
  radarCtx.beginPath();
  radarCtx.moveTo(0, 0);
  radarCtx.arc(0, 0, radius, -Math.PI / 2 - Math.PI / 6, -Math.PI / 2 + Math.PI / 6);
  radarCtx.closePath();
  radarCtx.fill();

  radarCtx.fillStyle = 'rgba(240, 173, 44, 0.82)';
  for (const point of points) {
    if (!Array.isArray(point) || point.length < 2) {
      continue;
    }
    const x = Number(point[0]);
    const y = Number(point[1]);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      continue;
    }
    const canvasX = -y / rangeMax * radius;
    const canvasY = -x / rangeMax * radius;
    radarCtx.beginPath();
    radarCtx.arc(canvasX, canvasY, 2.1, 0, Math.PI * 2);
    radarCtx.fill();
  }

  radarCtx.fillStyle = '#0f76d6';
  radarCtx.beginPath();
  radarCtx.moveTo(0, -12);
  radarCtx.lineTo(-9, 10);
  radarCtx.lineTo(9, 10);
  radarCtx.closePath();
  radarCtx.fill();

  radarCtx.restore();
}

function setStatusMessage(message, durationMs = 0) {
  statusLine.textContent = message;
  statusOverrideUntil = durationMs > 0 ? Date.now() + durationMs : 0;
}

function setDefaultStatus(message) {
  if (Date.now() < statusOverrideUntil) {
    return;
  }
  statusLine.textContent = message;
}

function requestRender() {
  if (renderQueued) {
    return;
  }
  renderQueued = true;
  window.requestAnimationFrame(() => {
    renderQueued = false;
    renderScene();
  });
}

async function apiGet(url) {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`${url} -> ${response.status}`);
  }
  return response.json();
}

async function apiPost(url, payload) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload || {}),
  });
  if (!response.ok) {
    throw new Error(`${url} -> ${response.status}`);
  }
  return response.json();
}

function resizeCanvas() {
  const centerWorld = state.map && state.mapView.initialized
    ? canvasToWorld(canvas.width / 2, canvas.height / 2)
    : null;
  const logicalWidth = canvas.parentElement
    ? canvas.parentElement.clientWidth
    : canvas.clientWidth;
  const size = Math.max(320, Math.floor(logicalWidth));
  canvas.width = size;
  canvas.height = size;
  if (state.map) {
    syncMapView(centerWorld);
  } else {
    requestRender();
  }
}

function setPageMode(mode) {
  const wasNavActive = navPage.classList.contains("active");
  manualModeButton.classList.toggle("active", mode === "manual");
  navModeButton.classList.toggle("active", mode === "nav");
  navPage.classList.toggle("active", mode === "nav");
  manualPage.classList.toggle("active", mode === "manual");
  pageTitle.textContent = mode === "manual" ? "手动页面" : "导航页面";
  if (mode === "nav" && !wasNavActive) {
    window.requestAnimationFrame(() => {
      resizeCanvas();
    });
  }
}

function setStatusDot(element, isOnline, onlineClass = "online") {
  element.classList.remove("online", "offline", "manual", "nav");
  element.classList.add(isOnline ? onlineClass : "offline");
}

function getMapPixelFromWorld(x, y) {
  if (!state.map) {
    return { x: 0, y: 0 };
  }
  const { height, resolution, origin } = state.map;
  const px = (x - origin.x) / resolution;
  const py = (y - origin.y) / resolution;
  return {
    x: px,
    y: height - py,
  };
}

function getWorldFromMapPixel(x, y) {
  if (!state.map) {
    return { x: 0, y: 0 };
  }
  const { height, resolution, origin } = state.map;
  return {
    x: origin.x + x * resolution,
    y: origin.y + (height - y) * resolution,
  };
}

function rotateVector(x, y, angle) {
  const cosAngle = Math.cos(angle);
  const sinAngle = Math.sin(angle);
  return {
    x: x * cosAngle - y * sinAngle,
    y: x * sinAngle + y * cosAngle,
  };
}

function getRobotMapPixel() {
  if (!state.map || !state.status?.odom) {
    return null;
  }
  return getMapPixelFromWorld(state.status.odom.x, state.status.odom.y);
}

function getCanvasCenter() {
  return {
    x: canvas.width / 2,
    y: canvas.height / 2,
  };
}

function getMapRotation() {
  const yaw = state.mapView.headingReferenceYaw;
  return Number.isFinite(yaw) ? yaw - Math.PI / 2 : 0;
}

function getScreenRobotYaw() {
  const yaw = state.status?.odom?.yaw;
  if (!Number.isFinite(yaw)) {
    return 0;
  }
  return -yaw + getMapRotation();
}

function setViewCenterForMapPixelAtCanvasPoint(mapPixelX, mapPixelY, canvasX, canvasY, scale = state.mapView.scale) {
  const canvasCenter = getCanvasCenter();
  const rotation = getMapRotation();
  const relativeCanvas = rotateVector(
    canvasX - canvasCenter.x,
    canvasY - canvasCenter.y,
    -rotation,
  );

  state.mapView.centerX = mapPixelX - relativeCanvas.x / scale;
  state.mapView.centerY = mapPixelY - relativeCanvas.y / scale;
}

function mapPixelToCanvas(
  x,
  y,
  scale = state.mapView.scale,
  centerX = state.mapView.centerX,
  centerY = state.mapView.centerY,
) {
  const canvasCenter = getCanvasCenter();
  const rotation = getMapRotation();
  const relative = rotateVector(
    (x - centerX) * scale,
    (y - centerY) * scale,
    rotation,
  );

  return {
    x: canvasCenter.x + relative.x,
    y: canvasCenter.y + relative.y,
  };
}

function canvasToMapPixel(
  x,
  y,
  scale = state.mapView.scale,
  centerX = state.mapView.centerX,
  centerY = state.mapView.centerY,
) {
  const canvasCenter = getCanvasCenter();
  const rotation = getMapRotation();
  const relative = rotateVector(
    x - canvasCenter.x,
    y - canvasCenter.y,
    -rotation,
  );

  return {
    x: centerX + relative.x / scale,
    y: centerY + relative.y / scale,
  };
}

function worldToCanvas(x, y) {
  if (!state.map) {
    return { x: 0, y: 0 };
  }
  const mapPixel = getMapPixelFromWorld(x, y);
  return mapPixelToCanvas(mapPixel.x, mapPixel.y);
}

function canvasToWorld(x, y) {
  if (!state.map) {
    return { x: 0, y: 0 };
  }
  const mapPixel = canvasToMapPixel(x, y);
  return getWorldFromMapPixel(mapPixel.x, mapPixel.y);
}

function getGridCellAtMapPixel(mapPixelX, mapPixelY) {
  if (!state.map) {
    return null;
  }

  const gridX = Math.floor(mapPixelX);
  const imageY = Math.floor(mapPixelY);
  if (
    gridX < 0 ||
    imageY < 0 ||
    gridX >= state.map.width ||
    imageY >= state.map.height
  ) {
    return null;
  }

  const gridY = state.map.height - 1 - imageY;
  const index = gridY * state.map.width + gridX;
  const value = state.map.data?.[index];
  if (!Number.isFinite(value)) {
    return null;
  }

  return {
    x: gridX,
    y: gridY,
    value,
  };
}

function isCellFree(cell) {
  if (!cell) {
    return false;
  }
  return cell.value >= 0 && cell.value < 15;
}

function findNearestFreeGoal(mapPixelX, mapPixelY, maxRadius = 12) {
  if (!state.map) {
    return null;
  }

  const baseX = Math.floor(mapPixelX);
  const baseImageY = Math.floor(mapPixelY);
  let bestCell = null;
  let bestDistanceSq = Number.POSITIVE_INFINITY;

  for (let radius = 0; radius <= maxRadius; radius += 1) {
    for (let dy = -radius; dy <= radius; dy += 1) {
      for (let dx = -radius; dx <= radius; dx += 1) {
        if (Math.max(Math.abs(dx), Math.abs(dy)) !== radius) {
          continue;
        }

        const cell = getGridCellAtMapPixel(baseX + dx, baseImageY + dy);
        if (!isCellFree(cell)) {
          continue;
        }

        const distanceSq = dx * dx + dy * dy;
        if (distanceSq < bestDistanceSq) {
          bestDistanceSq = distanceSq;
          bestCell = cell;
        }
      }
    }

    if (bestCell) {
      return {
        mapPixelX: bestCell.x + 0.5,
        mapPixelY: state.map.height - bestCell.y - 0.5,
        world: getWorldFromMapPixel(bestCell.x + 0.5, state.map.height - bestCell.y - 0.5),
      };
    }
  }

  return null;
}

function getCanvasEventPoint(event) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = rect.width > 0 ? canvas.width / rect.width : 1;
  const scaleY = rect.height > 0 ? canvas.height / rect.height : 1;
  return {
    x: (event.clientX - rect.left) * scaleX,
    y: (event.clientY - rect.top) * scaleY,
  };
}

function getMapPointerEntries() {
  return Array.from(mapPointers.entries()).slice(0, 2);
}

function beginMapPanGesture(pointerId, point, moved = false) {
  mapGesture = {
    mode: "pan",
    pointerId,
    startX: point.x,
    startY: point.y,
    startCenterX: state.mapView.centerX,
    startCenterY: state.mapView.centerY,
    moved,
  };
}

function beginMapPinchGesture() {
  const pointerEntries = getMapPointerEntries();
  if (pointerEntries.length < 2) {
    return;
  }

  const first = pointerEntries[0][1];
  const second = pointerEntries[1][1];
  const centerX = (first.x + second.x) / 2;
  const centerY = (first.y + second.y) / 2;
  const focus = canvasToMapPixel(centerX, centerY);

  mapGesture = {
    mode: "pinch",
    startDistance: Math.max(Math.hypot(second.x - first.x, second.y - first.y), 1),
    startScale: state.mapView.scale,
    focusMapX: focus.x,
    focusMapY: focus.y,
  };
}

function updateMapPinchGesture() {
  const pointerEntries = getMapPointerEntries();
  if (pointerEntries.length < 2) {
    return;
  }

  if (!mapGesture || mapGesture.mode !== "pinch") {
    beginMapPinchGesture();
  }
  if (!mapGesture || mapGesture.mode !== "pinch") {
    return;
  }

  const first = pointerEntries[0][1];
  const second = pointerEntries[1][1];
  const centerX = (first.x + second.x) / 2;
  const centerY = (first.y + second.y) / 2;
  const distance = Math.max(Math.hypot(second.x - first.x, second.y - first.y), 1);
  const nextScale = clamp(
    mapGesture.startScale * (distance / mapGesture.startDistance),
    state.mapView.minScale,
    state.mapView.maxScale,
  );
  state.mapView.scale = nextScale;
  setViewCenterForMapPixelAtCanvasPoint(
    mapGesture.focusMapX,
    mapGesture.focusMapY,
    centerX,
    centerY,
    nextScale,
  );
  requestRender();
}

function getFitScale() {
  if (!state.map) {
    return 1;
  }
  const margin = 28;
  return Math.min(
    (canvas.width - margin * 2) / state.map.width,
    (canvas.height - margin * 2) / state.map.height,
  );
}

function getDefaultMapScale(centerOnRobot = true) {
  const fitScale = getFitScale();
  if (!centerOnRobot) {
    return fitScale;
  }
  return fitScale * DEFAULT_LOCAL_VIEW_SCALE_FACTOR;
}

function centerMapOn(mapPixelX, mapPixelY) {
  state.mapView.centerX = mapPixelX;
  state.mapView.centerY = mapPixelY;
}

function resetMapView(centerOnRobot = true) {
  if (!state.map) {
    return;
  }
  const fitScale = getFitScale();
  state.mapView.minScale = fitScale * 0.85;
  state.mapView.maxScale = fitScale * 8.0;
  state.mapView.scale = clamp(
    getDefaultMapScale(centerOnRobot),
    state.mapView.minScale,
    state.mapView.maxScale,
  );
  state.mapView.initialized = true;

  const robot = centerOnRobot ? state.status?.odom : null;
  if (robot) {
    const mapPixel = getMapPixelFromWorld(robot.x, robot.y);
    centerMapOn(mapPixel.x, mapPixel.y);
  } else {
    state.mapView.centerX = state.map.width / 2;
    state.mapView.centerY = state.map.height / 2;
  }
  requestRender();
}

function syncMapView(centerWorld = null) {
  if (!state.map) {
    return;
  }

  const fitScale = getFitScale();

  if (!state.mapView.initialized) {
    resetMapView(Boolean(centerWorld));
    return;
  }

  state.mapView.minScale = fitScale * 0.85;
  state.mapView.maxScale = fitScale * 8.0;
  state.mapView.scale = clamp(
    state.mapView.scale,
    state.mapView.minScale,
    state.mapView.maxScale,
  );

  if (centerWorld) {
    const centerPixel = getMapPixelFromWorld(centerWorld.x, centerWorld.y);
    centerMapOn(centerPixel.x, centerPixel.y);
  }

  requestRender();
}

function zoomAtCanvasPoint(canvasX, canvasY, factor) {
  if (!state.map) {
    return;
  }
  const focus = canvasToMapPixel(canvasX, canvasY);
  const nextScale = clamp(
    state.mapView.scale * factor,
    state.mapView.minScale,
    state.mapView.maxScale,
  );
  state.mapView.scale = nextScale;
  setViewCenterForMapPixelAtCanvasPoint(
    focus.x,
    focus.y,
    canvasX,
    canvasY,
    nextScale,
  );
  requestRender();
}

function zoomMap(factor) {
  zoomAtCanvasPoint(canvas.width / 2, canvas.height / 2, factor);
}

function deriveGoalYaw(goal) {
  const robot = state.status?.odom;
  if (!robot) {
    return 0.0;
  }
  return Math.atan2(goal.y - robot.y, goal.x - robot.x);
}

function hasSameMapGeometry(previousMap, nextMap) {
  if (!previousMap || !nextMap) {
    return false;
  }
  return (
    previousMap.width === nextMap.width &&
    previousMap.height === nextMap.height &&
    previousMap.resolution === nextMap.resolution &&
    previousMap.origin?.x === nextMap.origin?.x &&
    previousMap.origin?.y === nextMap.origin?.y
  );
}

function rebuildMapImage() {
  if (!state.map) {
    mapImage = null;
    state.mapView.initialized = false;
    return;
  }

  const { width, height, data } = state.map;
  const imageData = new ImageData(width, height);

  for (let i = 0; i < data.length; i += 1) {
    const occupancy = data[i];
    const srcX = i % width;
    const srcY = Math.floor(i / width);
    const imageY = height - 1 - srcY;
    const index = (imageY * width + srcX) * 4;

    let r = 246;
    let g = 250;
    let b = 255;
    if (occupancy < 0) {
      r = 223;
      g = 231;
      b = 242;
    } else if (occupancy >= 100) {
      r = 31;
      g = 48;
      b = 71;
    } else if (occupancy > 0) {
      const shade = 248 - Math.round((occupancy / 100.0) * 185);
      r = shade;
      g = shade + 2;
      b = shade + 8;
    }

    imageData.data[index] = clamp(r, 0, 255);
    imageData.data[index + 1] = clamp(g, 0, 255);
    imageData.data[index + 2] = clamp(b, 0, 255);
    imageData.data[index + 3] = 255;
  }

  const offscreen = document.createElement("canvas");
  offscreen.width = width;
  offscreen.height = height;
  offscreen.getContext("2d").putImageData(imageData, 0, 0);
  mapImage = offscreen;
}

function drawMapBackdrop() {
  const gradient = ctx.createLinearGradient(0, 0, 0, canvas.height);
  gradient.addColorStop(0, "#f9fcff");
  gradient.addColorStop(1, "#e9f0f7");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = "rgba(17, 51, 85, 0.08)";
  ctx.lineWidth = 1;
  for (let x = 24; x < canvas.width; x += 24) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }
  for (let y = 24; y < canvas.height; y += 24) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }
}

function drawMapGrid() {
  if (!state.map) {
    return;
  }

  const meterStepInPixels = 1.0 / state.map.resolution;
  const spacing = meterStepInPixels * state.mapView.scale;
  if (spacing < 28) {
    return;
  }

  const corners = [
    canvasToMapPixel(0, 0),
    canvasToMapPixel(canvas.width, 0),
    canvasToMapPixel(0, canvas.height),
    canvasToMapPixel(canvas.width, canvas.height),
  ];
  const minX = Math.min(...corners.map((point) => point.x));
  const maxX = Math.max(...corners.map((point) => point.x));
  const minY = Math.min(...corners.map((point) => point.y));
  const maxY = Math.max(...corners.map((point) => point.y));
  const xStart = Math.floor(minX / meterStepInPixels) * meterStepInPixels;
  const xEnd = Math.ceil(maxX / meterStepInPixels) * meterStepInPixels;
  const yStart = Math.floor(minY / meterStepInPixels) * meterStepInPixels;
  const yEnd = Math.ceil(maxY / meterStepInPixels) * meterStepInPixels;

  ctx.save();
  ctx.strokeStyle = "rgba(15, 118, 214, 0.12)";
  ctx.lineWidth = 1;

  for (let x = xStart; x <= xEnd; x += meterStepInPixels) {
    const start = mapPixelToCanvas(x, yStart);
    const end = mapPixelToCanvas(x, yEnd);
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(end.x, end.y);
    ctx.stroke();
  }

  for (let y = yStart; y <= yEnd; y += meterStepInPixels) {
    const start = mapPixelToCanvas(xStart, y);
    const end = mapPixelToCanvas(xEnd, y);
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(end.x, end.y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawRobot() {
  if (!state.status?.odom || !state.map) {
    return;
  }

  const pose = state.status.odom;
  const p = worldToCanvas(pose.x, pose.y);
  const radiusPx = Math.max(
    10,
    (state.status.robot_radius / state.map.resolution) * state.mapView.scale,
  );

  ctx.save();
  ctx.translate(p.x, p.y);
  ctx.rotate(getScreenRobotYaw());
  ctx.fillStyle = "#0f76d6";
  ctx.beginPath();
  ctx.arc(0, 0, radiusPx, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "#ffffff";
  ctx.beginPath();
  ctx.moveTo(radiusPx * 0.95, 0);
  ctx.lineTo(-radiusPx * 0.22, radiusPx * 0.58);
  ctx.lineTo(-radiusPx * 0.22, -radiusPx * 0.58);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawPath() {
  if (!state.path || state.path.length < 2) {
    return;
  }

  ctx.save();
  ctx.strokeStyle = "#15a56f";
  ctx.lineWidth = 3;
  ctx.beginPath();
  state.path.forEach((point, index) => {
    const p = worldToCanvas(point.x, point.y);
    if (index === 0) {
      ctx.moveTo(p.x, p.y);
    } else {
      ctx.lineTo(p.x, p.y);
    }
  });
  ctx.stroke();
  ctx.restore();
}

function drawGoal() {
  const goal = state.pendingGoal || state.status?.goal;
  if (!goal) {
    return;
  }

  const p = worldToCanvas(goal.x, goal.y);
  const accent = state.pendingGoal ? "#ff8a00" : "#d64b28";

  ctx.save();
  ctx.strokeStyle = accent;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(p.x, p.y, 12, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(p.x - 16, p.y);
  ctx.lineTo(p.x + 16, p.y);
  ctx.moveTo(p.x, p.y - 16);
  ctx.lineTo(p.x, p.y + 16);
  ctx.stroke();

  if (Number.isFinite(goal.yaw)) {
    ctx.translate(p.x, p.y);
    ctx.rotate(-goal.yaw + getMapRotation());
    ctx.fillStyle = accent;
    ctx.beginPath();
    ctx.moveTo(20, 0);
    ctx.lineTo(5, 7);
    ctx.lineTo(5, -7);
    ctx.closePath();
    ctx.fill();
  }

  if (state.pendingGoal) {
    ctx.fillStyle = "#ff8a00";
    ctx.font = "700 13px 'Noto Sans SC', sans-serif";
    ctx.fillText("待确认", p.x + 14, p.y - 14);
  }
  ctx.restore();
}

function renderScene() {
  drawMapBackdrop();

  if (mapImage && state.map) {
    ctx.save();
    ctx.imageSmoothingEnabled = false;
    const canvasCenter = getCanvasCenter();
    ctx.translate(canvasCenter.x, canvasCenter.y);
    ctx.rotate(getMapRotation());
    ctx.scale(state.mapView.scale, state.mapView.scale);
    ctx.translate(-state.mapView.centerX, -state.mapView.centerY);
    ctx.drawImage(
      mapImage,
      0,
      0,
      state.map.width,
      state.map.height,
    );
    ctx.restore();
    drawMapGrid();
  }

  drawPath();
  drawGoal();
  drawRobot();
}

async function fetchStatus() {
  try {
    const payload = await apiGet("/api/status");
    const shouldLockHeading = (
      state.mapView.headingReferenceYaw === null &&
      Number.isFinite(payload?.odom?.yaw)
    );
    state.status = payload;
    if (shouldLockHeading) {
      state.mapView.headingReferenceYaw = payload.odom.yaw;
      if (state.map && state.mapView.initialized) {
        resetMapView();
        return;
      }
    }
    setDefaultStatus(
      payload.has_odom
        ? "已连接机器人，可缩放查看地图；点“选点导航”后再确认发起导航。"
        : "已连网页服务，等待机器人位姿。",
    );

    if (payload.map_version !== state.mapVersion) {
      state.mapVersion = payload.map_version;
      fetchMap();
    }

    if (payload.path_version !== state.pathVersion) {
      state.pathVersion = payload.path_version;
      fetchPath();
    }

    updateStatusCards();
    requestRender();
  } catch (error) {
    setStatusMessage(`连接失败：${error.message}`, 3000);
  }
}

async function fetchMap() {
  const previousMap = state.map;
  const previousView = {
    initialized: state.mapView.initialized,
    scale: state.mapView.scale,
    centerX: state.mapView.centerX,
    centerY: state.mapView.centerY,
  };
  const centerWorld = state.map && state.mapView.initialized
    ? canvasToWorld(canvas.width / 2, canvas.height / 2)
    : null;
  const payload = await apiGet("/api/map");
  if (!payload.ok) {
    return;
  }
  state.map = payload.map;
  rebuildMapImage();

  if (previousView.initialized && hasSameMapGeometry(previousMap, state.map)) {
    const fitScale = getFitScale();
    state.mapView.minScale = fitScale * 0.85;
    state.mapView.maxScale = fitScale * 8.0;
    state.mapView.scale = clamp(
      previousView.scale,
      state.mapView.minScale,
      state.mapView.maxScale,
    );
    state.mapView.centerX = previousView.centerX;
    state.mapView.centerY = previousView.centerY;
    state.mapView.initialized = true;
    requestRender();
  } else if (centerWorld) {
    syncMapView(centerWorld);
  } else {
    resetMapView();
  }
  updateStatusCards();
}

async function fetchPath() {
  const payload = await apiGet("/api/path");
  if (!payload.ok) {
    return;
  }
  state.path = payload.path;
  requestRender();
}

function updateGoalControls() {
  pickGoalButton.classList.toggle("active", state.goalPickArmed);
  pickGoalButton.textContent = state.goalPickArmed ? "选点中..." : "选点导航";
  clearGoalButton.disabled = !state.pendingGoal;
  confirmGoalButton.disabled = !state.pendingGoal;
}

function updateStatusCards() {
  const odom = state.status?.odom;
  const goal = state.pendingGoal || state.status?.goal;
  const mode = state.status?.control_mode || "nav";
  const isPaused = mode === "pause";
  const hasActivePath = Array.isArray(state.path) && state.path.length > 1;
  const showResume = isPaused && hasActivePath;
  const devices = state.status?.devices || {};
  const radar = state.status?.scan || {};

  poseText.textContent = odom
    ? `x ${formatNumber(odom.x)}  y ${formatNumber(odom.y)}  yaw ${formatNumber(odom.yaw)}`
    : "--";
  velocityText.textContent = odom
    ? `v ${formatNumber(odom.linear_x)}  w ${formatNumber(odom.angular_z)}`
    : "--";
  manualPoseText.textContent = poseText.textContent;
  manualVelocityText.textContent = velocityText.textContent;
  mapText.textContent = state.map
    ? `${state.map.width}×${state.map.height} @ ${formatNumber(state.map.resolution)}m`
    : "等待地图";
  goalText.textContent = goal
    ? `${state.pendingGoal ? "候选" : "目标"} x ${formatNumber(goal.x)}  y ${formatNumber(goal.y)}`
    : "未设置";

  if (manualRadarStatus) {
    manualRadarStatus.textContent = devices.radar ? "在线" : "离线";
    manualRadarStatus.classList.toggle("online", Boolean(devices.radar));
    manualRadarStatus.classList.toggle("offline", !devices.radar);
  }
  if (manualRadarCaption) {
    const count = Number.isFinite(radar.valid_count) ? radar.valid_count : 0;
    const nearest = formatDistance(radar.nearest_range);
    manualRadarCaption.textContent = devices.radar
      ? `有效点 ${count} 个，最近障碍 ${nearest}，上方为车头方向`
      : '等待雷达扫描...';
  }
  drawRadarPanel();

  joystickBase.classList.toggle("disabled", mode !== "manual");
  stopButton.textContent = showResume ? "继续" : "即停";
  modeText.textContent = mode === "manual" ? "手动" : isPaused ? "暂停" : "导航";
  setStatusDot(modeDot, true, mode === "manual" || isPaused ? "manual" : "nav");
  setStatusDot(radarDot, Boolean(devices.radar));
  setStatusDot(baseDot, Boolean(devices.base));
  updateGoalControls();
}

function getJoystickMetrics() {
  const rect = joystickBase.getBoundingClientRect();
  joystickCenter = {
    x: rect.left + rect.width / 2,
    y: rect.top + rect.height / 2,
  };
  joystickRadius = rect.width * 0.32;
}

function setJoystickPosition(nx, ny) {
  const x = nx * joystickRadius;
  const y = ny * joystickRadius;
  joystickKnob.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
}

function applyJoystickSnap(nx, ny) {
  const magnitude = Math.hypot(nx, ny);
  if (magnitude < JOYSTICK_SNAP_MIN_MAGNITUDE) {
    return { nx, ny };
  }

  const unitX = nx / magnitude;
  const unitY = ny / magnitude;
  const targets = [
    { x: 1, y: 0 },
    { x: -1, y: 0 },
    { x: 0, y: 1 },
    { x: 0, y: -1 },
  ];

  let bestTarget = null;
  let bestDot = -Infinity;
  for (const target of targets) {
    const dot = unitX * target.x + unitY * target.y;
    if (dot > bestDot) {
      bestDot = dot;
      bestTarget = target;
    }
  }

  if (!bestTarget || bestDot < JOYSTICK_SNAP_COS_THRESHOLD) {
    return { nx, ny };
  }

  const blend = ((bestDot - JOYSTICK_SNAP_COS_THRESHOLD) / (1 - JOYSTICK_SNAP_COS_THRESHOLD)) * JOYSTICK_SNAP_STRENGTH;
  const mixX = unitX * (1 - blend) + bestTarget.x * blend;
  const mixY = unitY * (1 - blend) + bestTarget.y * blend;
  const mixMagnitude = Math.hypot(mixX, mixY) || 1;

  return {
    nx: (mixX / mixMagnitude) * magnitude,
    ny: (mixY / mixMagnitude) * magnitude,
  };
}

async function flushCmdVel() {
  if (state.status?.control_mode !== "manual") {
    return;
  }
  if (cmdVelRequestInFlight) {
    return;
  }
  cmdVelRequestInFlight = true;
  try {
    await apiPost("/api/cmd_vel", lastCmd);
  } catch (error) {
    setStatusMessage(`速度指令发送失败：${error.message}`, 3000);
  } finally {
    cmdVelRequestInFlight = false;
  }
}

function ensureCmdVelLoop() {
  if (cmdVelLoop) {
    return;
  }
  cmdVelLoop = setInterval(() => {
    flushCmdVel();
  }, 120);
}

function stopCmdVelLoop() {
  if (!cmdVelLoop) {
    return;
  }
  clearInterval(cmdVelLoop);
  cmdVelLoop = null;
}

function queueCmdVel(linear, angular) {
  if (state.status?.control_mode !== "manual") {
    return;
  }
  lastCmd = { linear, angular };
  if (cmdVelTimer) {
    return;
  }
  cmdVelTimer = setTimeout(() => {
    cmdVelTimer = null;
    flushCmdVel();
  }, 60);
}

function resetJoystick() {
  stopCmdVelLoop();
  setJoystickPosition(0, 0);
  lastCmd = { linear: 0, angular: 0 };
  apiPost("/api/hold", {}).catch(() => {});
}

function onJoystickMove(clientX, clientY) {
  const dx = clientX - joystickCenter.x;
  const dy = clientY - joystickCenter.y;
  const distance = Math.hypot(dx, dy) || 1;
  const scale = Math.min(1, joystickRadius / distance);
  const rawNx = (dx * scale) / joystickRadius;
  const rawNy = (dy * scale) / joystickRadius;
  const { nx, ny } = applyJoystickSnap(rawNx, rawNy);

  setJoystickPosition(nx, ny);

  const linearLimit = state.status?.limits?.linear ?? 0.25;
  const angularLimit = state.status?.limits?.angular ?? 1.2;
  queueCmdVel(-ny * linearLimit, -nx * angularLimit);
}

joystickBase.addEventListener("pointerdown", (event) => {
  if (state.status?.control_mode !== "manual") {
    setStatusMessage("导航模式下摇杆已禁用，请先切到手动模式。", 2200);
    return;
  }
  joystickPointerId = event.pointerId;
  getJoystickMetrics();
  ensureCmdVelLoop();
  joystickBase.setPointerCapture(event.pointerId);
  onJoystickMove(event.clientX, event.clientY);
});

joystickBase.addEventListener("pointermove", (event) => {
  if (event.pointerId !== joystickPointerId) {
    return;
  }
  onJoystickMove(event.clientX, event.clientY);
});

function releaseJoystick(event) {
  if (event.pointerId !== joystickPointerId) {
    return;
  }
  joystickPointerId = null;
  resetJoystick();
}

joystickBase.addEventListener("pointerup", releaseJoystick);
joystickBase.addEventListener("pointercancel", releaseJoystick);

canvas.addEventListener("wheel", (event) => {
  if (!state.map) {
    return;
  }
  event.preventDefault();
  const { x, y } = getCanvasEventPoint(event);
  const factor = event.deltaY < 0 ? 1.12 : 1 / 1.12;
  zoomAtCanvasPoint(x, y, factor);
}, { passive: false });

canvas.addEventListener("pointerdown", (event) => {
  if (!state.map) {
    return;
  }
  const point = getCanvasEventPoint(event);
  mapPointers.set(event.pointerId, point);
  if (mapPointers.size >= 2) {
    beginMapPinchGesture();
  } else {
    beginMapPanGesture(event.pointerId, point);
  }
  canvas.setPointerCapture(event.pointerId);
});

canvas.addEventListener("pointermove", (event) => {
  if (!state.map || !mapPointers.has(event.pointerId)) {
    return;
  }
  const point = getCanvasEventPoint(event);
  mapPointers.set(event.pointerId, point);

  if (mapPointers.size >= 2) {
    updateMapPinchGesture();
    return;
  }

  if (!mapGesture || mapGesture.mode !== "pan" || event.pointerId !== mapGesture.pointerId) {
    return;
  }

  const { x, y } = point;
  const dx = x - mapGesture.startX;
  const dy = y - mapGesture.startY;

  if (Math.hypot(dx, dy) > 6) {
    mapGesture.moved = true;
  }

  if (mapGesture.moved) {
    const relativeMap = rotateVector(dx, dy, -getMapRotation());
    state.mapView.centerX = mapGesture.startCenterX - relativeMap.x / state.mapView.scale;
    state.mapView.centerY = mapGesture.startCenterY - relativeMap.y / state.mapView.scale;
    requestRender();
  }
});

function finishMapGesture(event) {
  if (!state.map || !mapPointers.has(event.pointerId)) {
    return;
  }

  const point = getCanvasEventPoint(event);
  mapPointers.set(event.pointerId, point);

  const wasTap = (
    mapGesture &&
    mapGesture.mode === "pan" &&
    event.pointerId === mapGesture.pointerId &&
    !mapGesture.moved &&
    mapPointers.size === 1
  );

  if (wasTap && state.goalPickArmed) {
    if (state.status?.control_mode === "manual") {
      setStatusMessage("当前是手动模式，先切到导航模式再选目标。", 2500);
    } else {
      const mapPixel = canvasToMapPixel(point.x, point.y);
      const snappedGoal = findNearestFreeGoal(mapPixel.x, mapPixel.y);
      if (!snappedGoal) {
        setStatusMessage("这里是未知区或障碍区，请换一个空白区域选点。", 2600);
        mapPointers.delete(event.pointerId);
        if (mapPointers.size === 0) {
          mapGesture = null;
        }
        return;
      }

      const goal = snappedGoal.world;
      state.pendingGoal = {
        x: goal.x,
        y: goal.y,
        yaw: deriveGoalYaw(goal),
      };
      state.goalPickArmed = false;
      updateStatusCards();
      requestRender();
      setStatusMessage("候选目标已选中，请点击“确认导航”后发送。", 2500);
    }
  }

  mapPointers.delete(event.pointerId);

  if (mapPointers.size >= 2) {
    beginMapPinchGesture();
  } else if (mapPointers.size === 1) {
    const [pointerId, remainingPoint] = Array.from(mapPointers.entries())[0];
    beginMapPanGesture(pointerId, remainingPoint, true);
  } else {
    mapGesture = null;
  }
}

canvas.addEventListener("pointerup", finishMapGesture);
canvas.addEventListener("pointercancel", finishMapGesture);

stopButton.addEventListener("click", async () => {
  try {
    const response = await apiPost("/api/stop", {});
    resetJoystick();
    state.goalPickArmed = false;
    if (state.status && response?.mode) {
      state.status.control_mode = response.mode;
    }
    updateStatusCards();
    setStatusMessage(
      response?.mode === "pause" ? "已暂停当前导航。" : "已继续当前导航。",
      2200,
    );
  } catch (error) {
    setStatusMessage(`暂停失败：${error.message}`, 3000);
  }
});

manualModeButton.addEventListener("click", async () => {
  try {
    await apiPost("/api/mode", { mode: "manual" });
    await apiPost("/api/hold", {});
    if (state.status) {
      state.status.control_mode = "manual";
    }
    setPageMode("manual");
    setStatusMessage("已切到手动模式。", 1800);
    updateStatusCards();
  } catch (error) {
    setStatusMessage(`切换手动模式失败：${error.message}`, 3000);
  }
});

navModeButton.addEventListener("click", async () => {
  try {
    await apiPost("/api/mode", { mode: "nav" });
    resetJoystick();
    if (state.status) {
      state.status.control_mode = "nav";
    }
    setPageMode("nav");
    setStatusMessage("已切到导航模式。", 1800);
    updateStatusCards();
  } catch (error) {
    setStatusMessage(`切换导航模式失败：${error.message}`, 3000);
  }
});

fitMapButton.addEventListener("click", () => {
  resetMapView();
  setStatusMessage("地图视图已重置。", 1200);
});

pickGoalButton.addEventListener("click", () => {
  state.goalPickArmed = !state.goalPickArmed;
  if (state.goalPickArmed) {
    setStatusMessage("请轻点地图选择目标点，确认后才会开始导航。", 2500);
  } else {
    setStatusMessage("已退出选点。", 1200);
  }
  updateGoalControls();
});

clearGoalButton.addEventListener("click", () => {
  state.pendingGoal = null;
  state.goalPickArmed = false;
  updateStatusCards();
  requestRender();
  setStatusMessage("已清除候选目标。", 1200);
});

confirmGoalButton.addEventListener("click", async () => {
  if (!state.pendingGoal) {
    return;
  }
  if (state.status?.control_mode === "manual") {
    setStatusMessage("当前是手动模式，先切到导航模式再确认。", 2500);
    return;
  }

  const payload = { ...state.pendingGoal };
  try {
    await apiPost("/api/goal", payload);
    state.pendingGoal = null;
    updateStatusCards();
    requestRender();
    fetchStatus();
    setStatusMessage("导航目标已确认并发送。", 1800);
  } catch (error) {
    setStatusMessage(`目标发送失败：${error.message}`, 3000);
  }
});

refreshMapButton.addEventListener("click", () => {
  fetchMap().catch((error) => {
    setStatusMessage(`刷新地图失败：${error.message}`, 3000);
  });
});

window.addEventListener("resize", () => {
  resizeCanvas();
  drawRadarPanel();
});

resizeCanvas();
setPageMode("nav");
drawRadarPanel();
fetchStatus();
fetchMap().catch(() => {});
fetchPath().catch(() => {});
setInterval(fetchStatus, 800);
