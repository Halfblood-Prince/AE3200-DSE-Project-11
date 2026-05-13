const utcTime = document.querySelector("#utc-time");

function tick() {
  const now = new Date();
  const formatter = new Intl.DateTimeFormat("en-US", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
    timeZone: "UTC"
  });
  utcTime.textContent = formatter.format(now);
}

tick();
setInterval(tick, 1000);

document.querySelectorAll(".metric strong").forEach((node, index) => {
  node.style.animationDelay = `${index * 45}ms`;
});

const cameraFeed = document.querySelector("#camera-feed");
const cameraStatus = document.querySelector("#camera-status");
let cameraSocket = null;
let cameraReconnectTimer = null;
let pendingCameraFrame = null;
let cameraDecoding = false;
let cameraFrameCount = 0;
let cameraFrameWindowStarted = performance.now();

function setCameraStatus(label) {
  if (!cameraStatus) {
    return;
  }
  cameraStatus.textContent = label;
}

function cameraStreamUrl() {
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${protocol}//${window.location.host}/api/camera/live`;
}

function drawCameraBitmap(bitmap) {
  const context = cameraFeed?.getContext?.("2d");
  if (!context) {
    return;
  }

  const rect = cameraFeed.getBoundingClientRect();
  const pixelRatio = window.devicePixelRatio || 1;
  const displayWidth = rect.width || bitmap.width;
  const displayHeight = rect.height || bitmap.height;
  const canvasWidth = Math.max(1, Math.round(displayWidth * pixelRatio));
  const canvasHeight = Math.max(1, Math.round(displayHeight * pixelRatio));

  if (cameraFeed.width !== canvasWidth || cameraFeed.height !== canvasHeight) {
    cameraFeed.width = canvasWidth;
    cameraFeed.height = canvasHeight;
  }

  const scale = Math.max(canvasWidth / bitmap.width, canvasHeight / bitmap.height);
  const drawWidth = bitmap.width * scale;
  const drawHeight = bitmap.height * scale;
  const offsetX = (canvasWidth - drawWidth) / 2;
  const offsetY = (canvasHeight - drawHeight) / 2;

  context.clearRect(0, 0, canvasWidth, canvasHeight);
  context.drawImage(bitmap, offsetX, offsetY, drawWidth, drawHeight);
}

function updateCameraFrameRate() {
  cameraFrameCount += 1;
  const now = performance.now();
  const elapsed = now - cameraFrameWindowStarted;
  if (elapsed < 1000) {
    return;
  }

  const fps = Math.round((cameraFrameCount * 1000) / elapsed);
  setCameraStatus(`${Math.min(fps, 60)} fps  Live`);
  cameraFrameCount = 0;
  cameraFrameWindowStarted = now;
}

async function decodeCameraFrames() {
  cameraDecoding = true;

  while (pendingCameraFrame) {
    const frame = pendingCameraFrame;
    pendingCameraFrame = null;

    try {
      const bitmap = await createImageBitmap(frame);
      await new Promise((resolve) => {
        requestAnimationFrame(() => {
          drawCameraBitmap(bitmap);
          bitmap.close?.();
          updateCameraFrameRate();
          resolve();
        });
      });
    } catch {
      cameraFeed.dataset.live = "false";
      setCameraStatus("Waiting");
    }
  }

  cameraDecoding = false;
}

function renderCameraFrame(frame) {
  pendingCameraFrame = frame;
  if (!cameraDecoding) {
    decodeCameraFrames();
  }
}

function connectCameraFeed() {
  if (!cameraFeed?.getContext) {
    return;
  }

  clearTimeout(cameraReconnectTimer);
  if (cameraSocket) {
    cameraSocket.onclose = null;
    cameraSocket.close();
  }

  setCameraStatus("Connecting");
  const socket = new WebSocket(cameraStreamUrl());
  cameraSocket = socket;
  socket.binaryType = "blob";

  socket.onopen = () => {
    cameraFeed.dataset.live = "true";
    setCameraStatus("Live");
    cameraFrameCount = 0;
    cameraFrameWindowStarted = performance.now();
  };

  socket.onmessage = (event) => {
    if (typeof event.data !== "string") {
      renderCameraFrame(event.data);
    }
  };

  socket.onerror = () => {
    cameraFeed.dataset.live = "false";
    setCameraStatus("Waiting");
  };

  socket.onclose = () => {
    if (cameraSocket !== socket) {
      return;
    }
    cameraSocket = null;
    cameraFeed.dataset.live = "false";
    setCameraStatus("Waiting");
    cameraReconnectTimer = setTimeout(connectCameraFeed, 1500);
  };
}

connectCameraFeed();

window.addEventListener("beforeunload", () => {
  if (cameraSocket) {
    cameraSocket.onclose = null;
    cameraSocket.close();
  }
});

const controlStatus = document.querySelector("#control-status");
const driveSpeedValue = document.querySelector("#drive-speed");
const driveSpeedUp = document.querySelector("#drive-speed-up");
const driveSpeedDown = document.querySelector("#drive-speed-down");
const driveSlow = document.querySelector("#drive-slow");
const driveFast = document.querySelector("#drive-fast");
const driveStop = document.querySelector("#drive-stop");
const activeDriveKeys = new Set();
const minDriveSpeed = 0.25;
const maxDriveSpeed = 2.5;
const driveCommandIntervalMs = 40;
let driveSpeed = 1.25;
const turnSpeed = 1.15;
let lastCommand = { linear_x: 0, angular_z: 0 };
let driveTick = null;

const keyToDrive = {
  arrowup: "forward",
  w: "forward",
  arrowdown: "back",
  s: "back",
  arrowleft: "left",
  a: "left",
  arrowright: "right",
  d: "right",
  " ": "stop"
};

const driveButtons = Array.from(document.querySelectorAll("[data-drive]"));

function setControlStatus(label, state = "ready") {
  if (!controlStatus) {
    return;
  }
  controlStatus.textContent = label;
  controlStatus.dataset.state = state;
}

function updateDriveSpeed() {
  driveSpeed = Math.max(minDriveSpeed, Math.min(maxDriveSpeed, Number(driveSpeed.toFixed(2))));
  if (driveSpeedValue) {
    driveSpeedValue.textContent = driveSpeed.toFixed(2);
  }
}

function commandFromKeys() {
  let linear_x = 0;
  let angular_z = 0;

  if (activeDriveKeys.has("forward")) {
    linear_x += driveSpeed;
  }
  if (activeDriveKeys.has("back")) {
    linear_x -= driveSpeed;
  }
  if (activeDriveKeys.has("left")) {
    angular_z += turnSpeed;
  }
  if (activeDriveKeys.has("right")) {
    angular_z -= turnSpeed;
  }

  return { linear_x, angular_z };
}

function reflectDriveButtons() {
  driveButtons.forEach((button) => {
    const action = button.dataset.drive;
    button.classList.toggle("active", activeDriveKeys.has(action));
  });
}

async function sendDriveCommand(command, force = false) {
  if (
    !force &&
    command.linear_x === lastCommand.linear_x &&
    command.angular_z === lastCommand.angular_z
  ) {
    return;
  }

  lastCommand = command;
  try {
    const response = await fetch("/api/control/cmd_vel", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(command)
    });
    if (!response.ok) {
      setControlStatus("Offline", "error");
      return;
    }
    setControlStatus(command.linear_x || command.angular_z ? "Driving" : "Hold", "ready");
  } catch {
    setControlStatus("Offline", "error");
  }
}

function sendCurrentDrive(force = false) {
  const command = commandFromKeys();
  reflectDriveButtons();
  sendDriveCommand(command, force);
}

function startDriveLoop() {
  if (driveTick) {
    return;
  }
  driveTick = setInterval(() => {
    if (activeDriveKeys.size > 0) {
      sendDriveCommand(commandFromKeys(), true);
    }
  }, driveCommandIntervalMs);
}

function stopDriveLoopIfIdle() {
  if (activeDriveKeys.size === 0 && driveTick) {
    clearInterval(driveTick);
    driveTick = null;
  }
}

function beginDrive(action) {
  if (action === "stop") {
    activeDriveKeys.clear();
    sendCurrentDrive(true);
    stopDriveLoopIfIdle();
    return;
  }
  activeDriveKeys.add(action);
  startDriveLoop();
  sendCurrentDrive(true);
}

function endDrive(action) {
  activeDriveKeys.delete(action);
  sendCurrentDrive(true);
  stopDriveLoopIfIdle();
}

function shouldIgnoreKeyboardEvent(event) {
  const tagName = event.target?.tagName;
  return tagName === "INPUT" || tagName === "TEXTAREA" || event.target?.isContentEditable;
}

document.addEventListener("keydown", (event) => {
  if (shouldIgnoreKeyboardEvent(event)) {
    return;
  }
  const action = keyToDrive[event.key.toLowerCase()];
  if (!action) {
    return;
  }
  event.preventDefault();
  beginDrive(action);
});

document.addEventListener("keyup", (event) => {
  const action = keyToDrive[event.key.toLowerCase()];
  if (!action || action === "stop") {
    return;
  }
  event.preventDefault();
  endDrive(action);
});

driveButtons.forEach((button) => {
  const action = button.dataset.drive;
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    beginDrive(action);
  });
  button.addEventListener("pointerup", () => endDrive(action));
  button.addEventListener("pointercancel", () => endDrive(action));
  button.addEventListener("pointerleave", () => endDrive(action));
});

driveSpeedUp?.addEventListener("click", () => {
  driveSpeed += 0.25;
  updateDriveSpeed();
});

driveSpeedDown?.addEventListener("click", () => {
  driveSpeed -= 0.25;
  updateDriveSpeed();
});

driveSlow?.addEventListener("click", () => {
  driveSpeed = 0.75;
  updateDriveSpeed();
});

driveFast?.addEventListener("click", () => {
  driveSpeed = 2.0;
  updateDriveSpeed();
});

driveStop?.addEventListener("click", () => {
  activeDriveKeys.clear();
  sendCurrentDrive(true);
  stopDriveLoopIfIdle();
});

window.addEventListener("blur", () => {
  activeDriveKeys.clear();
  sendCurrentDrive(true);
  stopDriveLoopIfIdle();
});

updateDriveSpeed();
