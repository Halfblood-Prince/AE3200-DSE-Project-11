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
let cameraPeerConnection = null;
let cameraReconnectTimer = null;
let cameraFrameCount = 0;
let cameraFrameWindowStarted = performance.now();
let cameraConnectAttempt = 0;
let cameraFrameCallbackId = null;

function setCameraStatus(label) {
  if (!cameraStatus) {
    return;
  }
  cameraStatus.textContent = label;
}

function updateCameraFrameRate() {
  cameraFrameCount += 1;
  const now = performance.now();
  const elapsed = now - cameraFrameWindowStarted;
  if (elapsed < 1000) {
    return;
  }

  const fps = Math.round((cameraFrameCount * 1000) / elapsed);
  setCameraStatus(`${Math.min(fps, 60)} fps  H.264`);
  cameraFrameCount = 0;
  cameraFrameWindowStarted = now;
}

function startCameraFrameMonitor() {
  if (!cameraFeed?.requestVideoFrameCallback) {
    setCameraStatus("H.264");
    return;
  }

  if (cameraFrameCallbackId !== null && cameraFeed.cancelVideoFrameCallback) {
    cameraFeed.cancelVideoFrameCallback(cameraFrameCallbackId);
  }

  const onFrame = () => {
    updateCameraFrameRate();
    if (cameraFeed.srcObject) {
      cameraFrameCallbackId = cameraFeed.requestVideoFrameCallback(onFrame);
    }
  };
  cameraFrameCallbackId = cameraFeed.requestVideoFrameCallback(onFrame);
}

function closeCameraFeed() {
  if (cameraFrameCallbackId !== null && cameraFeed?.cancelVideoFrameCallback) {
    cameraFeed.cancelVideoFrameCallback(cameraFrameCallbackId);
    cameraFrameCallbackId = null;
  }

  if (cameraFeed) {
    cameraFeed.pause?.();
    cameraFeed.srcObject = null;
    cameraFeed.dataset.live = "false";
  }

  if (cameraPeerConnection) {
    cameraPeerConnection.ontrack = null;
    cameraPeerConnection.onconnectionstatechange = null;
    cameraPeerConnection.oniceconnectionstatechange = null;
    cameraPeerConnection.close();
    cameraPeerConnection = null;
  }
}

function scheduleCameraReconnect(delay = 1500) {
  clearTimeout(cameraReconnectTimer);
  if (cameraFeed) {
    cameraFeed.dataset.live = "false";
  }
  setCameraStatus("Waiting");
  cameraReconnectTimer = setTimeout(connectCameraFeed, delay);
}

function waitForIceGatheringComplete(peerConnection, timeoutMs = 8000) {
  if (peerConnection.iceGatheringState === "complete") {
    return Promise.resolve();
  }

  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      peerConnection.removeEventListener("icegatheringstatechange", onStateChange);
      reject(new Error("Timed out gathering local ICE candidates."));
    }, timeoutMs);

    function onStateChange() {
      if (peerConnection.iceGatheringState !== "complete") {
        return;
      }
      clearTimeout(timeout);
      peerConnection.removeEventListener("icegatheringstatechange", onStateChange);
      resolve();
    }

    peerConnection.addEventListener("icegatheringstatechange", onStateChange);
  });
}

function cameraIceServerConfig(iceServers) {
  return {
    iceServers: iceServers.map((server) => ({ urls: server }))
  };
}

async function connectCameraFeed() {
  if (!cameraFeed) {
    return;
  }

  clearTimeout(cameraReconnectTimer);
  closeCameraFeed();

  setCameraStatus("Connecting");
  const attempt = ++cameraConnectAttempt;

  try {
    if (!window.RTCPeerConnection) {
      throw new Error("WebRTC is not supported by this browser.");
    }

    const configResponse = await fetch("/api/camera/webrtc/config", { cache: "no-store" });
    if (!configResponse.ok) {
      throw new Error("WebRTC camera config is unavailable.");
    }
    const config = await configResponse.json();

    if (attempt !== cameraConnectAttempt) {
      return;
    }

    const peerConnection = new RTCPeerConnection(
      cameraIceServerConfig(config.iceServers || [])
    );

    cameraPeerConnection = peerConnection;
    peerConnection.addTransceiver("video", { direction: "recvonly" });

    peerConnection.ontrack = (event) => {
      const [stream] = event.streams;
      cameraFeed.srcObject = stream || new MediaStream([event.track]);
      cameraFeed.dataset.live = "true";
      setCameraStatus("H.264");
      cameraFrameCount = 0;
      cameraFrameWindowStarted = performance.now();
      startCameraFrameMonitor();
      cameraFeed.play?.().catch(() => {
        setCameraStatus("Waiting");
      });
    };

    peerConnection.onconnectionstatechange = () => {
      if (["closed", "disconnected", "failed"].includes(peerConnection.connectionState)) {
        if (cameraPeerConnection === peerConnection) {
          scheduleCameraReconnect();
        }
      }
    };

    peerConnection.oniceconnectionstatechange = () => {
      if (["closed", "disconnected", "failed"].includes(peerConnection.iceConnectionState)) {
        if (cameraPeerConnection === peerConnection) {
          scheduleCameraReconnect();
        }
      }
    };

    const offer = await peerConnection.createOffer();
    await peerConnection.setLocalDescription(offer);
    await waitForIceGatheringComplete(peerConnection);

    if (attempt !== cameraConnectAttempt || cameraPeerConnection !== peerConnection) {
      return;
    }

    const answerResponse = await fetch("/api/camera/webrtc/offer", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(peerConnection.localDescription)
    });
    if (!answerResponse.ok) {
      throw new Error("WebRTC camera answer is unavailable.");
    }

    const answer = await answerResponse.json();
    await peerConnection.setRemoteDescription(answer);
  } catch {
    if (attempt === cameraConnectAttempt) {
      closeCameraFeed();
      scheduleCameraReconnect();
    }
  }
}

connectCameraFeed();

window.addEventListener("beforeunload", () => {
  clearTimeout(cameraReconnectTimer);
  closeCameraFeed();
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
