import * as THREE from "three";
import { PLYLoader } from "three/addons/loaders/PLYLoader.js";
import * as GaussianSplats3D from "@mkkellogg/gaussian-splats-3d";

const CLOUDS = {
  1: "../data/1.ply",
  2: "../data/2.ply",
};

const SPLAT_PATH = "../data/MIC_3DGS_aligned_preserved.ply";
const DEFAULT_POSES = [
  {
    id: "pose-1",
    cloud: 1,
    label: "1",
    name: "位姿 01",
    position: [1.513747, -1.437068, 1.179259],
  },
  {
    id: "pose-2",
    cloud: 2,
    label: "2",
    name: "位姿 02",
    position: [-0.098587, -1.663455, 1.24905],
  },
];

const cloudViewport = document.querySelector("#pointCloudViewport");
const appShell = document.querySelector("#appShell");
const splatViewport = document.querySelector("#splatViewport");
const poseOverlay = document.querySelector("#poseOverlay");
const cloudStatus = document.querySelector("#cloudStatus");
const splatStatus = document.querySelector("#splatStatus");
const expandSplatButton = document.querySelector("#expandSplat");
const cloudButtons = {
  1: document.querySelector("#loadCloud1"),
  2: document.querySelector("#loadCloud2"),
};

const pointScene = new THREE.Scene();
pointScene.background = new THREE.Color(0x15171b);

const pointCamera = new THREE.PerspectiveCamera(55, 1, 0.01, 5000);
pointCamera.position.set(2.2, -2.2, 1.4);

const pointRenderer = new THREE.WebGLRenderer({ antialias: false, powerPreference: "high-performance" });
pointRenderer.setPixelRatio(1);
cloudViewport.appendChild(pointRenderer.domElement);

pointScene.add(new THREE.HemisphereLight(0xffffff, 0x30343a, 1.4));

const splatScene = new THREE.Scene();
const splatCamera = new THREE.PerspectiveCamera(55, 1, 0.01, 5000);
const splatRenderer = new THREE.WebGLRenderer({ antialias: false, alpha: true, powerPreference: "high-performance" });
splatRenderer.setPixelRatio(1);
splatViewport.appendChild(splatRenderer.domElement);

let activeCloud = null;
let activePoints = null;
let activeGeometry = null;
let appMode = "splat";
let splatViewer = null;
let splatDropIn = null;
let splatLoaded = false;
let poseAnchors = DEFAULT_POSES;
let occlusionPoints = [];
let poseMarkerElements = [];
let splatSceneCenter = new THREE.Vector3(2.128, -0.751, 1.111);
let splatSceneExtent = 4.8;
let overviewPoints = null;
let overviewGeometry = null;
let overviewFitted = false;
let pointRenderRequested = true;
let lastMarkerUpdate = 0;
let pointDragMode = null;
let pointLastMouseX = 0;
let pointLastMouseY = 0;
let pointOrbitDistance = 5;
let splatDragMode = null;
let splatLastMouseX = 0;
let splatLastMouseY = 0;
let splatOrbitDistance = 5;

const projectionPoint = new THREE.Vector3();
const occlusionPoint = new THREE.Vector3();
const pointOrbitTarget = new THREE.Vector3();
const pointOrbitForward = new THREE.Vector3(0, 1, 0);
const pointOrbitUp = new THREE.Vector3(0, 0, 1);
const pointForward = new THREE.Vector3();
const pointRight = new THREE.Vector3();
const pointPan = new THREE.Vector3();
const splatOrbitTarget = new THREE.Vector3();
const splatOrbitForward = new THREE.Vector3(0, 0, -1);
const splatOrbitUp = new THREE.Vector3(0, -1, 0);
const splatForward = new THREE.Vector3();
const splatRight = new THREE.Vector3();
const splatPan = new THREE.Vector3();
const splatCalibrationViews = {
  "front-y": { forward: new THREE.Vector3(0, 1, 0), up: new THREE.Vector3(0, 0, 1) },
  "back-y": { forward: new THREE.Vector3(0, -1, 0), up: new THREE.Vector3(0, 0, 1) },
  "front-x": { forward: new THREE.Vector3(1, 0, 0), up: new THREE.Vector3(0, 0, 1) },
  "back-x": { forward: new THREE.Vector3(-1, 0, 0), up: new THREE.Vector3(0, 0, 1) },
  "front-z": { forward: new THREE.Vector3(0, 0, 1), up: new THREE.Vector3(0, 1, 0) },
  "back-z": { forward: new THREE.Vector3(0, 0, -1), up: new THREE.Vector3(0, -1, 0) },
};

function syncViewportSizes() {
  resizePointViewport();
  resizeSplatViewport();
  applySplatOrbitCamera();
  applyPointOrbitCamera();
}

function setAppMode(mode) {
  if (appMode === mode) {
    requestAnimationFrame(syncViewportSizes);
    return;
  }
  appMode = mode;
  appShell.classList.toggle("mode-splat", mode === "splat");
  appShell.classList.toggle("mode-detail", mode === "detail");
  pointDragMode = null;
  splatDragMode = null;
  requestAnimationFrame(syncViewportSizes);
}

function showSplatMode() {
  setAppMode("splat");
}

function showDetailMode() {
  setAppMode("detail");
}

function createSplatPoseMarkers() {
  poseOverlay.replaceChildren();
  poseMarkerElements = [];
  for (const pose of poseAnchors) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "splat-pose";
    button.dataset.cloud = String(pose.cloud);
    button.dataset.pose = pose.id;
    button.textContent = pose.label;
    button.title = `${pose.name} - 加载 ${pose.cloud}.ply`;
    button.addEventListener("click", (event) => {
      event.stopPropagation();
      loadCloud(pose.cloud);
    });
    poseOverlay.appendChild(button);
    poseMarkerElements.push({ pose, element: button });
  }
  setActivePose(activeCloud ?? 1);
}

function setActivePose(id) {
  document.querySelectorAll(".pose-button").forEach((button) => {
    button.classList.toggle("active", button.dataset.cloud === String(id));
  });
  document.querySelectorAll(".splat-pose").forEach((button) => {
    button.classList.toggle("active", button.dataset.cloud === String(id));
  });
}

function relaxOrbitLikeControls(controls) {
  if (!controls) return;
  if ("enabled" in controls) controls.enabled = false;
  if ("enableRotate" in controls) controls.enableRotate = false;
  if ("enableZoom" in controls) controls.enableZoom = false;
  if ("enablePan" in controls) controls.enablePan = false;
  if ("minPolarAngle" in controls) controls.minPolarAngle = -Infinity;
  if ("maxPolarAngle" in controls) controls.maxPolarAngle = Infinity;
  if ("minAzimuthAngle" in controls) controls.minAzimuthAngle = -Infinity;
  if ("maxAzimuthAngle" in controls) controls.maxAzimuthAngle = Infinity;
  if ("screenSpacePanning" in controls) controls.screenSpacePanning = true;
  if ("enableDamping" in controls) controls.enableDamping = false;
}

function updateSplatSceneBounds() {
  const points = occlusionPoints.length ? occlusionPoints : poseAnchors.map((pose) => pose.position);
  if (!points.length) return;

  const min = new THREE.Vector3(Infinity, Infinity, Infinity);
  const max = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
  for (const point of points) {
    min.min(occlusionPoint.fromArray(point));
    max.max(occlusionPoint);
  }
  splatSceneCenter.copy(min).add(max).multiplyScalar(0.5);
  splatSceneExtent = Math.max(max.x - min.x, max.y - min.y, max.z - min.z, 1);
}

function setSplatCameraFromView(view) {
  const camera = getSplatCamera();
  if (!camera) return;
  splatOrbitTarget.copy(splatSceneCenter);
  splatOrbitDistance = splatSceneExtent * 1.25;
  splatOrbitForward.copy(view.forward).normalize();
  splatOrbitUp.copy(view.up).normalize();
  applySplatOrbitCamera();
}

function applySplatOrbitCamera() {
  const camera = getSplatCamera();
  if (!camera) return;
  camera.position.copy(splatOrbitTarget).addScaledVector(splatOrbitForward, -splatOrbitDistance);
  camera.up.copy(splatOrbitUp);
  camera.lookAt(splatOrbitTarget);
  camera.updateMatrixWorld?.();
}

function resetSplatCameraToZUpOverview() {
  setSplatCameraFromView(splatCalibrationViews["front-y"]);
}

function forceSplatFrontView() {
  resetSplatCameraToZUpOverview();
  splatStatus.textContent = "高斯模型已加载 · +Y正面视角";
}

function applyPointOrbitCamera() {
  pointCamera.position.copy(pointOrbitTarget).addScaledVector(pointOrbitForward, -pointOrbitDistance);
  pointCamera.up.copy(pointOrbitUp);
  pointCamera.lookAt(pointOrbitTarget);
  pointCamera.updateMatrixWorld?.();
  requestPointRender();
}

function rotatePointOrbit(deltaX, deltaY) {
  pointOrbitForward.applyAxisAngle(pointOrbitUp, -deltaX * 0.006).normalize();
  pointRight.crossVectors(pointOrbitForward, pointOrbitUp).normalize();
  const candidate = pointOrbitForward.clone().applyAxisAngle(pointRight, -deltaY * 0.006).normalize();
  if (Math.abs(candidate.dot(pointOrbitUp)) < 0.96) {
    pointOrbitForward.copy(candidate);
  }
  applyPointOrbitCamera();
}

function panPointOrbit(deltaX, deltaY) {
  pointForward.copy(pointOrbitTarget).sub(pointCamera.position).normalize();
  pointRight.crossVectors(pointForward, pointOrbitUp).normalize();
  const panScale = pointOrbitDistance * 0.0014;
  pointPan
    .copy(pointRight)
    .multiplyScalar(-deltaX * panScale)
    .addScaledVector(pointOrbitUp, deltaY * panScale);
  pointOrbitTarget.add(pointPan);
  applyPointOrbitCamera();
}

function initPointNavigationControls() {
  cloudViewport.tabIndex = 0;
  cloudViewport.addEventListener("contextmenu", (event) => event.preventDefault());

  cloudViewport.addEventListener("mousedown", (event) => {
    event.preventDefault();
    pointDragMode = event.button === 2 ? "pan" : "rotate";
    pointLastMouseX = event.clientX;
    pointLastMouseY = event.clientY;
    cloudViewport.focus();
  });

  window.addEventListener("mouseup", () => {
    pointDragMode = null;
  });

  cloudViewport.addEventListener("mousemove", (event) => {
    if (!pointDragMode) return;
    event.preventDefault();
    const deltaX = event.clientX - pointLastMouseX;
    const deltaY = event.clientY - pointLastMouseY;
    pointLastMouseX = event.clientX;
    pointLastMouseY = event.clientY;
    if (pointDragMode === "pan") panPointOrbit(deltaX, deltaY);
    else rotatePointOrbit(deltaX, deltaY);
  });

  cloudViewport.addEventListener("wheel", (event) => {
    event.preventDefault();
    const zoomFactor = Math.exp(event.deltaY * 0.001);
    pointOrbitDistance = THREE.MathUtils.clamp(pointOrbitDistance * zoomFactor, 0.02, 5000);
    applyPointOrbitCamera();
  }, { passive: false });
}

function rotateSplatOrbit(deltaX, deltaY) {
  splatOrbitForward.applyAxisAngle(splatOrbitUp, -deltaX * 0.006).normalize();
  splatRight.crossVectors(splatOrbitForward, splatOrbitUp).normalize();
  const candidate = splatOrbitForward.clone().applyAxisAngle(splatRight, -deltaY * 0.006).normalize();
  if (Math.abs(candidate.dot(splatOrbitUp)) < 0.96) {
    splatOrbitForward.copy(candidate);
  }
  applySplatOrbitCamera();
}

function panSplatOrbit(deltaX, deltaY) {
  const camera = getSplatCamera();
  if (!camera) return;
  splatForward.copy(splatOrbitTarget).sub(camera.position).normalize();
  splatRight.crossVectors(splatForward, splatOrbitUp).normalize();
  const panScale = splatOrbitDistance * 0.0014;
  splatPan
    .copy(splatRight)
    .multiplyScalar(-deltaX * panScale)
    .addScaledVector(splatOrbitUp, deltaY * panScale);
  splatOrbitTarget.add(splatPan);
  applySplatOrbitCamera();
}

function initSplatNavigationControls() {
  splatViewport.tabIndex = 0;
  splatViewport.addEventListener("contextmenu", (event) => event.preventDefault());

  splatViewport.addEventListener("mousedown", (event) => {
    event.preventDefault();
    event.stopImmediatePropagation();
    splatDragMode = event.button === 2 ? "pan" : "rotate";
    splatLastMouseX = event.clientX;
    splatLastMouseY = event.clientY;
    splatViewport.focus();
  }, true);

  window.addEventListener("mouseup", () => {
    splatDragMode = null;
  });

  splatViewport.addEventListener("mousemove", (event) => {
    if (!splatDragMode) return;
    event.preventDefault();
    event.stopImmediatePropagation();
    const deltaX = event.clientX - splatLastMouseX;
    const deltaY = event.clientY - splatLastMouseY;
    splatLastMouseX = event.clientX;
    splatLastMouseY = event.clientY;
    if (splatDragMode === "pan") panSplatOrbit(deltaX, deltaY);
    else rotateSplatOrbit(deltaX, deltaY);
  }, true);

  splatViewport.addEventListener("wheel", (event) => {
    event.preventDefault();
    event.stopImmediatePropagation();
    const zoomFactor = Math.exp(event.deltaY * 0.001);
    splatOrbitDistance = THREE.MathUtils.clamp(splatOrbitDistance * zoomFactor, 0.05, splatSceneExtent * 8);
    applySplatOrbitCamera();
  }, { capture: true, passive: false });
}

function getSplatCamera() {
  return splatCamera;
}

function isAnchorOccluded(anchorNdc, camera, width, height) {
  if (!occlusionPoints.length) return false;

  const radiusX = 32 / Math.max(width, 1);
  const radiusY = 32 / Math.max(height, 1);
  for (const point of occlusionPoints) {
    occlusionPoint.fromArray(point).project(camera);
    if (occlusionPoint.z < -1 || occlusionPoint.z > 1) continue;
    if (Math.abs(occlusionPoint.x - anchorNdc.x) > radiusX) continue;
    if (Math.abs(occlusionPoint.y - anchorNdc.y) > radiusY) continue;
    if (occlusionPoint.z < anchorNdc.z - 0.018) return true;
  }
  return false;
}

function updateSplatPoseMarkers() {
  const now = performance.now();
  if (now - lastMarkerUpdate < 90) {
    requestAnimationFrame(updateSplatPoseMarkers);
    return;
  }
  lastMarkerUpdate = now;

  const camera = getSplatCamera();
  const width = splatViewport.clientWidth;
  const height = splatViewport.clientHeight;
  if (camera && width && height) {
    camera.updateMatrixWorld?.();
    camera.updateProjectionMatrix?.();
    for (const { pose, element } of poseMarkerElements) {
      projectionPoint.fromArray(pose.position).project(camera);
      const outside =
        projectionPoint.x < -1 ||
        projectionPoint.x > 1 ||
        projectionPoint.y < -1 ||
        projectionPoint.y > 1 ||
        projectionPoint.z < -1 ||
        projectionPoint.z > 1;
      const occluded = !outside && isAnchorOccluded(projectionPoint, camera, width, height);
      element.classList.toggle("hidden", outside || occluded);
      element.style.left = `${((projectionPoint.x + 1) * 0.5) * width}px`;
      element.style.top = `${((-projectionPoint.y + 1) * 0.5) * height}px`;
    }
  }
  requestAnimationFrame(updateSplatPoseMarkers);
}

async function loadPoseAssets() {
  try {
    const response = await fetch("./pose_assets.json");
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const assets = await response.json();
    if (Array.isArray(assets.poses) && assets.poses.length) poseAnchors = assets.poses;
    if (Array.isArray(assets.occlusionPoints)) {
      const stride = Math.max(1, Math.ceil(assets.occlusionPoints.length / 8000));
      occlusionPoints = assets.occlusionPoints.filter((_, index) => index % stride === 0);
    }
  } catch (error) {
    console.warn("Using fallback pose anchors", error);
  }
  updateSplatSceneBounds();
  createOverviewCloud();
  createSplatPoseMarkers();
}

function createOverviewCloud() {
  if (!occlusionPoints.length || overviewPoints) return;

  overviewGeometry = new THREE.BufferGeometry();
  const positions = new Float32Array(occlusionPoints.length * 3);
  for (let index = 0; index < occlusionPoints.length; index += 1) {
    positions[index * 3] = occlusionPoints[index][0];
    positions[index * 3 + 1] = occlusionPoints[index][1];
    positions[index * 3 + 2] = occlusionPoints[index][2];
  }
  overviewGeometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  overviewGeometry.computeBoundingSphere();
  overviewGeometry.computeBoundingBox();

  overviewPoints = new THREE.Points(
    overviewGeometry,
    new THREE.PointsMaterial({
      color: 0x7f8790,
      opacity: 0.22,
      transparent: true,
      depthWrite: false,
      size: 0.012,
      sizeAttenuation: true,
    }),
  );
  overviewPoints.renderOrder = -1;
  pointScene.add(overviewPoints);

  if (!overviewFitted) {
    fitObject(overviewPoints);
    overviewFitted = true;
  }
}

function resizePointViewport() {
  const { clientWidth, clientHeight } = cloudViewport;
  pointRenderer.setSize(clientWidth, clientHeight, false);
  pointCamera.aspect = clientWidth / Math.max(clientHeight, 1);
  pointCamera.updateProjectionMatrix();
  requestPointRender();
}

function resizeSplatViewport() {
  const { clientWidth, clientHeight } = splatViewport;
  splatRenderer.setSize(clientWidth, clientHeight, false);
  splatCamera.aspect = clientWidth / Math.max(clientHeight, 1);
  splatCamera.updateProjectionMatrix();
}

function requestPointRender() {
  pointRenderRequested = true;
}

function fitObject(object) {
  const box = new THREE.Box3().setFromObject(object);
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  if (!Number.isFinite(sphere.radius) || sphere.radius <= 0) return;

  const view = splatCalibrationViews["front-y"];
  const forward = view.forward.clone().normalize();
  const up = view.up.clone().normalize();
  const distance = sphere.radius / Math.sin(THREE.MathUtils.degToRad(pointCamera.fov * 0.5));
  pointOrbitTarget.copy(sphere.center);
  pointOrbitForward.copy(forward);
  pointOrbitUp.copy(up);
  pointOrbitDistance = distance * 1.15;
  applyPointOrbitCamera();
  pointCamera.near = Math.max(distance / 1000, 0.001);
  pointCamera.far = distance * 12;
  pointCamera.updateProjectionMatrix();
  requestPointRender();
}

function disposeActiveCloud() {
  if (activePoints) {
    pointScene.remove(activePoints);
    activePoints.material.dispose();
    activePoints = null;
  }
  if (activeGeometry) {
    activeGeometry.dispose();
    activeGeometry = null;
  }
}

async function loadCloud(id) {
  showDetailMode();
  if (activeCloud === id) return;
  activeCloud = id;
  cloudStatus.textContent = `正在加载 ${id}.ply`;
  Object.entries(cloudButtons).forEach(([key, button]) => button.classList.toggle("active", key === String(id)));
  setActivePose(id);

  const loader = new PLYLoader();
  loader.load(
    CLOUDS[id],
    (geometry) => {
      disposeActiveCloud();
      geometry.computeBoundingSphere();
      geometry.computeBoundingBox();
      activeGeometry = geometry;

      const material = new THREE.PointsMaterial({
        size: 0.006,
        vertexColors: geometry.hasAttribute("color"),
        sizeAttenuation: true,
      });
      activePoints = new THREE.Points(geometry, material);
      pointScene.add(activePoints);
      if (!overviewFitted) {
        fitObject(activePoints);
        overviewFitted = true;
      }
      cloudStatus.textContent = `${id}.ply 已加载：${geometry.getAttribute("position").count.toLocaleString()} 点`;
      requestPointRender();
    },
    (event) => {
      if (event.total) {
        cloudStatus.textContent = `正在加载 ${id}.ply：${Math.round((event.loaded / event.total) * 100)}%`;
      }
    },
    (error) => {
      console.error(error);
      cloudStatus.textContent = `${id}.ply 加载失败，请确认通过本地服务器打开`;
    },
  );
}

function animatePointCloud() {
  if (pointRenderRequested) {
    pointRenderer.render(pointScene, pointCamera);
    pointRenderRequested = false;
  }
  requestAnimationFrame(animatePointCloud);
}

function animateSplatScene() {
  splatRenderer.render(splatScene, splatCamera);
  requestAnimationFrame(animateSplatScene);
}

function initSplatPreview() {
  splatDropIn = new GaussianSplats3D.DropInViewer({
    gpuAcceleratedSort: false,
    sharedMemoryForWorkers: false,
  });
  splatViewer = splatDropIn.viewer;
  window.splatViewer = splatViewer;
  window.splatCamera = splatCamera;
  splatScene.add(splatDropIn);
  resizeSplatViewport();
  resetSplatCameraToZUpOverview();

  splatDropIn
    .addSplatScene(SPLAT_PATH, {
      progressiveLoad: true,
      showLoadingUI: false,
    })
    .then(() => {
      splatLoaded = true;
      forceSplatFrontView();
    })
    .catch((error) => {
      console.error(error);
      splatStatus.textContent = "高斯模型加载失败";
    });
}

window.addEventListener("resize", resizePointViewport);
window.addEventListener("resize", resizeSplatViewport);
document.querySelectorAll("[data-splat-view]").forEach((button) => {
  button.addEventListener("click", (event) => {
    event.stopPropagation();
    const view = splatCalibrationViews[button.dataset.splatView];
    if (!view) return;
    setSplatCameraFromView(view);
    splatStatus.textContent = `高斯模型已加载 · 视角 ${button.textContent}`;
  });
});
document.querySelector("#fitCloud").addEventListener("click", () => {
  if (activePoints) fitObject(activePoints);
});
expandSplatButton.addEventListener("click", (event) => {
  event.preventDefault();
  event.stopPropagation();
  showSplatMode();
});
cloudButtons[1].addEventListener("click", () => loadCloud(1));
cloudButtons[2].addEventListener("click", () => loadCloud(2));
document.querySelectorAll(".pose-button").forEach((button) => {
  button.addEventListener("click", () => loadCloud(Number(button.dataset.cloud)));
});

resizePointViewport();
resizeSplatViewport();
loadPoseAssets().then(() => {
  initSplatPreview();
});
animatePointCloud();
animateSplatScene();
updateSplatPoseMarkers();
initPointNavigationControls();
initSplatNavigationControls();
