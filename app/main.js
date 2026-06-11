import * as THREE from "three";
import { TrackballControls } from "three/addons/controls/TrackballControls.js";
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
const splatViewport = document.querySelector("#splatViewport");
const poseOverlay = document.querySelector("#poseOverlay");
const cloudStatus = document.querySelector("#cloudStatus");
const splatStatus = document.querySelector("#splatStatus");
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

const pointControls = new TrackballControls(pointCamera, pointRenderer.domElement);
pointControls.noRotate = false;
pointControls.noZoom = false;
pointControls.noPan = false;
pointControls.staticMoving = true;
pointControls.dynamicDampingFactor = 0.08;
pointControls.rotateSpeed = 3.2;
pointControls.zoomSpeed = 1.2;
pointControls.panSpeed = 0.65;

pointScene.add(new THREE.HemisphereLight(0xffffff, 0x30343a, 1.4));

let activeCloud = null;
let activePoints = null;
let activeGeometry = null;
let splatViewer = null;
let poseAnchors = DEFAULT_POSES;
let occlusionPoints = [];
let poseMarkerElements = [];
let overviewPoints = null;
let overviewGeometry = null;
let overviewFitted = false;
let pointRenderRequested = true;
let lastMarkerUpdate = 0;

const projectionPoint = new THREE.Vector3();
const occlusionPoint = new THREE.Vector3();

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
  if ("minPolarAngle" in controls) controls.minPolarAngle = -Infinity;
  if ("maxPolarAngle" in controls) controls.maxPolarAngle = Infinity;
  if ("minAzimuthAngle" in controls) controls.minAzimuthAngle = -Infinity;
  if ("maxAzimuthAngle" in controls) controls.maxAzimuthAngle = Infinity;
  if ("screenSpacePanning" in controls) controls.screenSpacePanning = true;
  if ("enableDamping" in controls) controls.enableDamping = false;
}

function getSplatCamera() {
  if (!splatViewer) return null;
  return (
    splatViewer.camera ||
    splatViewer.perspectiveCamera ||
    splatViewer.threeCamera ||
    splatViewer.sceneCamera ||
    splatViewer.controls?.object ||
    null
  );
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
  pointControls.handleResize?.();
  requestPointRender();
}

function requestPointRender() {
  pointRenderRequested = true;
}

function fitObject(object) {
  const box = new THREE.Box3().setFromObject(object);
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  if (!Number.isFinite(sphere.radius) || sphere.radius <= 0) return;

  pointControls.target.copy(sphere.center);
  const distance = sphere.radius / Math.sin(THREE.MathUtils.degToRad(pointCamera.fov * 0.5));
  pointCamera.position.copy(sphere.center).add(new THREE.Vector3(distance * 0.55, -distance * 0.9, distance * 0.45));
  pointCamera.near = Math.max(distance / 1000, 0.001);
  pointCamera.far = distance * 12;
  pointCamera.updateProjectionMatrix();
  pointControls.update();
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
  pointControls.update();
  if (pointRenderRequested) {
    pointRenderer.render(pointScene, pointCamera);
    pointRenderRequested = false;
  }
  requestAnimationFrame(animatePointCloud);
}

function initSplatPreview() {
  const viewer = new GaussianSplats3D.Viewer({
    rootElement: splatViewport,
    cameraUp: [0, 0, 1],
    initialCameraPosition: [2.2, -2.4, 1.4],
    initialCameraLookAt: [0, 0, 0],
    sharedMemoryForWorkers: false,
  });
  splatViewer = viewer;
  window.splatViewer = viewer;
  relaxOrbitLikeControls(viewer.controls);

  viewer
    .addSplatScene(SPLAT_PATH, {
      progressiveLoad: true,
      showLoadingUI: false,
    })
    .then(() => {
      splatStatus.textContent = "高斯模型已加载";
      relaxOrbitLikeControls(viewer.controls);
      viewer.start();
    })
    .catch((error) => {
      console.error(error);
      splatStatus.textContent = "高斯模型加载失败";
    });
}

window.addEventListener("resize", resizePointViewport);
pointControls.addEventListener("change", requestPointRender);
document.querySelector("#fitCloud").addEventListener("click", () => {
  if (activePoints) fitObject(activePoints);
});
cloudButtons[1].addEventListener("click", () => loadCloud(1));
cloudButtons[2].addEventListener("click", () => loadCloud(2));
document.querySelectorAll(".pose-button").forEach((button) => {
  button.addEventListener("click", () => loadCloud(Number(button.dataset.cloud)));
});

resizePointViewport();
loadPoseAssets();
animatePointCloud();
updateSplatPoseMarkers();
initSplatPreview();
loadCloud(1);
