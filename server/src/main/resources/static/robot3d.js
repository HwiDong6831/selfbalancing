// 대시보드 3D 자세 뷰. app.js 와는 window.robot3d 로만 주고받는다.
import * as THREE from 'three';
import { GLTFLoader } from './vendor/loaders/GLTFLoader.js';
import { OrbitControls } from './vendor/controls/OrbitControls.js';

const PIVOT_Y = 100.3;   // 회전 중심(꼬리 끝)까지의 높이 [mm]
const HALF_Z  = 23.0;    // 몸통 두께의 절반 [mm]
const WHEEL_Z = 42.0;    // 몸통 안에서의 휠 깊이 [mm]
const TILT_SIGN  = 1;
const WHEEL_SIGN = -1;

const AXIS_LEN = 230;    // 축 길이 [mm]
const ARC_R    = 165;    // 각도 호의 반지름 [mm]
const ARC_N    = 32;     // 호를 이루는 선분 수

let tilt = 0, spin = 0;
let started = false;

window.robot3d = {
  set(angleDeg, encDeg) {
    tilt = angleDeg || 0;
    spin = encDeg || 0;
  },
};

// 씬을 짜고 렌더 루프를 시작한다.
function init(el) {
  const scene    = new THREE.Scene();
  const camera   = new THREE.PerspectiveCamera(40, 1, 1, 4000);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });

  camera.position.set(300, 210, 420);
  renderer.setPixelRatio(window.devicePixelRatio);
  el.appendChild(renderer.domElement);

  scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x1a1f26, 2.2));
  const sun = new THREE.DirectionalLight(0xffffff, 2.0);
  sun.position.set(220, 420, 320);
  scene.add(sun);
  scene.add(new THREE.GridHelper(800, 16, 0x3a424d, 0x252b33));

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 90, 0);
  controls.enableDamping = true;

  const body = new THREE.Group();
  scene.add(body);

  const skin = (color, metalness) =>
    new THREE.MeshStandardMaterial({ color, metalness, roughness: 0.5 });

  let wheel = null;
  const loader = new GLTFLoader();
  const place = (root, z, material) => {
    root.traverse((o) => { if (o.isMesh) o.material = material; });
    root.position.set(0, PIVOT_Y, z - HALF_Z);
    body.add(root);
    return root;
  };
  loader.load('model/body.glb',  (g) => place(g.scene, 0, skin(0x8b949e, 0.25)));
  loader.load('model/wheel.glb', (g) => { wheel = place(g.scene, WHEEL_Z, skin(0x58a6ff, 0.4)); });

  const line = (color, parent) => {
    const g = new THREE.BufferGeometry().setFromPoints(
      [new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, AXIS_LEN, 0)]);
    parent.add(new THREE.Line(g, new THREE.LineBasicMaterial({ color })));
  };
  line(0x6e7681, scene);
  line(0xd29922, body);

  const arcPos = new THREE.BufferAttribute(new Float32Array((ARC_N + 1) * 3), 3);
  const arcGeom = new THREE.BufferGeometry();
  arcGeom.setAttribute('position', arcPos);
  scene.add(new THREE.Line(arcGeom, new THREE.LineBasicMaterial({ color: 0xd29922 })));

  const label = document.createElement('div');
  label.className = 'view3d-label';
  el.appendChild(label);

  const fit = () => {
    const w = el.clientWidth, h = el.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  new ResizeObserver(fit).observe(el);
  fit();

  const DEG = Math.PI / 180;
  const at = new THREE.Vector3();
  (function tick() {
    requestAnimationFrame(tick);
    const rad = TILT_SIGN * tilt * DEG;
    body.rotation.z = rad;
    if (wheel) wheel.rotation.z = WHEEL_SIGN * spin * DEG;

    for (let i = 0; i <= ARC_N; i++) {
      const a = (i / ARC_N) * rad;
      arcPos.setXYZ(i, -Math.sin(a) * ARC_R, Math.cos(a) * ARC_R, 0);
    }
    arcPos.needsUpdate = true;

    const half = rad / 2, r = ARC_R * 1.15;
    at.set(-Math.sin(half) * r, Math.cos(half) * r, 0).project(camera);
    label.style.left = `${(at.x * 0.5 + 0.5) * el.clientWidth}px`;
    label.style.top  = `${(-at.y * 0.5 + 0.5) * el.clientHeight}px`;
    label.textContent = `${tilt.toFixed(1)}°`;

    controls.update();
    renderer.render(scene, camera);
  })();
}

const panel = document.getElementById('view3d-panel');
const start = () => {
  if (started || !panel.open) return;
  started = true;
  init(document.getElementById('view3d'));
};
panel.addEventListener('toggle', start);
start();
