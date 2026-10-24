import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment.js";
import type { Estimate, Snapshot, Visuals } from "./api";

// The Three.js world uses the backend's frame directly: +Z up, meters.
// MuJoCo quaternions [w,x,y,z] are converted explicitly to THREE.Quaternion(x,y,z,w).
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

export function quatWXYZ(q: number[]): THREE.Quaternion {
  return new THREE.Quaternion(q[1], q[2], q[3], q[0]);
}

/** Body name of a draggable item: "part_<id>" or "dest_<id>". */
export type DragTarget = string;

export interface SceneViewCallbacks {
  onDragEnd: (target: DragTarget, x: number, y: number) => void;
  onSelect: (target: DragTarget) => void;
}

export interface Marker { body: string; text: string; color: string }

export class SceneView {
  renderer: THREE.WebGLRenderer;
  scene = new THREE.Scene();
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  bodies = new Map<string, THREE.Group>();
  private content = new THREE.Group();
  private raf = 0;
  private ro: ResizeObserver;
  private estimates = new THREE.Group();
  private markers = new THREE.Group();
  private workspace = new THREE.Group();
  private frustum: THREE.Group | null = null;
  private selection: THREE.LineSegments;
  private raycaster = new THREE.Raycaster();
  private dragging: DragTarget | null = null;
  private dragOffset = new THREE.Vector3();
  private dragPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -0.4);
  private hold: { target: DragTarget; until: number } | null = null;
  private sizes = new Map<string, number[]>();     // body -> full size (parts: xyz, dests: xy)
  private draggable = new Set<string>();
  private proxies: THREE.Object3D[] = [];
  private unitEdges = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
  showProxies = false;
  editable = false;
  selected: DragTarget | null = null;
  tableTop = 0.4;
  geometryRev = -1;

  constructor(private host: HTMLElement, private cb: SceneViewCallbacks) {
    const r = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
    r.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    r.outputColorSpace = THREE.SRGBColorSpace;
    r.toneMapping = THREE.ACESFilmicToneMapping;
    r.toneMappingExposure = 1.0;
    r.shadowMap.enabled = true;
    r.shadowMap.type = THREE.PCFSoftShadowMap;
    host.appendChild(r.domElement);
    this.renderer = r;

    this.scene.background = new THREE.Color("#1a1c1e");
    this.scene.fog = new THREE.Fog("#1a1c1e", 4, 9);
    const pmrem = new THREE.PMREMGenerator(r);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    this.scene.environmentIntensity = 0.55;

    this.camera = new THREE.PerspectiveCamera(36, 1, 0.02, 30);
    this.camera.position.set(2.05, -1.6, 1.62);
    this.controls = new OrbitControls(this.camera, r.domElement);
    this.controls.target.set(0.36, 0, 0.56);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.controls.minDistance = 0.5;
    this.controls.maxDistance = 5;

    this.scene.add(new THREE.HemisphereLight("#e9eef2", "#2a2622", 0.7));
    const key = new THREE.DirectionalLight("#fff6ea", 2.3);
    key.position.set(1.6, -1.2, 2.6);
    key.target.position.set(0.4, 0, 0.4);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    const s = key.shadow.camera;
    s.left = -1.1; s.right = 1.1; s.top = 1.1; s.bottom = -1.1; s.near = 0.5; s.far = 6;
    key.shadow.bias = -0.0004;
    key.shadow.normalBias = 0.02;
    this.scene.add(key, key.target);
    const rim = new THREE.DirectionalLight("#9fd8e0", 0.6);
    rim.position.set(-1.5, 1.4, 1.6);
    this.scene.add(rim);

    this.selection = new THREE.LineSegments(this.unitEdges, new THREE.LineBasicMaterial({ color: "#ffd089", transparent: true, opacity: 0.9 }));
    this.selection.visible = false;
    this.scene.add(this.content, this.estimates, this.markers, this.selection, this.workspace);
    const grid = new THREE.GridHelper(8, 32, "#34383b", "#2a2d30");
    grid.rotateX(Math.PI / 2);
    grid.position.z = 0.0005;
    this.scene.add(grid);

    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(host);
    this.resize();
    r.domElement.addEventListener("pointerdown", this.onDown);
    r.domElement.addEventListener("pointermove", this.onMove);
    window.addEventListener("pointerup", this.onUp);
    const loop = () => {
      this.raf = requestAnimationFrame(loop);
      this.controls.update();
      r.render(this.scene, this.camera);
    };
    loop();
  }

  dispose() {
    cancelAnimationFrame(this.raf);
    this.ro.disconnect();
    window.removeEventListener("pointerup", this.onUp);
    this.renderer.domElement.removeEventListener("pointerdown", this.onDown);
    this.renderer.domElement.removeEventListener("pointermove", this.onMove);
    disposeTree(this.scene);
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  resize() {
    const w = this.host.clientWidth || 1, h = this.host.clientHeight || 1;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  // Build meshes from the backend's compiled MuJoCo visual geoms (frames match mj geom_pos/quat).
  // Called again on every structural scene change; the previous content is disposed first.
  build(vis: Visuals, blob: ArrayBuffer) {
    disposeTree(this.content);
    this.content.clear();
    this.bodies.clear();
    this.proxies = [];
    this.sizes.clear();
    this.draggable.clear();
    this.tableTop = vis.table.top_z;
    this.dragPlane.constant = -this.tableTop;
    for (const p of vis.parts) { this.sizes.set(p.body, p.size_m); this.draggable.add(p.body); }
    for (const d of vis.destinations) this.draggable.add(d.body);
    const geoCache = new Map<number, THREE.BufferGeometry>();
    for (const g of vis.geoms) {
      let group = this.bodies.get(g.body);
      if (!group) {
        group = new THREE.Group();
        group.name = g.body;
        this.bodies.set(g.body, group);
        this.content.add(group);
      }
      let geom: THREE.BufferGeometry;
      const [sx, sy, sz] = g.size;
      switch (g.kind) {
        case "box": geom = new THREE.BoxGeometry(2 * sx, 2 * sy, 2 * sz); break;
        case "sphere": geom = new THREE.SphereGeometry(sx, 24, 16); break;
        case "cylinder": geom = new THREE.CylinderGeometry(sx, sx, 2 * sy, 32).rotateX(Math.PI / 2); break;
        case "capsule": geom = new THREE.CapsuleGeometry(sx, 2 * sy, 8, 16).rotateX(Math.PI / 2); break;
        case "plane": geom = new THREE.PlaneGeometry(2 * (sx || 4), 2 * (sy || 4)); break;
        case "mesh": {
          const info = g.mesh!;
          let cached = geoCache.get(info.offset);
          if (!cached) {
            cached = new THREE.BufferGeometry();
            cached.setAttribute("position", new THREE.BufferAttribute(new Float32Array(blob, info.offset, info.count * 3), 3));
            cached.setAttribute("normal", new THREE.BufferAttribute(new Float32Array(blob, info.offset + info.count * 12, info.count * 3), 3));
            cached.computeBoundingSphere();
            geoCache.set(info.offset, cached);
          }
          geom = cached;
          break;
        }
        default: continue;
      }
      if (g.body.startsWith("dest_")) {
        const d = vis.destinations.find((x) => x.body === g.body);
        if (d) this.sizes.set(g.body, [2 * sx, 2 * sy]);
      }
      let obj: THREE.Object3D;
      if (g.proxy) {
        obj = new THREE.LineSegments(new THREE.EdgesGeometry(geom), new THREE.LineBasicMaterial({ color: "#ffe08a", transparent: true, opacity: 0.85, depthTest: false }));
        obj.renderOrder = 9;
        obj.visible = this.showProxies;
        this.proxies.push(obj);
      } else {
        const color = new THREE.Color().setRGB(g.rgba[0], g.rgba[1], g.rgba[2], THREE.SRGBColorSpace);
        const isFloor = g.id === "floor";
        const isPad = g.body.startsWith("dest_");
        const isPart = g.body.startsWith("part_");
        const mat = new THREE.MeshStandardMaterial({
          color: isFloor ? new THREE.Color("#202224") : color,
          roughness: isFloor ? 0.95 : isPart ? 0.5 : Math.max(0.25, 0.6 - 0.5 * g.specular),
          metalness: 0.02,
          transparent: g.rgba[3] < 1,
          opacity: g.rgba[3],
          emissive: isPad ? color.clone().multiplyScalar(0.25) : new THREE.Color(0),
          side: g.kind === "mesh" && isPart ? THREE.DoubleSide : THREE.FrontSide,
        });
        const mesh = new THREE.Mesh(geom, mat);
        mesh.castShadow = !isFloor && !isPad;
        mesh.receiveShadow = true;
        obj = mesh;
      }
      obj.position.set(g.pos[0], g.pos[1], g.pos[2]);
      obj.quaternion.copy(quatWXYZ(g.quat_wxyz));
      obj.userData.body = g.body;
      group.add(obj);
    }
    if (!this.workspace.children.length) this.buildWorkspace(vis);
    if (!this.frustum) this.buildFrustum(vis);
    this.geometryRev = vis.geometry_rev;
  }

  setShowProxies(v: boolean) {
    this.showProxies = v;
    for (const p of this.proxies) p.visible = v;
  }

  private buildWorkspace(vis: Visuals) {
    const { reach_min, reach_max, abs_y_max } = vis.workspace;
    const shape = new THREE.Shape();
    const a0 = -Math.asin(Math.min(1, abs_y_max / reach_max));
    shape.absarc(0, 0, reach_max, a0, -a0, false);
    shape.absarc(0, 0, reach_min, -Math.asin(Math.min(1, abs_y_max / reach_min) * 0.999), Math.asin(Math.min(1, abs_y_max / reach_min) * 0.999), true);
    const geo = new THREE.ShapeGeometry(shape, 64);
    const fill = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: "#2fc4b2", transparent: true, opacity: 0.07, depthWrite: false }));
    fill.position.z = vis.table.top_z + 0.0008;
    const edge = new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({ color: "#2fc4b2", transparent: true, opacity: 0.35 }));
    edge.position.z = vis.table.top_z + 0.001;
    this.workspace.add(fill, edge);
  }

  private buildFrustum(vis: Visuals) {
    const c = vis.camera;
    const pos = new THREE.Vector3(...(c.position as [number, number, number]));
    const tgt = new THREE.Vector3(...(c.target as [number, number, number]));
    const g = new THREE.Group();
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.05, 0.04), new THREE.MeshStandardMaterial({ color: "#2b2f33", roughness: 0.5 }));
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 0.02, 20), new THREE.MeshStandardMaterial({ color: "#0e1113", roughness: 0.2 }));
    lens.rotateX(Math.PI / 2);
    lens.position.z = -0.03;
    g.add(body, lens);
    const fov = (c.vertical_fov_deg * Math.PI) / 180, aspect = 4 / 3, d = 0.11;
    const hh = Math.tan(fov / 2) * d, hw = hh * aspect;
    const pts = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]].map(([x, y]) => new THREE.Vector3(x, y, -d));
    const lines: THREE.Vector3[] = [];
    pts.forEach((p, i) => { lines.push(new THREE.Vector3(), p, p, pts[(i + 1) % 4]); });
    g.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(lines), new THREE.LineBasicMaterial({ color: "#3fe0cf", transparent: true, opacity: 0.35 })));
    g.position.copy(pos);
    g.up.set(0, 0, 1);
    g.lookAt(tgt);           // Object3D.lookAt points +Z at target for non-cameras
    g.rotateY(Math.PI);      // camera optics look along local -Z
    this.frustum = g;
    this.scene.add(g);
  }

  setShowCamera(v: boolean) { if (this.frustum) this.frustum.visible = v; }

  update(snap: Snapshot) {
    const now = performance.now();
    for (const [name, group] of this.bodies) {
      const p = snap.poses[name];
      group.visible = !!p || name === "world" || name === "table";
      if (!p) continue;
      if (this.dragging === name || (this.hold && this.hold.target === name && now < this.hold.until)) continue;
      group.position.set(p[0], p[1], p[2]);
      group.quaternion.set(p[4], p[5], p[6], p[3]);
    }
    if (this.hold && now >= this.hold.until) this.hold = null;
    this.workspace.visible = this.editable;
    this.updateSelection();
  }

  releaseHold() { this.hold = null; }

  setEstimates(est: Record<string, Estimate>, labelFor: (id: string) => string) {
    disposeTree(this.estimates);
    this.estimates.clear();
    for (const [id, e] of Object.entries(est)) {
      if (!e.valid || !e.position_m) continue;
      const size = this.sizes.get(`part_${id}`) ?? [0.05, 0.05, 0.06];
      const box = new THREE.LineSegments(this.unitEdges, new THREE.LineBasicMaterial({ color: "#3fe0cf", transparent: true, opacity: 0.95, depthTest: false }));
      box.renderOrder = 10;
      box.position.set(e.position_m[0], e.position_m[1], e.position_m[2]);
      box.scale.set(size[0] + 0.006, size[1] + 0.006, size[2] + 0.006);
      box.rotation.set(0, 0, e.yaw_rad ?? 0);
      const lbl = makeLabel(labelFor(id), "#3fe0cf");
      lbl.position.set(e.position_m[0], e.position_m[1], e.position_m[2] + size[2] / 2 + 0.04);
      this.estimates.add(box, lbl);
    }
  }

  setMarkers(markers: Marker[]) {
    disposeTree(this.markers);
    this.markers.clear();
    for (const m of markers) {
      const g = this.bodies.get(m.body);
      if (!g) continue;
      const s = makeLabel(m.text, m.color, 0.85);
      s.userData.follow = m.body;
      s.position.set(g.position.x, g.position.y, this.tableTop + 0.035);
      this.markers.add(s);
    }
  }

  refreshMarkers() {
    for (const s of this.markers.children) {
      const g = this.bodies.get(s.userData.follow as string);
      if (g) s.position.set(g.position.x, g.position.y, this.tableTop + 0.035);
    }
  }

  private updateSelection() {
    const target = this.selected ? this.bodies.get(this.selected) : null;
    if (!target || !this.editable || !target.visible) { this.selection.visible = false; return; }
    this.selection.visible = true;
    const size = this.sizes.get(this.selected!) ?? [0.08, 0.08];
    if (this.selected!.startsWith("part_")) {
      this.selection.position.copy(target.position);
      this.selection.quaternion.copy(target.quaternion);
      this.selection.scale.set(size[0] + 0.012, size[1] + 0.012, (size[2] ?? 0.05) + 0.012);
    } else {
      this.selection.position.set(target.position.x, target.position.y, this.tableTop + 0.003);
      this.selection.quaternion.copy(target.quaternion);
      this.selection.scale.set(size[0] + 0.01, size[1] + 0.01, 0.004);
    }
    this.refreshMarkers();
  }

  private pointer(ev: PointerEvent) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    return new THREE.Vector2(((ev.clientX - rect.left) / rect.width) * 2 - 1, -((ev.clientY - rect.top) / rect.height) * 2 + 1);
  }

  private pick(ev: PointerEvent): DragTarget | null {
    this.raycaster.setFromCamera(this.pointer(ev), this.camera);
    const objs = [...this.draggable].map((n) => this.bodies.get(n)).filter((g) => g && g.visible) as THREE.Object3D[];
    const hit = this.raycaster.intersectObjects(objs, true).find((h) => !(h.object instanceof THREE.LineSegments));
    return hit ? ((hit.object.userData.body as DragTarget) ?? null) : null;
  }

  private planePoint(ev: PointerEvent): THREE.Vector3 | null {
    this.raycaster.setFromCamera(this.pointer(ev), this.camera);
    const p = new THREE.Vector3();
    return this.raycaster.ray.intersectPlane(this.dragPlane, p) ? p : null;
  }

  private onDown = (ev: PointerEvent) => {
    if (ev.button !== 0) return;
    const t = this.pick(ev);
    if (!t) return;
    this.selected = t;
    this.cb.onSelect(t);
    if (!this.editable) return;
    const g = this.bodies.get(t)!;
    const p = this.planePoint(ev);
    if (!p) return;
    this.dragging = t;
    this.dragOffset.set(g.position.x - p.x, g.position.y - p.y, 0);
    this.controls.enabled = false;
    this.renderer.domElement.setPointerCapture(ev.pointerId);
    this.renderer.domElement.style.cursor = "grabbing";
  };

  private onMove = (ev: PointerEvent) => {
    if (!this.dragging) {
      this.renderer.domElement.style.cursor = this.editable && this.pick(ev) ? "grab" : "";
      return;
    }
    const p = this.planePoint(ev);
    if (!p) return;
    const g = this.bodies.get(this.dragging)!;
    g.position.x = p.x + this.dragOffset.x;
    g.position.y = p.y + this.dragOffset.y;
    this.updateSelection();
  };

  private onUp = () => {
    if (!this.dragging) return;
    const t = this.dragging;
    const g = this.bodies.get(t)!;
    this.dragging = null;
    this.controls.enabled = true;
    this.renderer.domElement.style.cursor = "";
    // keep the local preview until the backend's accepted revision arrives (or 1.5 s)
    this.hold = { target: t, until: performance.now() + 1500 };
    this.cb.onDragEnd(t, g.position.x, g.position.y);
  };

  viewPreset(name: "overview" | "top" | "camera") {
    if (name === "top") { this.camera.position.set(0.42, -0.001, 2.2); this.controls.target.set(0.42, 0, 0.4); }
    else if (name === "camera") { this.camera.position.set(1.18, 0, 1.02); this.controls.target.set(0.5, 0, 0.4); }
    else { this.camera.position.set(2.05, -1.6, 1.62); this.controls.target.set(0.36, 0, 0.56); }
  }
}

function disposeTree(root: THREE.Object3D) {
  root.traverse((o) => {
    const m = o as THREE.Mesh;
    if (m.geometry && !(m.geometry as THREE.BufferGeometry & { shared?: boolean }).shared) m.geometry.dispose?.();
    const mat = m.material as THREE.Material | THREE.Material[] | undefined;
    if (Array.isArray(mat)) mat.forEach((x) => x.dispose());
    else if (mat) {
      (mat as THREE.SpriteMaterial).map?.dispose?.();
      mat.dispose?.();
    }
  });
}

function makeLabelTexture(text: string, color: string): THREE.CanvasTexture {
  const c = document.createElement("canvas");
  c.width = 320; c.height = 64;
  const ctx = c.getContext("2d")!;
  ctx.font = "600 25px Inter, Segoe UI, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillStyle = "rgba(16,18,20,0.8)";
  const w = Math.min(310, ctx.measureText(text).width + 28);
  ctx.beginPath();
  ctx.roundRect((320 - w) / 2, 10, w, 44, 12);
  ctx.fill();
  ctx.fillStyle = color;
  ctx.fillText(text, 160, 33, 290);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function makeLabel(text: string, color: string, scale = 1): THREE.Sprite {
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: makeLabelTexture(text, color), depthTest: false, transparent: true }));
  s.scale.set(0.25 * scale, 0.05 * scale, 1);
  s.renderOrder = 11;
  return s;
}
