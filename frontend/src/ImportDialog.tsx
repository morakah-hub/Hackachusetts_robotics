import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { getJSON, mm, newId, PALETTE, post, postBytes, type AssetMeta, type PartSpec, type Role, type Snapshot } from "./api";

type Act = (fn: () => Promise<Record<string, unknown>>, ok?: string) => Promise<Record<string, unknown> | null>;
interface Materials { presets: Record<string, { label: string; density_kg_m3: number; friction: number }>; note: string }

const ROLE_LABEL: Record<Role, string> = { movable: "Movable part", fixture: "Fixture / obstacle", visual: "Visual reference" };

export default function ImportDialog({ snap, onClose, onAdded, act }: { snap: Snapshot; onClose: () => void; onAdded: (id: string) => void; act: Act }) {
  const [file, setFile] = useState<{ name: string; bytes: ArrayBuffer } | null>(null);
  const [units, setUnits] = useState("mm");
  const [scale, setScale] = useState("1");
  const [rot, setRot] = useState([0, 0, 0]);
  const [meta, setMeta] = useState<AssetMeta | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [role, setRole] = useState<Role>("movable");
  const [mats, setMats] = useState<Materials | null>(null);
  const [material, setMaterial] = useState("pla_20");
  const [adv, setAdv] = useState(false);
  const [massG, setMassG] = useState("");
  const [fric, setFric] = useState("");
  const [name, setName] = useState("");
  const used = new Set(snap.scene.parts.filter((p) => p.role === "movable").map((p) => p.color));
  const [color, setColor] = useState(Object.keys(PALETTE).find((c) => !used.has(c)) ?? "");
  const [category, setCategory] = useState("");
  const [x, setX] = useState("420");
  const [y, setY] = useState("300");
  const [busy, setBusy] = useState(false);

  useEffect(() => { void getJSON<Materials>("/api/materials").then(setMats).catch(() => undefined); }, []);

  // Upload (re)parses the file on the backend with the chosen units / scale / orientation.
  useEffect(() => {
    if (!file) return;
    const s = Number(scale);
    if (!Number.isFinite(s) || s <= 0) { setError("Scale must be a positive number"); return; }
    let cancelled = false;
    const t = setTimeout(async () => {
      setBusy(true);
      try {
        const q = new URLSearchParams({ filename: file.name, units, scale: String(s), rx: String(rot[0]), ry: String(rot[1]), rz: String(rot[2]) });
        const m = await postBytes<AssetMeta>(`/api/assets?${q}`, file.bytes);
        if (cancelled) return;
        setMeta(m);
        setError(null);
        setRole((r) => (m.eligible[r].ok ? r : m.eligible.movable.ok ? "movable" : m.eligible.fixture.ok ? "fixture" : "visual"));
      } catch (e) {
        if (!cancelled) { setMeta(null); setError((e as Error).message); }
      } finally {
        if (!cancelled) setBusy(false);
      }
    }, 250);
    return () => { cancelled = true; clearTimeout(t); };
  }, [file, units, scale, rot]);

  const preset = mats?.presets[material];
  const volume = meta ? meta.volume_m3 ?? meta.box_volume_m3 : 0;
  const presetMass = preset && meta ? preset.density_kg_m3 * volume : null;
  const mass = adv && massG.trim() ? Number(massG) / 1000 : presetMass ?? 0.1;
  const friction = adv && fric.trim() ? Number(fric) : preset?.friction ?? 0.8;

  const add = async () => {
    if (!meta) return;
    const taken = new Set([...snap.scene.parts.map((p) => p.id), ...snap.scene.destinations.map((d) => d.id)]);
    const label = name.trim() || meta.source_name.replace(/\.stl$/i, "");
    const part: PartSpec = {
      id: newId(label, taken), name: label, role, asset_id: meta.id, size: meta.dims_m, mass: role === "movable" ? mass : Math.max(mass, 0.05),
      friction, material: adv ? null : material, color: role === "movable" ? color : "orange", category: role === "movable" ? category.trim() : "",
      x: Number(x) / 1000, y: Number(y) / 1000, yaw: 0,
    };
    const scene = JSON.parse(JSON.stringify(snap.scene));
    scene.parts.push(part);
    const r = await act(() => post("/api/scene", { scene, expected_revision: snap.scene_revision, why: `imported ${label}` }), `Added ${label}`);
    if (r) onAdded(part.id);
  };

  const onFile = async (f: File | undefined) => {
    if (!f) return;
    setName(f.name.replace(/\.stl$/i, "").replace(/[_-]+/g, " ").slice(0, 40));
    setFile({ name: f.name, bytes: await f.arrayBuffer() });
  };

  const elig = meta?.eligible[role];
  const canAdd = !!meta && !!elig?.ok && !busy && (role !== "movable" || !!color) && Number.isFinite(mass) && mass > 0 && Number.isFinite(friction);

  return (
    <div className="modal-back" onClick={onClose}>
      <div className="modal" onClick={(e) => e.stopPropagation()}>
        <div className="modal-head"><h3>Import STL part</h3><button className="ghost" onClick={onClose}>Close</button></div>
        <div className="modal-body">
          <div className="col">
            <label className="filepick">{file ? file.name : "Choose an .stl file (binary or ASCII)…"}
              <input type="file" accept=".stl" onChange={(e) => void onFile(e.target.files?.[0])} /></label>
            <p className="hint">Try <code>examples/stl/</code>: a 40×60×45 mm spacer (binary), a 45 mm cube (ASCII) and an L bracket (not graspable).</p>
            <div className="placement">
              <label><span>units</span><select value={units} onChange={(e) => setUnits(e.target.value)}>
                <option value="mm">mm</option><option value="cm">cm</option><option value="m">m</option><option value="in">inch</option></select></label>
              <label><span>scale</span><input value={scale} onChange={(e) => setScale(e.target.value)} /><em>×</em></label>
              <span />
            </div>
            <div className="placement">
              {(["x", "y", "z"] as const).map((a, i) => (
                <label key={a}><span>rotate {a}</span><select value={rot[i]} onChange={(e) => setRot(rot.map((v, k) => (k === i ? Number(e.target.value) : v)))}>
                  {[0, 90, 180, 270].map((d) => <option key={d} value={d}>{d}°</option>)}</select></label>
              ))}
            </div>
            {busy && <p className="hint">Parsing…</p>}
            {error && <div className="badge fail">{error}</div>}
            {meta && (
              <>
                <div className="kv small"><span>Format / triangles</span><b>{meta.source_format} · {meta.triangles.toLocaleString()}</b></div>
                <div className="kv"><span>Size (x × y × z)</span><b>{meta.dims_m.map((v) => (v * 1000).toFixed(1)).join(" × ")} mm</b></div>
                <div className="kv small"><span>Closed mesh / fills box</span><b>{meta.watertight ? "yes" : "no"} · {meta.fill_ratio !== null ? `${Math.round(meta.fill_ratio * 100)}%` : "unknown"}</b></div>
                <div className="kv small"><span>Collision</span><b>box proxy (approximate physics)</b></div>
              </>
            )}
          </div>
          <div className="col">
            <MeshPreview assetId={meta?.id ?? null} dims={meta?.dims_m ?? null} />
            {meta && (
              <>
                <div className="roles">
                  {(["movable", "fixture", "visual"] as Role[]).map((r) => (
                    <label key={r} className={`role ${meta.eligible[r].ok ? "" : "no"}`}>
                      <input type="radio" name="role" checked={role === r} disabled={!meta.eligible[r].ok} onChange={() => setRole(r)} />
                      <span>{ROLE_LABEL[r]}</span>
                      {!meta.eligible[r].ok && <small>{meta.eligible[r].reasons[0]}</small>}
                    </label>
                  ))}
                </div>
                {role === "movable" && <p className="hint ok">Fits the validated top-down grasp: closes across the {mm(Math.min(meta.dims_m[0], meta.dims_m[1]), 0)} side.</p>}
                {role !== "visual" && (
                  <>
                    <label className="field"><span>Material preset</span>
                      <select value={material} onChange={(e) => setMaterial(e.target.value)} disabled={adv}>
                        {mats && Object.entries(mats.presets).map(([k, v]) => <option key={k} value={k}>{v.label} ({v.density_kg_m3} kg/m³, μ {v.friction})</option>)}
                      </select></label>
                    <div className="kv small"><span>Mass / friction used</span><b>{(mass * 1000).toFixed(0)} g · {friction.toFixed(2)}</b></div>
                    <button className="ghost wide small-btn" onClick={() => setAdv((v) => !v)}>{adv ? "Use preset" : "Advanced: enter measured mass and friction"}</button>
                    {adv && (
                      <div className="placement">
                        <label><span>mass</span><input value={massG} onChange={(e) => setMassG(e.target.value)} placeholder={presetMass ? (presetMass * 1000).toFixed(0) : ""} /><em>g</em></label>
                        <label><span>friction</span><input value={fric} onChange={(e) => setFric(e.target.value)} placeholder={String(preset?.friction ?? 0.8)} /><em /></label>
                        <span />
                      </div>
                    )}
                    {mats && <p className="hint">{mats.note}</p>}
                  </>
                )}
                <div className="row2"><input value={name} onChange={(e) => setName(e.target.value)} placeholder="Name" />
                  {role === "movable" && <select value={color} onChange={(e) => setColor(e.target.value)}>
                    <option value="">color…</option>
                    {Object.keys(PALETTE).map((c) => <option key={c} value={c} disabled={used.has(c)}>{c}{used.has(c) ? " (in use)" : ""}</option>)}
                  </select>}
                </div>
                {role === "movable" && <input value={category} onChange={(e) => setCategory(e.target.value)} placeholder="Category for sorting (optional)" />}
                <div className="placement">
                  <label><span>x</span><input value={x} onChange={(e) => setX(e.target.value)} /><em>mm</em></label>
                  <label><span>y</span><input value={y} onChange={(e) => setY(e.target.value)} /><em>mm</em></label>
                  <span />
                </div>
                {role === "movable" && <p className="hint">The camera identifies movable parts by their color; the part is printed in that color in the simulation.</p>}
                <button className="primary wide" disabled={!canAdd} onClick={() => void add()}>Add to scene</button>
              </>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}

function MeshPreview({ assetId, dims }: { assetId: string | null; dims: number[] | null }) {
  const host = useRef<HTMLDivElement>(null);
  const state = useRef<{ r: THREE.WebGLRenderer; scene: THREE.Scene; cam: THREE.PerspectiveCamera; group: THREE.Group; raf: number } | null>(null);
  useEffect(() => {
    const el = host.current!;
    const r = new THREE.WebGLRenderer({ antialias: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    r.setSize(el.clientWidth, 170);
    r.outputColorSpace = THREE.SRGBColorSpace;
    el.appendChild(r.domElement);
    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#15181a");
    scene.add(new THREE.HemisphereLight("#ffffff", "#333333", 1.2));
    const dl = new THREE.DirectionalLight("#ffffff", 1.6);
    dl.position.set(1, -1.5, 2);
    scene.add(dl);
    const cam = new THREE.PerspectiveCamera(35, el.clientWidth / 170, 0.001, 10);
    cam.up.set(0, 0, 1);
    const group = new THREE.Group();
    scene.add(group);
    const loop = () => {
      const s = state.current!;
      s.raf = requestAnimationFrame(loop);
      group.rotation.z += 0.006;
      r.render(scene, cam);
    };
    state.current = { r, scene, cam, group, raf: 0 };
    loop();
    return () => { cancelAnimationFrame(state.current!.raf); r.dispose(); r.domElement.remove(); };
  }, []);
  useEffect(() => {
    const s = state.current;
    if (!s) return;
    s.group.clear();
    if (!assetId || !dims) return;
    let cancelled = false;
    void fetch(`/api/assets/${assetId}/mesh`).then((res) => res.arrayBuffer()).then((buf) => {
      if (cancelled) return;
      const g = new STLLoader().parse(buf);
      g.computeVertexNormals();
      s.group.add(new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: "#d9d2c5", roughness: 0.6, side: THREE.DoubleSide })));
      const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(dims[0], dims[1], dims[2])), new THREE.LineBasicMaterial({ color: "#ffe08a" }));
      s.group.add(box);
      const d = Math.max(...dims);
      s.cam.position.set(1.6 * d, -2.2 * d, 1.5 * d);
      s.cam.lookAt(0, 0, 0);
    });
    return () => { cancelled = true; };
  }, [assetId, dims]);
  return (
    <div className="preview" ref={host}>
      <span className="cap">{dims ? `canonical mesh · yellow = box collision proxy ${dims.map((v) => (v * 1000).toFixed(0)).join("×")} mm` : "preview"}</span>
    </div>
  );
}
