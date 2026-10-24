import { useCallback, useEffect, useMemo, useState } from "react";
import {
  ApiError, mm, newId, PALETTE, PHASES, post, postBytes, getJSON,
  type DestSpec, type Estimate, type PartSpec, type ProjectInfo, type Recipe, type SceneCfg, type Snapshot, type Step,
} from "./api";
import { useSim } from "./useSim";
import Viewport from "./Viewport";
import ImportDialog from "./ImportDialog";
import type { DragTarget, Marker } from "./sceneView";

type Mode = "camera_rgbd" | "known_world";
type Section = "project" | "parts" | "dests" | "workflow";
type Act = (fn: () => Promise<Record<string, unknown>>, ok?: string) => Promise<Record<string, unknown> | null>;
const deg = (r: number) => (r * 180) / Math.PI;
const rad = (d: number) => (d * Math.PI) / 180;
const EDITABLE = ["idle", "completed", "failed", "stopped"];
const clone = <T,>(x: T): T => JSON.parse(JSON.stringify(x)) as T;

const STATUS_LABEL: Record<string, string> = {
  loading: "Loading", idle: "Ready", running: "Running", paused: "Paused", completed: "Completed",
  failed: "Failed", stopped: "Stopped", error: "Backend error",
};

export default function App() {
  const { snap, conn } = useSim();
  const [mode, setMode] = useState<Mode>("camera_rgbd");
  const [selected, setSelected] = useState<DragTarget | null>(null);
  const [section, setSection] = useState<Section>("workflow");
  const [notice, setNotice] = useState<{ kind: "error" | "info"; text: string } | null>(null);
  const [pendingRev, setPendingRev] = useState<number | null>(null);
  const [busy, setBusy] = useState(false);
  const [devOpen, setDevOpen] = useState(false);
  const [importOpen, setImportOpen] = useState(false);
  const [geom, setGeom] = useState<{ ok: boolean; msg?: string } | null>(null);
  const [showCamera, setShowCamera] = useState(true);
  const [showProxies, setShowProxies] = useState(false);

  const status = conn !== "connected" ? "disconnected" : snap?.status ?? "loading";
  const editable = !!snap && EDITABLE.includes(snap.status) && conn === "connected";
  const revisionPending = pendingRev !== null && (snap?.scene_revision ?? 0) < pendingRev;

  useEffect(() => {
    if (pendingRev !== null && snap && snap.scene_revision >= pendingRev) setPendingRev(null);
  }, [snap, pendingRev]);

  const act: Act = useCallback(async (fn, ok) => {
    setBusy(true);
    try {
      const r = await fn();
      if (typeof r.scene_revision === "number") setPendingRev(r.scene_revision);
      if (ok) setNotice({ kind: "info", text: ok });
      return r;
    } catch (e) {
      const err = e as ApiError;
      setNotice({ kind: "error", text: `${err.code ?? "ERROR"}: ${err.message}` });
      return null;
    } finally {
      setBusy(false);
    }
  }, []);

  const rev = snap?.scene_revision;
  const editPose = useCallback((target: DragTarget, v: Record<string, number | null>) => {
    const isPart = target.startsWith("part_");
    const id = target.slice(5);
    return act(() => post("/api/scene/edit", { [isPart ? "parts" : "dests"]: { [id]: v }, expected_revision: rev }));
  }, [act, rev]);

  const setScene = useCallback((mutate: (s: SceneCfg) => void, why: string, ok?: string) => {
    if (!snap) return Promise.resolve(null);
    const s = clone(snap.scene);
    mutate(s);
    return act(() => post("/api/scene", { scene: s, expected_revision: snap.scene_revision, why }), ok);
  }, [act, snap]);

  const setRecipe = useCallback((r: Recipe) => act(() => post("/api/recipe", { recipe: r })), [act]);

  const onDragEnd = useCallback((t: DragTarget, x: number, y: number) => { void editPose(t, { x, y }); }, [editPose]);

  const run = () => act(() => post("/api/run", { type: "workflow", perception_mode: mode, scene_revision: snap?.scene_revision }))
    .then((r) => r && setNotice(null));
  const control = (action: string, extra: Record<string, unknown> = {}) => act(() => post("/api/control", { action, ...extra }));

  const result = snap?.result ?? null;
  const showResult = result && snap && ["completed", "failed", "stopped"].includes(snap.status);
  const estimates: Record<string, Estimate> = snap?.estimates ?? {};

  const markers: Marker[] = useMemo(() => {
    if (!snap) return [];
    const out: Marker[] = [];
    const num = new Map<string, number[]>();
    snap.recipe.steps.forEach((s, i) => { if (s.action === "place" && s.dest) num.set(s.dest, [...(num.get(s.dest) ?? []), i + 1]); });
    for (const d of snap.scene.destinations) {
      const n = num.get(d.id);
      out.push({ body: `dest_${d.id}`, text: `${n ? n.join(",") + " · " : ""}${d.name}${d.zone ? ` (${d.zone})` : ""}`, color: n ? "#ffd089" : "#9fd8d0" });
    }
    return out;
  }, [snap?.recipe, snap?.scene.destinations]);   // eslint-disable-line react-hooks/exhaustive-deps

  const selPart = selected?.startsWith("part_") ? snap?.scene.parts.find((p) => `part_${p.id}` === selected) ?? null : null;
  const selDest = selected?.startsWith("dest_") ? snap?.scene.destinations.find((d) => `dest_${d.id}` === selected) ?? null : null;
  const onSelect = (t: DragTarget) => { setSelected(t); setSection(t.startsWith("part_") ? "parts" : "dests"); };

  return (
    <div className="app">
      <header className="topbar">
        <div className="brand">
          <svg viewBox="0 0 32 32" width="26" height="26"><rect width="32" height="32" rx="7" fill="#24282b" /><path d="M8 24 L14 9 L19 17 L24 13" stroke="#2fc4b2" strokeWidth="3" fill="none" strokeLinecap="round" strokeLinejoin="round" /><circle cx="24" cy="13" r="2.6" fill="#f08a24" /></svg>
          <span className="name">Armstrong</span>
          <span className="sub">{snap?.scene.name ?? "Tabletop workcell"}</span>
        </div>
        <div className="top-right">
          <span className={`pill conn-${conn}`}><i />{conn === "connected" ? "Connected" : conn === "connecting" ? "Connecting…" : "Disconnected — retrying"}</span>
          <span className={`pill status-${status}`}><i />{STATUS_LABEL[status] ?? status}{snap?.phase ? ` · step ${snap.progress?.current_step ?? "–"} ${snap.phase}` : ""}</span>
          <span className="pill muted">rev {snap?.scene_revision ?? "–"}</span>
          <button className={`ghost ${devOpen ? "on" : ""}`} onClick={() => setDevOpen((v) => !v)}>Developer</button>
        </div>
      </header>

      <aside className="sidebar">
        <div className="side-scroll">
          <Accordion title="Project" open={section === "project"} onToggle={() => setSection("project")} summary={snap?.scene.name}>
            <ProjectPanel snap={snap} act={act} editable={editable && !busy} />
          </Accordion>
          <Accordion title="Parts" open={section === "parts"} onToggle={() => setSection("parts")} summary={snap ? `${snap.scene.parts.length} in scene` : ""}>
            {snap && <PartsPanel snap={snap} selected={selPart} disabled={!editable || busy} onSelect={(id) => setSelected(`part_${id}`)}
              setScene={setScene} editPose={editPose} onImport={() => setImportOpen(true)} />}
          </Accordion>
          <Accordion title="Destinations" open={section === "dests"} onToggle={() => setSection("dests")} summary={snap ? `${snap.scene.destinations.length} defined` : ""}>
            {snap && <DestPanel snap={snap} selected={selDest} disabled={!editable || busy} onSelect={(id) => setSelected(`dest_${id}`)}
              setScene={setScene} editPose={editPose} />}
          </Accordion>
          <Accordion title="Workflow" open={section === "workflow"} onToggle={() => setSection("workflow")}
            summary={snap ? (snap.recipe_status.ok ? `${snap.recipe.steps.length} step${snap.recipe.steps.length === 1 ? "" : "s"} · valid` : "needs attention") : ""}>
            {snap && <WorkflowPanel snap={snap} disabled={!editable || busy} setRecipe={setRecipe} setScene={setScene} notify={setNotice} />}
          </Accordion>
        </div>
        <section className="card runbar">
          <div className="runrow">
            <select value={mode} onChange={(e) => setMode(e.target.value as Mode)} disabled={!editable} title="How parts are located">
              <option value="camera_rgbd">Camera RGB-D (image-derived)</option>
              <option value="known_world">Known world — development only</option>
            </select>
          </div>
          {mode === "known_world" && <div className="badge warn">DEV MODE: plans from simulator poses, not the camera</div>}
          <div className="controls">
            <button className="primary" disabled={!editable || busy || revisionPending || !snap?.recipe_status.ok} onClick={run}
              title={snap?.recipe_status.ok ? "" : snap?.recipe_status.error ?? ""}>{revisionPending ? "Waiting…" : "Run"}</button>
            {snap?.status === "paused"
              ? <button onClick={() => control("resume")}>Resume</button>
              : <button disabled={snap?.status !== "running"} onClick={() => control("pause")}>Pause</button>}
            <button disabled={!(snap?.status === "running" || snap?.status === "paused")} onClick={() => control("stop")}>Stop</button>
            <button disabled={busy || !snap || snap.status === "loading"} onClick={() => control("reset")} title="Arm home, parts back to their start positions">Reset</button>
          </div>
          {snap?.status === "paused" && (
            <div className="controls small">
              <button onClick={() => control("step", { steps: 5 })}>Step 10 ms</button>
              <button onClick={() => control("step", { steps: 50 })}>Step 100 ms</button>
            </div>
          )}
          <RunProgress snap={snap} />
        </section>
      </aside>

      <main className="stage">
        <Viewport snap={snap} editable={editable} selected={selected} showCamera={showCamera} showProxies={showProxies}
          estimates={estimates} markers={markers} onSelect={onSelect} onDragEnd={onDragEnd} onLoaded={(ok, msg) => setGeom({ ok, msg })} />
        {(!geom || conn !== "connected") && (
          <div className="overlay-center">
            {conn !== "connected" ? <><div className="spinner" />Connecting to the local backend…</> : <><div className="spinner" />Loading robot geometry…</>}
          </div>
        )}
        {geom && !geom.ok && <div className="overlay-center error">Could not load scene geometry: {geom.msg}</div>}
        <PhaseStrip snap={snap} />
        {notice && <div className={`toast ${notice.kind}`} onClick={() => setNotice(null)}>{notice.text}<span className="x">×</span></div>}
        {showResult && result && <ResultBanner snap={snap!} />}
        <div className="stage-toggles">
          <label><input type="checkbox" checked={showCamera} onChange={(e) => setShowCamera(e.target.checked)} /> camera</label>
          <label title="Box collision proxies used by physics"><input type="checkbox" checked={showProxies} onChange={(e) => setShowProxies(e.target.checked)} /> collision proxies</label>
        </div>
        {importOpen && snap && <ImportDialog snap={snap} onClose={() => setImportOpen(false)} act={act}
          onAdded={(id) => { setImportOpen(false); setSelected(`part_${id}`); setSection("parts"); }} />}
      </main>

      <aside className="rightbar">
        <CameraPanel snap={snap} />
        <PerceptionPanel snap={snap} mode={mode} onObserve={() => act(() => post("/api/perception/observe"))} canObserve={editable && !busy} />
        <ResultPanel snap={snap} />
      </aside>

      {devOpen && snap && <DevDrawer snap={snap} onClose={() => setDevOpen(false)} act={act} />}
    </div>
  );
}

function Accordion({ title, open, onToggle, summary, children }: { title: string; open: boolean; onToggle: () => void; summary?: string; children: React.ReactNode }) {
  return (
    <section className={`card acc ${open ? "open" : ""}`}>
      <button className="acc-head" onClick={onToggle}><h3>{title}</h3><small>{summary}</small><span className="chev">{open ? "▾" : "▸"}</span></button>
      {open && <div className="acc-body">{children}</div>}
    </section>
  );
}

// ------------------------------------------------------------------ project

function ProjectPanel({ snap, act, editable }: { snap: Snapshot | null; act: Act; editable: boolean }) {
  const [list, setList] = useState<ProjectInfo[]>([]);
  const [pick, setPick] = useState("");
  const [name, setName] = useState("");
  const refresh = useCallback(() => getJSON<{ projects: ProjectInfo[] }>("/api/projects").then((r) => setList(r.projects)).catch(() => undefined), []);
  useEffect(() => { void refresh(); }, [refresh]);
  useEffect(() => { if (snap && !name) setName(snap.scene.name); }, [snap, name]);
  const chosen = list.find((p) => `${p.source}:${p.id}` === pick);
  const open = () => chosen && act(() => post("/api/projects/open", { source: chosen.source, id: chosen.id }), `Opened “${chosen.name}”`);
  const save = () => act(() => post("/api/projects/save", { name }), `Saved “${name}” to user_data/projects`).then(() => refresh());
  const importFile = async (f: File | undefined) => {
    if (!f) return;
    const buf = await f.arrayBuffer();
    await act(() => postBytes("/api/projects/import", buf), `Imported “${f.name}”`);
    void refresh();
  };
  return (
    <>
      <label className="field"><span>Open an example or saved project</span>
        <select value={pick} onChange={(e) => setPick(e.target.value)}>
          <option value="">Choose…</option>
          <optgroup label="Examples (shipped)">{list.filter((p) => p.source === "example").map((p) => <option key={p.id} value={`example:${p.id}`}>{p.name}</option>)}</optgroup>
          <optgroup label="My projects">{list.filter((p) => p.source === "user").map((p) => <option key={p.id} value={`user:${p.id}`}>{p.name}</option>)}</optgroup>
        </select>
      </label>
      {chosen && <p className="hint">{chosen.description || `${chosen.parts} parts · ${chosen.steps} steps`}</p>}
      <button className="wide" disabled={!editable || !chosen} onClick={open}>Open</button>
      <div className="row2">
        <input value={name} onChange={(e) => setName(e.target.value)} placeholder="Project name" />
        <button disabled={!editable || !name.trim()} onClick={save}>Save</button>
      </div>
      <div className="row2">
        <a className="ghost link" href={`/api/projects/export?name=${encodeURIComponent(name || "project")}`}>Export bundle</a>
        <label className={`ghost link file ${editable ? "" : "disabled"}`}>Import bundle
          <input type="file" accept=".zip" disabled={!editable} onChange={(e) => { void importFile(e.target.files?.[0]); e.target.value = ""; }} /></label>
      </div>
      <p className="hint">Bundles (.armstrong.zip) contain the scene, workflow, settings and meshes — no local paths.</p>
    </>
  );
}

// ------------------------------------------------------------------ parts

function PartsPanel({ snap, selected, disabled, onSelect, setScene, editPose, onImport }: {
  snap: Snapshot; selected: PartSpec | null; disabled: boolean; onSelect: (id: string) => void;
  setScene: (m: (s: SceneCfg) => void, why: string, ok?: string) => Promise<unknown>;
  editPose: (t: DragTarget, v: Record<string, number | null>) => Promise<unknown>; onImport: () => void;
}) {
  const parts = snap.scene.parts;
  const used = new Set(parts.filter((p) => p.role === "movable").map((p) => p.color));
  const free = Object.keys(PALETTE).find((c) => !used.has(c));
  const addBlock = () => setScene((s) => {
    const id = newId(`${free}_block`, new Set([...s.parts.map((p) => p.id), ...s.destinations.map((d) => d.id)]));
    const spot = freeSpot(s);
    s.parts.push({ id, name: `${free![0].toUpperCase()}${free!.slice(1)} block`, role: "movable", asset_id: null, size: [0.05, 0.05, 0.06],
      mass: 0.4, friction: 1.0, material: null, color: free!, category: "", x: spot[0], y: spot[1], yaw: 0 });
  }, "block added", "Block added");
  return (
    <>
      <div className="list">
        {parts.map((p) => (
          <button key={p.id} className={`item ${selected?.id === p.id ? "on" : ""}`} onClick={() => onSelect(p.id)}>
            <span className="sw" style={{ background: p.role === "movable" ? PALETTE[p.color] : p.role === "fixture" ? "#5c636b" : "#bdb4a3" }} />
            <span className="nm">{p.name}</span>
            <span className="tag">{p.role === "movable" ? (p.category || "part") : p.role}</span>
            <span className="dims">{p.size.map((v) => (v * 1000).toFixed(0)).join("×")}</span>
          </button>
        ))}
      </div>
      <div className="row2">
        <button disabled={disabled || !free} onClick={addBlock} title={free ? "" : "All four camera colors are in use"}>+ Block</button>
        <button disabled={disabled} onClick={onImport}>Import STL…</button>
      </div>
      {selected && <PartEditor key={`${selected.id}-${snap.scene_revision}`} part={selected} snap={snap} disabled={disabled} setScene={setScene} editPose={editPose} />}
      {!selected && <p className="hint">Select a part here or click it in the scene. Drag parts on the table while stopped.</p>}
    </>
  );
}

function freeSpot(s: SceneCfg): [number, number] {
  const cands: [number, number][] = [[0.40, 0.30], [0.52, 0.30], [0.62, 0.20], [0.40, 0.18], [0.52, 0.12], [0.64, 0.32], [0.34, 0.24]];
  return cands.find(([x, y]) => s.parts.every((p) => Math.hypot(p.x - x, p.y - y) > 0.1)) ?? [0.45, 0.32];
}

function PartEditor({ part, snap, disabled, setScene, editPose }: {
  part: PartSpec; snap: Snapshot; disabled: boolean; setScene: (m: (s: SceneCfg) => void, why: string, ok?: string) => Promise<unknown>;
  editPose: (t: DragTarget, v: Record<string, number | null>) => Promise<unknown>;
}) {
  const [x, setX] = useState((part.x * 1000).toFixed(0));
  const [y, setY] = useState((part.y * 1000).toFixed(0));
  const [yaw, setYaw] = useState(deg(part.yaw).toFixed(0));
  const [name, setName] = useState(part.name);
  const [cat, setCat] = useState(part.category);
  const [mass, setMass] = useState((part.mass * 1000).toFixed(0));
  const [fric, setFric] = useState(part.friction.toFixed(2));
  const [adv, setAdv] = useState(false);
  const usedColors = new Set(snap.scene.parts.filter((p) => p.role === "movable" && p.id !== part.id).map((p) => p.color));
  const update = (m: (p: PartSpec) => void, why: string) => setScene((s) => { const p = s.parts.find((q) => q.id === part.id)!; m(p); }, why);
  const applyPose = () => {
    const v = { x: Number(x) / 1000, y: Number(y) / 1000, yaw: rad(Number(yaw)) };
    if (Object.values(v).every(Number.isFinite)) void editPose(`part_${part.id}`, v);
  };
  return (
    <div className="editor">
      <div className="row2">
        <input value={name} onChange={(e) => setName(e.target.value)} disabled={disabled} />
        <button disabled={disabled || name === part.name || !name.trim()} onClick={() => update((p) => { p.name = name.trim(); }, "renamed")}>Rename</button>
      </div>
      <div className="kv small"><span>Shape</span><b>{part.asset_id ? "uploaded STL" : "primitive box"} · {part.size.map((v) => (v * 1000).toFixed(1)).join(" × ")} mm</b></div>
      <div className="kv small"><span>Physics</span><b>{part.role === "visual" ? "visual only (no collision)" : part.asset_id ? "box proxy of the mesh (approximate)" : "box"}</b></div>
      <div className="placement">
        <label><span>x</span><input value={x} onChange={(e) => setX(e.target.value)} disabled={disabled} /><em>mm</em></label>
        <label><span>y</span><input value={y} onChange={(e) => setY(e.target.value)} disabled={disabled} /><em>mm</em></label>
        <label><span>yaw</span><input value={yaw} onChange={(e) => setYaw(e.target.value)} disabled={disabled} /><em>°</em></label>
        <button disabled={disabled} onClick={applyPose}>Apply start pose</button>
      </div>
      {part.role === "movable" && (
        <>
          <label className="field"><span>Camera color (identifies the part)</span>
            <select value={part.color} disabled={disabled} onChange={(e) => update((p) => { p.color = e.target.value; }, "color changed")}>
              {Object.keys(PALETTE).map((c) => <option key={c} value={c} disabled={usedColors.has(c)}>{c}{usedColors.has(c) ? " (in use)" : ""}</option>)}
            </select>
          </label>
          <div className="row2">
            <input value={cat} onChange={(e) => setCat(e.target.value)} placeholder="Category for sorting (e.g. large)" disabled={disabled} />
            <button disabled={disabled || cat === part.category} onClick={() => update((p) => { p.category = cat.trim(); }, "category set")}>Set</button>
          </div>
        </>
      )}
      {part.role !== "visual" && (
        <button className="ghost wide small-btn" onClick={() => setAdv((v) => !v)}>{adv ? "Hide" : "Advanced"}: mass {(part.mass * 1000).toFixed(0)} g · friction {part.friction.toFixed(2)}</button>
      )}
      {adv && (
        <div className="placement">
          <label><span>mass</span><input value={mass} onChange={(e) => setMass(e.target.value)} disabled={disabled} /><em>g</em></label>
          <label><span>friction</span><input value={fric} onChange={(e) => setFric(e.target.value)} disabled={disabled} /><em /></label>
          <span />
          <button disabled={disabled} onClick={() => update((p) => { p.mass = Number(mass) / 1000; p.friction = Number(fric); p.material = null; }, "physics changed")}>Apply physics</button>
        </div>
      )}
      <button className="ghost wide danger" disabled={disabled} onClick={() => setScene((s) => { s.parts = s.parts.filter((p) => p.id !== part.id); }, "part removed", `Removed ${part.name}`)}>Remove from scene</button>
    </div>
  );
}

// ------------------------------------------------------------------ destinations

function DestPanel({ snap, selected, disabled, onSelect, setScene, editPose }: {
  snap: Snapshot; selected: DestSpec | null; disabled: boolean; onSelect: (id: string) => void;
  setScene: (m: (s: SceneCfg) => void, why: string, ok?: string) => Promise<unknown>;
  editPose: (t: DragTarget, v: Record<string, number | null>) => Promise<unknown>;
}) {
  const add = () => setScene((s) => {
    const id = newId("slot", new Set([...s.parts.map((p) => p.id), ...s.destinations.map((d) => d.id)]));
    // keep >= 120 mm between pads (room for the open gripper) and away from parts' start positions
    const cands: [number, number][] = [[0.42, -0.14], [0.56, -0.14], [0.42, -0.28], [0.56, -0.28], [0.66, -0.02], [0.34, -0.18], [0.66, -0.16], [0.48, -0.38]];
    const spot = cands.find(([x, y]) => s.destinations.every((d) => Math.hypot(d.x - x, d.y - y) >= 0.12)
      && s.parts.every((p) => Math.hypot(p.x - x, p.y - y) >= 0.1)) ?? [0.45, -0.2];
    s.destinations.push({ id, name: `Slot ${s.destinations.length + 1}`, x: spot[0], y: spot[1], yaw: null, zone: "", size: [0.07, 0.07] });
  }, "destination added", "Destination added");
  return (
    <>
      <div className="list">
        {snap.scene.destinations.map((d) => (
          <button key={d.id} className={`item ${selected?.id === d.id ? "on" : ""}`} onClick={() => onSelect(d.id)}>
            <span className="sw teal" /><span className="nm">{d.name}</span><span className="tag">{d.zone || "slot"}</span>
            <span className="dims">{d.yaw === null ? "any yaw" : `${deg(d.yaw).toFixed(0)}°`}</span>
          </button>
        ))}
      </div>
      <button className="wide" disabled={disabled || snap.scene.destinations.length >= 8} onClick={add}>+ Destination</button>
      {selected && <DestEditor key={`${selected.id}-${snap.scene_revision}`} dest={selected} disabled={disabled} setScene={setScene} editPose={editPose} />}
      {!selected && <p className="hint">Destinations are user-defined goal pads (not perceived). A requested yaw makes the robot rotate the part.</p>}
    </>
  );
}

function DestEditor({ dest, disabled, setScene, editPose }: {
  dest: DestSpec; disabled: boolean; setScene: (m: (s: SceneCfg) => void, why: string, ok?: string) => Promise<unknown>;
  editPose: (t: DragTarget, v: Record<string, number | null>) => Promise<unknown>;
}) {
  const [x, setX] = useState((dest.x * 1000).toFixed(0));
  const [y, setY] = useState((dest.y * 1000).toFixed(0));
  const [yaw, setYaw] = useState(dest.yaw === null ? "" : deg(dest.yaw).toFixed(0));
  const [name, setName] = useState(dest.name);
  const [zone, setZone] = useState(dest.zone);
  const apply = () => {
    const v: Record<string, number | null> = { x: Number(x) / 1000, y: Number(y) / 1000, yaw: yaw.trim() === "" ? null : rad(Number(yaw)) };
    if (Number.isFinite(v.x) && Number.isFinite(v.y) && (v.yaw === null || Number.isFinite(v.yaw))) void editPose(`dest_${dest.id}`, v);
  };
  return (
    <div className="editor">
      <div className="row2">
        <input value={name} onChange={(e) => setName(e.target.value)} disabled={disabled} />
        <button disabled={disabled || name === dest.name || !name.trim()} onClick={() => setScene((s) => { s.destinations.find((d) => d.id === dest.id)!.name = name.trim(); }, "renamed")}>Rename</button>
      </div>
      <div className="placement">
        <label><span>x</span><input value={x} onChange={(e) => setX(e.target.value)} disabled={disabled} /><em>mm</em></label>
        <label><span>y</span><input value={y} onChange={(e) => setY(e.target.value)} disabled={disabled} /><em>mm</em></label>
        <label><span>yaw</span><input value={yaw} onChange={(e) => setYaw(e.target.value)} placeholder="keep" disabled={disabled} /><em>°</em></label>
        <button disabled={disabled} onClick={apply}>Apply</button>
      </div>
      <div className="row2">
        <input value={zone} onChange={(e) => setZone(e.target.value)} placeholder="Zone name (for sorting)" disabled={disabled} />
        <button disabled={disabled || zone === dest.zone} onClick={() => setScene((s) => { s.destinations.find((d) => d.id === dest.id)!.zone = zone.trim(); }, "zone set")}>Set</button>
      </div>
      <button className="ghost wide danger" disabled={disabled} onClick={() => setScene((s) => { s.destinations = s.destinations.filter((d) => d.id !== dest.id); }, "destination removed", `Removed ${dest.name}`)}>Remove destination</button>
    </div>
  );
}

// ------------------------------------------------------------------ workflow

const SLOT_LAYOUT: [number, number][] = [[0.42, -0.13], [0.54, -0.13], [0.42, -0.25], [0.54, -0.25]];

function WorkflowPanel({ snap, disabled, setRecipe, setScene, notify }: {
  snap: Snapshot; disabled: boolean; setRecipe: (r: Recipe) => Promise<unknown>;
  setScene: (m: (s: SceneCfg) => void, why: string, ok?: string) => Promise<unknown>;
  notify: (n: { kind: "error" | "info"; text: string }) => void;
}) {
  const r = snap.recipe;
  const movable = snap.scene.parts.filter((p) => p.role === "movable");
  const dests = snap.scene.destinations;
  const zones = [...new Set(dests.map((d) => d.zone).filter(Boolean))];
  const cats = [...new Set(movable.map((p) => p.category).filter(Boolean))];
  const save = (m: (x: Recipe) => void) => { const x = clone(r); m(x); void setRecipe(x); };
  const move = (i: number, d: number) => save((x) => { const s = x.steps.splice(i, 1)[0]; x.steps.splice(i + d, 0, s); });

  const preset = async (kind: "move" | "kit" | "grid" | "sort") => {
    if (!movable.length) { notify({ kind: "error", text: "Add a movable part first." }); return; }
    if (kind === "move") {
      if (!dests.length) { notify({ kind: "error", text: "Add a destination first." }); return; }
      await setRecipe({ version: 1, name: "Move part", sort_rules: [], steps: [{ id: "s1", part: movable[0].id, action: "place", dest: dests[0].id }] });
      return;
    }
    if (kind === "sort") {
      const parts = movable.filter((p) => p.category);
      if (!parts.length) { notify({ kind: "error", text: "Give parts a category first (Parts › select a part › Category)." }); return; }
      const cs = [...new Set(parts.map((p) => p.category))];
      const res = await setScene((s) => {
        s.destinations = s.destinations.filter((d) => !d.id.startsWith("zone_"));
        cs.forEach((c, row) => parts.filter((p) => p.category === c).forEach((_, k) => {
          const idx = s.destinations.filter((d) => d.id.startsWith("zone_")).length;
          s.destinations.push({ id: newId(`zone_${c}_${k + 1}`, new Set([...s.parts.map((p) => p.id), ...s.destinations.map((d) => d.id)])),
            name: `${c} ${k + 1}`, x: 0.42 + 0.12 * k, y: -0.13 - 0.12 * row, yaw: null, zone: `${c} zone`, size: [0.07, 0.07] });
          void idx;
        }));
      }, "sorting zones created");
      if (!res) return;
      await setRecipe({ version: 1, name: "Sort by category", steps: parts.map((p, i) => ({ id: `s${i + 1}`, part: p.id, action: "sort" })),
        sort_rules: cs.map((c) => ({ category: c, zone: `${c} zone` })) });
      return;
    }
    const n = Math.min(movable.length, 4);
    const prefix = kind === "kit" ? "kit" : "grid";
    const label = kind === "kit" ? ["Kit slot A", "Kit slot B", "Kit slot C", "Kit slot D"] : ["Grid A1", "Grid A2", "Grid B1", "Grid B2"];
    const ids: string[] = [];
    const res = await setScene((s) => {
      s.destinations = s.destinations.filter((d) => !d.id.startsWith(`${prefix}_`));
      for (let i = 0; i < n; i++) {
        const id = newId(`${prefix}_${i + 1}`, new Set([...s.parts.map((p) => p.id), ...s.destinations.map((d) => d.id)]));
        ids.push(id);
        const [x, y] = kind === "grid" ? [0.43 + 0.10 * (i % 2), -0.10 - 0.10 * Math.floor(i / 2)] : SLOT_LAYOUT[i];
        s.destinations.push({ id, name: label[i], x, y, yaw: kind === "grid" ? 0 : null, zone: "", size: [0.07, 0.07] });
      }
    }, `${kind} slots created`);
    if (!res) return;
    await setRecipe({ version: 1, name: kind === "kit" ? "Kit parts" : "Arrange in grid", sort_rules: [],
      steps: movable.slice(0, n).map((p, i) => ({ id: `s${i + 1}`, part: p.id, action: "place", dest: ids[i] })) });
  };

  return (
    <>
      <div className="presets">
        <span>Presets</span>
        <button disabled={disabled} onClick={() => void preset("move")}>Move</button>
        <button disabled={disabled} onClick={() => void preset("kit")}>Kit</button>
        <button disabled={disabled} onClick={() => void preset("grid")}>Grid</button>
        <button disabled={disabled} onClick={() => void preset("sort")}>Sort</button>
      </div>
      <div className="steps">
        {r.steps.map((s, i) => (
          <StepCard key={`${i}-${s.id}`} i={i} step={s} n={r.steps.length} snap={snap} disabled={disabled}
            onChange={(ns) => save((x) => { x.steps[i] = ns; })} onMove={(d) => move(i, d)} onRemove={() => save((x) => { x.steps.splice(i, 1); })} />
        ))}
        {!r.steps.length && <p className="hint">No steps yet. Use a preset or add a step.</p>}
      </div>
      <button className="wide" disabled={disabled || r.steps.length >= 8 || !movable.length}
        onClick={() => save((x) => { x.steps.push({ id: `s${Date.now() % 100000}`, part: movable.find((p) => !x.steps.some((s) => s.part === p.id))?.id ?? movable[0].id, action: "place", dest: dests.find((d) => !x.steps.some((s) => s.dest === d.id))?.id ?? null }); })}>+ Step</button>
      {r.steps.some((s) => s.action === "sort") && (
        <div className="rules">
          <span className="lbl">Sorting rules</span>
          {cats.map((c) => {
            const rule = r.sort_rules.find((x) => x.category === c);
            return (
              <label key={c} className="rule"><span>{c} →</span>
                <select value={rule?.zone ?? ""} disabled={disabled} onChange={(e) => save((x) => { x.sort_rules = [...x.sort_rules.filter((y) => y.category !== c), { category: c, zone: e.target.value }]; })}>
                  <option value="">{zones.includes(c) ? `${c} (same name)` : "choose zone"}</option>
                  {zones.map((z) => <option key={z} value={z}>{z}</option>)}
                </select>
              </label>
            );
          })}
        </div>
      )}
      <div className={`badge ${snap.recipe_status.ok ? "ok" : "fail"}`}>{snap.recipe_status.ok ? `Valid for scene revision ${snap.recipe_status.revision}` : snap.recipe_status.error}</div>
    </>
  );
}

function StepCard({ i, step, n, snap, disabled, onChange, onMove, onRemove }: {
  i: number; step: Step; n: number; snap: Snapshot; disabled: boolean; onChange: (s: Step) => void; onMove: (d: number) => void; onRemove: () => void;
}) {
  const movable = snap.scene.parts.filter((p) => p.role === "movable");
  const part = movable.find((p) => p.id === step.part);
  const rec = snap.progress?.records?.find((r) => r.index === i + 1);
  const cur = snap.progress?.current_step === i + 1 && snap.status !== "idle";
  return (
    <div className={`stepcard ${cur ? "active" : ""} ${rec?.outcome ?? ""}`}>
      <div className="num">{i + 1}</div>
      <div className="body">
        <select value={step.part} disabled={disabled} onChange={(e) => onChange({ ...step, part: e.target.value })} title="Part to move">
          {movable.map((p) => <option key={p.id} value={p.id}>{p.name}</option>)}
        </select>
        <div className="line">
          <select value={step.action} disabled={disabled} onChange={(e) => onChange({ ...step, action: e.target.value as Step["action"] })} title="Action">
            <option value="place">place on</option>
            <option value="sort">sort</option>
          </select>
          {step.action === "place" ? (
            <select value={step.dest ?? ""} disabled={disabled} onChange={(e) => onChange({ ...step, dest: e.target.value })} title="Destination">
              <option value="">choose…</option>
              {snap.scene.destinations.map((d) => <option key={d.id} value={d.id}>{d.name}{d.yaw !== null ? ` (${deg(d.yaw).toFixed(0)}°)` : ""}</option>)}
            </select>
          ) : <span className="hint inline">{part?.category ? `“${part.category}” → free slot in its zone` : "part has no category"}</span>}
        </div>
        {rec && rec.outcome !== "running" && <div className={`rec ${rec.outcome}`}>{rec.outcome === "success" ? "✓ " : rec.outcome === "not_run" ? "– " : "✗ "}{rec.reason}</div>}
      </div>
      <div className="ops">
        <button disabled={disabled || i === 0} onClick={() => onMove(-1)} title="Move up">↑</button>
        <button disabled={disabled || i === n - 1} onClick={() => onMove(1)} title="Move down">↓</button>
        <button disabled={disabled} onClick={onRemove} title="Remove">×</button>
      </div>
    </div>
  );
}

// ------------------------------------------------------------------ run / results

function RunProgress({ snap }: { snap: Snapshot | null }) {
  const p = snap?.progress;
  if (!snap || !p) return <p className="hint">Run executes the workflow from the current scene; nothing is reset between steps.</p>;
  const running = snap.status === "running" || snap.status === "paused";
  return (
    <div className="progress">
      <div className="bar"><i style={{ width: `${(100 * p.completed) / Math.max(1, p.total)}%` }} /></div>
      <span>{running ? `Step ${p.current_step ?? "–"} of ${p.total} · ${snap.phase ?? ""}` : `${p.completed} of ${p.total} steps completed`}</span>
    </div>
  );
}

function PhaseStrip({ snap }: { snap: Snapshot | null }) {
  if (!snap || !snap.phases.length) return null;
  const stepNo = snap.progress?.current_step ?? snap.phases[snap.phases.length - 1]?.step ?? null;
  const log = snap.phases.filter((p) => p.step === stepNo);
  const current = snap.phase;
  const failedAt = snap.status === "failed" ? snap.result?.failure_stage : null;
  const done = new Set(log.filter((p) => p.t_end !== null).map((p) => p.phase));
  return (
    <div className="phases">
      {stepNo !== null && <span className="ph stepno">step {stepNo}</span>}
      {PHASES.map((ph) => {
        const cls = ph === failedAt ? "fail" : ph === current ? "active" : done.has(ph) ? "done" : "";
        return <span key={ph} className={`ph ${cls}`}>{ph}</span>;
      })}
    </div>
  );
}

function ResultBanner({ snap }: { snap: Snapshot }) {
  const r = snap.result!;
  const done = r.steps.filter((s) => s.outcome === "success").length;
  if (r.outcome === "success")
    return <div className="banner ok"><b>Workflow complete.</b> {done}/{r.steps.length} steps · {r.sim_duration_s.toFixed(1)} s simulated</div>;
  if (r.outcome === "stopped") return <div className="banner neutral"><b>Stopped</b> at step {r.failed_step ?? "–"} ({r.failure_stage}). {done} step{done === 1 ? "" : "s"} completed.</div>;
  return (
    <div className="banner fail"><b>Step {r.failed_step ?? "–"} failed at {r.failure_stage}</b> <code>{r.failure_code}</code>
      <div>{r.reason}</div><small>{done} of {r.steps.length} steps completed. {hintFor(r.failure_code)}</small></div>
  );
}

function hintFor(code: string | null): string {
  switch (code) {
    case "PERCEPTION_UNAVAILABLE": return "The robot did not move. Check the camera view: parts unmasked, visible, upright, one per color.";
    case "OUT_OF_WORKSPACE": return "Move the part or destination into the shaded workspace band.";
    case "PLAN_FAILED": return "Try a position closer to the robot or away from the table edge.";
    case "ROUTE_BLOCKED": return "Something is in the way of the arm or the carried part. Move it and run again.";
    case "DESTINATION_OCCUPIED": case "NO_FREE_SLOT": return "Clear the destination or add another slot.";
    case "GRASP_FAILED": return "Check the gripper (Developer › Faults) and the part size. Reset to try again.";
    case "PLACE_FAILED": case "DISTURBED": return "The part did not end where planned. Reset and inspect the Developer log.";
    case "CONTACT_VIOLATION": return "The evaluator recorded a forbidden contact. See the Developer log.";
    default: return "Reset to restore the start positions.";
  }
}

function CameraPanel({ snap }: { snap: Snapshot | null }) {
  const [tab, setTab] = useState<"rgb" | "depth">("rgb");
  const c = snap?.camera;
  const id = snap?.scene.camera.id ?? "rgbd";
  const fresh = c && c.age_s < 1.5;
  return (
    <section className="card camera">
      <h3>Camera <small>{snap ? `${snap.scene.camera.width}×${snap.scene.camera.height} · ${snap.scene.camera.rate_hz} Hz` : ""}</small></h3>
      <div className="seg small">
        <button className={tab === "rgb" ? "on" : ""} onClick={() => setTab("rgb")}>RGB</button>
        <button className={tab === "depth" ? "on" : ""} onClick={() => setTab("depth")}>Depth</button>
      </div>
      <div className="frame">
        {c ? <img src={`/api/camera/${id}/${tab}?f=${c.frame_id}`} alt={`${tab} camera frame`} /> : <div className="empty">No frame yet</div>}
        {c?.masked && <div className="frame-tag fail">LENS MASKED</div>}
      </div>
      <div className="kv small"><span>Frame</span><b>#{c?.frame_id ?? "–"} · {c ? `${(c.age_s * 1000).toFixed(0)} ms old` : "unavailable"}</b></div>
      <div className="kv small"><span>Revision / sim time</span><b>{c ? `${c.scene_revision} · ${c.sim_time_s.toFixed(2)} s` : "–"}</b></div>
      {!fresh && c && <div className="badge warn">Camera frame is stale</div>}
    </section>
  );
}

function PerceptionPanel({ snap, mode, onObserve, canObserve }: { snap: Snapshot | null; mode: Mode; onObserve: () => void; canObserve: boolean }) {
  const est = snap?.estimates ?? {};
  const parts = snap?.scene.parts.filter((p) => p.role === "movable") ?? [];
  return (
    <section className="card">
      <h3>Localization <small>{mode === "camera_rgbd" ? "camera RGB-D" : "known world (dev)"}</small></h3>
      {!Object.keys(est).length && <p className="hint">No current estimates. Scene edits invalidate them.</p>}
      {parts.map((p) => {
        const e = est[p.id];
        if (!e) return null;
        const t = snap?.truth[p.id]?.position_m;
        const err = e.valid && e.position_m && t ? Math.hypot(e.position_m[0] - t[0], e.position_m[1] - t[1]) : null;
        return (
          <div key={p.id} className="loc">
            <div className="kv"><span><i className="dot" style={{ background: PALETTE[p.color] }} />{p.name}</span>
              <b>{e.valid && e.position_m ? `${(e.position_m[0] * 1000).toFixed(0)}, ${(e.position_m[1] * 1000).toFixed(0)} mm · ${deg(e.yaw_rad ?? 0).toFixed(0)}°` : "not found"}</b></div>
            {e.valid ? <div className="kv small eval"><span>{e.source === "camera_rgbd" ? `frame #${e.frame_id} · error vs truth (evaluation)` : "simulator pose (dev)"}</span><b>{err !== null ? mm(err) : "unavailable"}</b></div>
              : <div className="badge fail small">{e.reasons.join("; ")}</div>}
          </div>
        );
      })}
      <button className="ghost wide" disabled={!canObserve} onClick={onObserve}>Locate parts now</button>
    </section>
  );
}

function ResultPanel({ snap }: { snap: Snapshot | null }) {
  const r = snap?.result;
  return (
    <section className="card">
      <h3>Result {r && <small>run {r.run_id}</small>}</h3>
      {!r && <p className="hint">Run the workflow to see measured results. Missing metrics show as unavailable.</p>}
      {r && (
        <>
          <div className={`outcome ${r.outcome}`}>{r.outcome === "success" ? `✓ All ${r.steps.length} steps completed` : r.outcome === "stopped" ? "Stopped by user" : `Failed at step ${r.failed_step}: ${r.reason}`}</div>
          <table className="steps-table">
            <thead><tr><th>#</th><th>Part → destination</th><th>Result</th><th>Placed</th><th>Seen</th></tr></thead>
            <tbody>
              {r.steps.map((s) => (
                <tr key={s.index} className={s.outcome}>
                  <td>{s.index}</td>
                  <td>{s.part_name} → {s.dest_name ?? (s.zone ? `${s.zone}` : "–")}</td>
                  <td>{s.outcome === "success" ? "✓" : s.outcome === "not_run" ? "not run" : s.outcome}</td>
                  <td>{s.metrics ? mm(s.metrics.placement_error_xy_m, 1).replace(" mm", "") : "–"}</td>
                  <td>{s.metrics ? mm(s.metrics.estimate_error_xy_m, 1).replace(" mm", "") : "–"}</td>
                </tr>
              ))}
            </tbody>
          </table>
          <p className="hint">Placed = final placement error (mm, ground-truth evaluator). Seen = camera estimate error at pick (mm, evaluation only).</p>
          <div className="kv small"><span>Forbidden contacts</span><b>{String(r.metrics.forbidden_contacts ?? "unavailable")}</b></div>
          <div className="kv small"><span>Duration (simulated / wall)</span><b>{r.sim_duration_s.toFixed(1)} s / {r.wall_duration_s.toFixed(1)} s</b></div>
          <p className="hint">{r.timing_note}.</p>
          <a className="ghost wide link" href={`/api/runs/${r.run_id}/export`}>Export run JSON</a>
        </>
      )}
    </section>
  );
}

function DevDrawer({ snap, onClose, act }: { snap: Snapshot; onClose: () => void; act: Act }) {
  const [tab, setTab] = useState<"controller" | "perception" | "faults" | "log">("controller");
  const idle = EDITABLE.includes(snap.status);
  return (
    <div className="drawer">
      <div className="drawer-head">
        <div className="seg small">
          {(["controller", "perception", "faults", "log"] as const).map((t) => <button key={t} className={tab === t ? "on" : ""} onClick={() => setTab(t)}>{t}</button>)}
        </div>
        <button className="ghost" onClick={onClose}>Close</button>
      </div>
      <div className="drawer-body">
        {tab === "controller" && (
          <div className="grid3">
            <div><h4>Controller in use</h4>
              <div className="kv small"><span>Active</span><b>{snap.controller.name}</b></div>
              <div className="kv small"><span>Fallback</span><b>{snap.controller.fallback ?? "none"}</b></div>
              <div className="kv small"><span>Rate</span><b>{snap.controller.rate_hz.toFixed(0)} Hz</b></div>
              <div className="kv small"><span>Model</span><b>{snap.controller.model}</b></div>
              {Object.entries(snap.controller.gains).map(([k, v]) => <div className="kv small" key={k}><span>{k}</span><b>{v}</b></div>)}
              <div className="kv small"><span>Torque MPC</span><b>planned (not implemented)</b></div>
            </div>
            <div><h4>Timing</h4>
              <div className="kv small"><span>Sim time</span><b>{snap.sim_time_s.toFixed(2)} s</b></div>
              <div className="kv small"><span>Real-time factor</span><b>{snap.realtime_factor?.toFixed(2) ?? "unavailable"}</b></div>
              <div className="kv small"><span>Physics dt</span><b>{(snap.timing.physics_dt_s * 1000).toFixed(1)} ms</b></div>
              <div className="kv small"><span>Step + control</span><b>{snap.timing.step_ms_mean?.toFixed(3) ?? "–"} ms</b></div>
              <div className="kv small"><span>Last scene build</span><b>{snap.timing.build_s.toFixed(2)} s</b></div>
            </div>
            <div><h4>Joints (rad)</h4>
              {snap.q.map((q, i) => <div className="kv small" key={i}><span>q{i + 1}</span><b>{q.toFixed(3)}</b></div>)}
            </div>
          </div>
        )}
        {tab === "perception" && (
          <div className="grid3">
            <div><h4>Estimates (planning input)</h4><pre>{JSON.stringify(snap.estimates, null, 1)}</pre></div>
            <div><h4>Plan (current step)</h4><pre>{JSON.stringify(snap.plan, null, 1)}</pre></div>
            <div><h4>Truth ({snap.truth_label})</h4><pre>{JSON.stringify(snap.truth, null, 1)}</pre></div>
          </div>
        )}
        {tab === "faults" && (
          <div className="faults">
            <p className="hint">Fault injection for negative tests. Changes are accepted while stopped.</p>
            <label><input type="checkbox" checked={snap.faults.camera_masked} disabled={!idle}
              onChange={(e) => act(() => post("/api/faults", { camera_masked: e.target.checked }))} /> Mask camera lens (RGB black, no depth)</label>
            <label><input type="checkbox" checked={snap.faults.gripper_disabled} disabled={!idle}
              onChange={(e) => act(() => post("/api/faults", { gripper_disabled: e.target.checked }))} /> Disable gripper closing</label>
          </div>
        )}
        {tab === "log" && (
          <div className="log">
            {[...snap.events].reverse().map((e, i) => <div key={i} className={`ev ${e.level}`}><span>{e.t_sim.toFixed(2)} s</span>{e.msg}</div>)}
          </div>
        )}
      </div>
    </div>
  );
}
