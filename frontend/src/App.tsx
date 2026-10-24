import { useCallback, useEffect, useMemo, useState } from "react";
import { ApiError, mm, PHASES, post, type Estimate, type Snapshot } from "./api";
import { useSim } from "./useSim";
import Viewport from "./Viewport";
import type { DragTarget } from "./sceneView";

type Mode = "camera_rgbd" | "known_world";
const deg = (r: number) => (r * 180) / Math.PI;
const EDITABLE = ["idle", "completed", "failed", "stopped"];

const STATUS_LABEL: Record<string, string> = {
  loading: "Loading", idle: "Ready", running: "Running", paused: "Paused", completed: "Completed",
  failed: "Failed", stopped: "Stopped", error: "Backend error",
};

export default function App() {
  const { snap, conn } = useSim();
  const [mode, setMode] = useState<Mode>("camera_rgbd");
  const [selected, setSelected] = useState<DragTarget>("block");
  const [notice, setNotice] = useState<{ kind: "error" | "info"; text: string } | null>(null);
  const [pendingRev, setPendingRev] = useState<number | null>(null);
  const [busy, setBusy] = useState(false);
  const [devOpen, setDevOpen] = useState(false);
  const [geom, setGeom] = useState<{ ok: boolean; msg?: string } | null>(null);
  const [showCamera, setShowCamera] = useState(true);

  const status = conn !== "connected" ? "disconnected" : snap?.status ?? "loading";
  const editable = !!snap && EDITABLE.includes(snap.status) && conn === "connected";
  const revisionPending = pendingRev !== null && (snap?.scene_revision ?? 0) < pendingRev;

  useEffect(() => {
    if (pendingRev !== null && snap && snap.scene_revision >= pendingRev) setPendingRev(null);
  }, [snap, pendingRev]);

  const act = useCallback(async (fn: () => Promise<Record<string, unknown>>, ok?: string) => {
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

  const edit = useCallback((body: Record<string, unknown>) =>
    act(() => post("/api/scene/edit", { ...body, expected_revision: snap?.scene_revision })), [act, snap?.scene_revision]);

  const onDragEnd = useCallback((t: DragTarget, x: number, y: number) => {
    void edit(t === "block" ? { block: { x, y } } : { pad: { x, y } });
  }, [edit]);

  const run = () => act(() => post("/api/run", {
    type: "pick_place", source_object_ids: ["block"], target_object_id: "pad",
    perception_mode: mode, scene_revision: snap?.scene_revision, task_id: `task-${Date.now()}`,
  }), undefined).then((r) => r && setNotice(null));
  const control = (action: string, extra: Record<string, unknown> = {}) => act(() => post("/api/control", { action, ...extra }));

  const result = snap?.result ?? null;
  const showResult = result && snap && ["completed", "failed", "stopped"].includes(snap.status);
  const est: Estimate | null = snap?.estimate ?? null;
  const estimateLabel = est?.source?.startsWith("known_world") ? "known-world pose (dev)" : "camera estimate";

  return (
    <div className="app">
      <header className="topbar">
        <div className="brand">
          <svg viewBox="0 0 32 32" width="26" height="26"><rect width="32" height="32" rx="7" fill="#24282b" /><path d="M8 24 L14 9 L19 17 L24 13" stroke="#2fc4b2" strokeWidth="3" fill="none" strokeLinecap="round" strokeLinejoin="round" /><circle cx="24" cy="13" r="2.6" fill="#f08a24" /></svg>
          <span className="name">Armstrong</span>
          <span className="sub">Tabletop workcell</span>
        </div>
        <div className="top-right">
          <span className={`pill conn-${conn}`}><i />{conn === "connected" ? "Connected" : conn === "connecting" ? "Connecting…" : "Disconnected — retrying"}</span>
          <span className={`pill status-${status}`}><i />{STATUS_LABEL[status] ?? status}{snap?.phase ? ` · ${snap.phase}` : ""}</span>
          <span className="pill muted">rev {snap?.scene_revision ?? "–"}</span>
          <button className={`ghost ${devOpen ? "on" : ""}`} onClick={() => setDevOpen((v) => !v)}>Developer</button>
        </div>
      </header>

      <aside className="sidebar">
        <section className="card">
          <h3>Robot</h3>
          <select value="panda" onChange={() => undefined}>
            <option value="panda">Franka Emika Panda</option>
            <option disabled>Universal Robots UR5e — Planned</option>
            <option disabled>UFACTORY xArm 7 — Planned</option>
          </select>
          <div className="kv"><span>Controller</span><b>{snap?.controller.name ?? "–"}</b></div>
          <div className="kv"><span>Gripper</span><b>{snap ? mm(snap.gripper_width_m, 1) : "–"}</b></div>
        </section>

        <section className="card">
          <h3>Scene <small>Demo: tabletop pick &amp; place</small></h3>
          <div className="seg">
            <button className={selected === "block" ? "on" : ""} onClick={() => setSelected("block")}><span className="sw orange" />Block</button>
            <button className={selected === "pad" ? "on" : ""} onClick={() => setSelected("pad")}><span className="sw teal" />Pad</button>
          </div>
          {snap && <PlacementEditor key={`${selected}-${snap.scene_revision}`} snap={snap} target={selected} disabled={!editable || busy} onApply={edit} />}
          <p className="hint">{editable ? "Drag the block or pad on the table, or type a position. The shaded band is the validated workspace." : "Scene edits are accepted while stopped."}</p>
          <button className="ghost wide" disabled={!editable || busy} onClick={() => control("reset", { default_scene: true })}>Load demo scene</button>
        </section>

        <section className="card">
          <h3>Task</h3>
          <label className="field"><span>Pick</span><select value="block" onChange={() => undefined}><option value="block">Orange block · 50×50×60 mm</option></select></label>
          <label className="field"><span>Place on</span><select value="pad" onChange={() => undefined}><option value="pad">Destination pad (user-defined goal)</option></select></label>
          <label className="field"><span>Locate by</span>
            <select value={mode} onChange={(e) => setMode(e.target.value as Mode)} disabled={!editable}>
              <option value="camera_rgbd">Camera RGB-D (image-derived)</option>
              <option value="known_world">Known world — development only</option>
            </select>
          </label>
          {mode === "known_world" && <div className="badge warn">DEV MODE: plans from simulator pose, not the camera</div>}
          <div className="controls">
            <button className="primary" disabled={!editable || busy || revisionPending} onClick={run}>{revisionPending ? "Waiting for scene…" : "Run"}</button>
            {snap?.status === "paused"
              ? <button onClick={() => control("resume")}>Resume</button>
              : <button disabled={snap?.status !== "running"} onClick={() => control("pause")}>Pause</button>}
            <button disabled={!(snap?.status === "running" || snap?.status === "paused")} onClick={() => control("stop")}>Stop</button>
            <button disabled={busy || !snap || snap.status === "loading"} onClick={() => control("reset")}>Reset</button>
          </div>
          {snap?.status === "paused" && (
            <div className="controls small">
              <button onClick={() => control("step", { steps: 5 })}>Step 10 ms</button>
              <button onClick={() => control("step", { steps: 50 })}>Step 100 ms</button>
            </div>
          )}
          <p className="hint">Reset returns the arm home and the block to its configured start. Pause freezes physics time.</p>
        </section>
      </aside>

      <main className="stage">
        <Viewport snap={snap} editable={editable} selected={selected} showCamera={showCamera} estimate={est} estimateLabel={estimateLabel}
          onSelect={setSelected} onDragEnd={onDragEnd} onLoaded={(ok, msg) => setGeom({ ok, msg })} />
        {(!geom || conn !== "connected") && (
          <div className="overlay-center">
            {conn !== "connected" ? <><div className="spinner" />Connecting to the local backend…</>
              : <><div className="spinner" />Loading robot geometry…</>}
          </div>
        )}
        {geom && !geom.ok && <div className="overlay-center error">Could not load scene geometry: {geom.msg}</div>}
        <PhaseStrip snap={snap} />
        {notice && (
          <div className={`toast ${notice.kind}`} onClick={() => setNotice(null)}>{notice.text}<span className="x">×</span></div>
        )}
        {showResult && result && <ResultBanner snap={snap!} />}
        <label className="cam-toggle"><input type="checkbox" checked={showCamera} onChange={(e) => setShowCamera(e.target.checked)} /> camera</label>
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

function PlacementEditor({ snap, target, disabled, onApply }: { snap: Snapshot; target: DragTarget; disabled: boolean; onApply: (b: Record<string, unknown>) => Promise<unknown> }) {
  const src = target === "block" ? snap.scene.block : snap.scene.pad;
  const [x, setX] = useState((src.x * 1000).toFixed(0));
  const [y, setY] = useState((src.y * 1000).toFixed(0));
  const [yaw, setYaw] = useState(target === "block" ? deg(snap.scene.block.yaw).toFixed(0) : "0");
  const apply = () => {
    const vx = Number(x) / 1000, vy = Number(y) / 1000, vyaw = (Number(yaw) * Math.PI) / 180;
    if (![vx, vy, vyaw].every(Number.isFinite)) return;
    void onApply(target === "block" ? { block: { x: vx, y: vy, yaw: vyaw } } : { pad: { x: vx, y: vy } });
  };
  return (
    <div className="placement">
      <label><span>x</span><input value={x} onChange={(e) => setX(e.target.value)} disabled={disabled} inputMode="numeric" /><em>mm</em></label>
      <label><span>y</span><input value={y} onChange={(e) => setY(e.target.value)} disabled={disabled} inputMode="numeric" /><em>mm</em></label>
      {target === "block" && <label><span>yaw</span><input value={yaw} onChange={(e) => setYaw(e.target.value)} disabled={disabled} inputMode="numeric" /><em>°</em></label>}
      <button disabled={disabled} onClick={apply}>Apply</button>
    </div>
  );
}

function PhaseStrip({ snap }: { snap: Snapshot | null }) {
  const log = snap?.phases ?? [];
  const current = snap?.phase;
  const failedAt = snap?.status === "failed" ? snap.result?.failure_stage : null;
  const done = new Set(log.filter((p) => p.t_end !== null).map((p) => p.phase));
  if (!snap || (!log.length && snap.status === "idle")) return null;
  return (
    <div className="phases">
      {PHASES.map((ph) => {
        const cls = ph === failedAt ? "fail" : ph === current ? "active" : done.has(ph) ? "done" : "";
        return <span key={ph} className={`ph ${cls}`}>{ph}</span>;
      })}
    </div>
  );
}

function ResultBanner({ snap }: { snap: Snapshot }) {
  const r = snap.result!;
  const m = r.metrics;
  if (r.outcome === "success")
    return <div className="banner ok"><b>Placed.</b> Measured placement error {mm(m.placement_error_xy_m)} · lifted {mm(m.max_lift_m, 0)} · {r.sim_duration_s.toFixed(1)} s</div>;
  if (r.outcome === "stopped") return <div className="banner neutral"><b>Stopped</b> during {r.failure_stage}. Reset or edit the scene, then run again.</div>;
  return <div className="banner fail"><b>Failed at {r.failure_stage}</b> <code>{r.failure_code}</code><div>{r.reason}</div><small>{hintFor(r.failure_code)}</small></div>;
}

function hintFor(code: string | null): string {
  switch (code) {
    case "PERCEPTION_UNAVAILABLE": return "The robot did not move. Check the camera view (unmasked, block visible and upright), then run again.";
    case "OUT_OF_WORKSPACE": return "Move the block or pad into the shaded workspace band.";
    case "PLAN_FAILED": return "Try a position closer to the robot or away from the table edge.";
    case "GRASP_FAILED": return "Check the gripper (Developer › Faults) and that the block is upright. Reset to try again.";
    case "PLACE_FAILED": return "The block did not end on the pad. Reset and run again; inspect the Developer log.";
    case "CONTACT_VIOLATION": return "The evaluator recorded a forbidden contact. See the Developer log.";
    default: return "Reset to restore the scene.";
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
      {tab === "depth" && <p className="hint">Depth display: near = light, far = dark, black = no return. Metric depth stays in the backend.</p>}
    </section>
  );
}

function PerceptionPanel({ snap, mode, onObserve, canObserve }: { snap: Snapshot | null; mode: Mode; onObserve: () => void; canObserve: boolean }) {
  const e = snap?.estimate;
  const truth = snap?.truth.block_position_m;
  const err = e?.valid && e.position_m && truth ? Math.hypot(e.position_m[0] - truth[0], e.position_m[1] - truth[1]) : null;
  return (
    <section className="card">
      <h3>Localization <small>{mode === "camera_rgbd" ? "camera RGB-D" : "known world (dev)"}</small></h3>
      {!e && <p className="hint">No current estimate. Edits invalidate the previous one.</p>}
      {e && e.valid && e.position_m && (
        <>
          <div className="kv"><span>Block at</span><b>{(e.position_m[0] * 1000).toFixed(1)}, {(e.position_m[1] * 1000).toFixed(1)} mm</b></div>
          <div className="kv"><span>Yaw (mod 90°)</span><b>{deg(e.yaw_rad ?? 0).toFixed(1)}°</b></div>
          {e.source === "camera_rgbd" && <div className="kv small"><span>Pixels / top-face points</span><b>{e.pixels} / {e.top_points}</b></div>}
          <div className="kv small"><span>Source</span><b>{e.source === "camera_rgbd" ? `frame #${e.frame_id}` : "simulator pose (dev)"}</b></div>
          <div className="kv small eval"><span>Error vs truth (evaluation)</span><b>{err !== null ? mm(err) : "unavailable"}</b></div>
        </>
      )}
      {e && !e.valid && <div className="badge fail">Invalid observation: {e.reasons.join("; ")}</div>}
      <button className="ghost wide" disabled={!canObserve} onClick={onObserve}>Locate block now</button>
    </section>
  );
}

function ResultPanel({ snap }: { snap: Snapshot | null }) {
  const r = snap?.result;
  const rows = useMemo(() => {
    if (!r) return [];
    const m = r.metrics;
    return [
      ["Outcome", r.outcome === "success" ? "Success" : r.outcome === "stopped" ? "Stopped" : `Failed (${r.failure_stage})`],
      ["Placement error (xy)", mm(m.placement_error_xy_m)],
      ["Height error", mm(m.placement_error_z_m)],
      ["Final tilt", typeof m.final_tilt_deg === "number" ? `${(m.final_tilt_deg as number).toFixed(1)}°` : "unavailable"],
      ["Max lift", mm(m.max_lift_m, 0)],
      ["Slip in hand", mm(m.object_slip_m)],
      ["Grasp width", mm(m.grasp_width_m)],
      ["Estimate error", mm(m.estimate_error_xy_m)],
      ["Forbidden contacts", String(m.forbidden_contacts ?? "unavailable")],
      ["Duration (sim / wall)", `${r.sim_duration_s.toFixed(1)} s / ${r.wall_duration_s.toFixed(1)} s`],
    ];
  }, [r]);
  return (
    <section className="card">
      <h3>Result {r && <small>run {r.run_id}</small>}</h3>
      {!r && <p className="hint">Run the task to see measured results. Missing metrics show as unavailable.</p>}
      {r && (
        <>
          <div className={`outcome ${r.outcome}`}>{r.outcome === "success" ? "✓ Block placed on pad" : r.outcome === "stopped" ? "Stopped by user" : r.reason}</div>
          {rows.map(([k, v]) => <div className="kv small" key={k}><span>{k}</span><b>{v}</b></div>)}
          <p className="hint">Measured by the ground-truth evaluator, separate from the camera estimate.</p>
          <a className="ghost wide link" href={`/api/runs/${r.run_id}/export`}>Export run JSON</a>
        </>
      )}
    </section>
  );
}

function DevDrawer({ snap, onClose, act }: { snap: Snapshot; onClose: () => void; act: (fn: () => Promise<Record<string, unknown>>, ok?: string) => Promise<unknown> }) {
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
              <div className="kv small"><span>Scene compile</span><b>{snap.timing.build_s.toFixed(2)} s</b></div>
            </div>
            <div><h4>Joints (rad)</h4>
              {snap.q.map((q, i) => <div className="kv small" key={i}><span>q{i + 1}</span><b>{q.toFixed(3)}</b></div>)}
            </div>
          </div>
        )}
        {tab === "perception" && (
          <div className="grid3">
            <div><h4>Estimate (planning input)</h4><pre>{JSON.stringify(snap.estimate, null, 1)}</pre></div>
            <div><h4>Plan</h4><pre>{JSON.stringify(snap.plan, null, 1)}</pre></div>
            <div><h4>Truth (evaluation/display only)</h4><pre>{JSON.stringify(snap.truth, null, 1)}</pre>
              <h4>Camera</h4><pre>{JSON.stringify(snap.camera ? { K: snap.camera.K, frame: snap.camera.frame_id } : null)}</pre></div>
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
