// Types and helpers for the local Armstrong API (same origin; see vite.config.ts for dev proxy).
// Backend conventions: meters, radians, +Z up, quaternions [w, x, y, z].

export type Status = "loading" | "idle" | "running" | "paused" | "completed" | "failed" | "stopped" | "error";
export type Role = "movable" | "fixture" | "visual";

export interface Estimate {
  id: string;
  valid: boolean;
  reasons: string[];
  position_m: number[] | null;
  yaw_rad: number | null;
  yaw_period_rad?: number | null;
  frame_id?: number;
  scene_revision?: number;
  pixels?: number;
  top_points?: number;
  top_z_m?: number | null;
  top_extent_m?: number[] | null;
  source?: string;
  color?: string;
  placement_error_xy_m?: number;
}

export interface PhaseEntry { phase: string; t_start: number; t_end: number | null; note: string; step: number | null }

export interface PartSpec {
  id: string; name: string; role: Role; asset_id: string | null; size: number[]; mass: number; friction: number;
  material: string | null; color: string; category: string; x: number; y: number; yaw: number;
}
export interface DestSpec { id: string; name: string; x: number; y: number; yaw: number | null; zone: string; size: number[] }
export interface SceneCfg {
  version: number; name: string; parts: PartSpec[]; destinations: DestSpec[];
  camera: { id: string; position: number[]; target: number[]; vertical_fov_deg: number; width: number; height: number; rate_hz: number };
}

export interface Step { id: string; part: string; action: "place" | "sort"; dest?: string | null }
export interface Recipe { version: number; name: string; steps: Step[]; sort_rules: { category: string; zone: string }[] }
export interface RecipeStatus { ok: boolean; error: string | null; code: string | null; steps: number; revision: number }

export interface StepRecord {
  index: number; step_id: string; part_id: string; part_name: string; action: string; dest_id: string | null;
  dest_name: string | null; zone: string | null; outcome: "success" | "failure" | "stopped" | "not_run" | "running";
  failure_stage?: string | null; failure_code?: string | null; reason: string | null;
  estimate?: Estimate | null; verify_estimate?: Estimate | null; metrics?: Record<string, unknown> | null;
  sim_duration_s?: number; wall_duration_s?: number;
}

export interface RunResult {
  run_id: string;
  outcome: "success" | "failure" | "stopped";
  failure_stage: string | null;
  failure_code: string | null;
  failed_step: number | null;
  reason: string;
  perception_mode: string;
  actual_controller: string;
  wall_duration_s: number;
  sim_duration_s: number;
  steps: StepRecord[];
  phases: PhaseEntry[];
  metrics: Record<string, unknown>;
  faults: Record<string, boolean>;
  scene_revision: number;
  timing_note: string;
}

export interface Snapshot {
  status: Status;
  error?: string;
  phase: string | null;
  run_id: string | null;
  scene_revision: number;
  geometry_rev: number;
  sim_time_s: number;
  realtime_factor: number | null;
  controller: { name: string; key: string; gains: Record<string, number>; rate_hz: number; fallback: string | null; model: string };
  q: number[];
  qdot: number[];
  gripper_width_m: number;
  gripper_command: number;
  poses: Record<string, number[]>;
  truth: Record<string, { position_m: number[]; yaw_rad: number }>;
  truth_label: string;
  scene: SceneCfg;
  faults: { camera_masked: boolean; gripper_disabled: boolean };
  estimates: Record<string, Estimate>;
  plan: Record<string, unknown> | null;
  camera: { frame_id: number; scene_revision: number; sim_time_s: number; age_s: number; width: number; height: number; K: number[][]; masked: boolean } | null;
  recipe: Recipe;
  recipe_status: RecipeStatus;
  progress: { current_step: number | null; total: number; completed: number; records: Partial<StepRecord>[] } | null;
  events: { t_sim: number; t_wall: number; level: string; msg: string }[];
  result: RunResult | null;
  timing: { physics_dt_s: number; step_ms_mean: number | null; build_s: number };
  phases: PhaseEntry[];
}

export interface VisualGeom {
  id: string; body: string; kind: "box" | "plane" | "sphere" | "cylinder" | "capsule" | "mesh"; size: number[]; pos: number[];
  quat_wxyz: number[]; rgba: number[]; specular: number; proxy: boolean; mesh?: { offset: number; count: number };
}

export interface Visuals {
  geoms: VisualGeom[];
  bodies: string[];
  parts: { id: string; name: string; role: Role; body: string; size_m: number[]; asset_id: string | null; color: string; category: string; approximation: string }[];
  destinations: { id: string; name: string; body: string; zone: string; yaw: number | null }[];
  camera: { id: string; position: number[]; target: number[]; vertical_fov_deg: number };
  table: { top_z: number; center_xy: number[]; half_xy: number[] };
  workspace: { reach_min: number; reach_max: number; abs_y_max: number };
  mesh_bytes: number;
  geometry_rev: number;
}

export interface AssetMeta {
  id: string; source_name: string; source_format: string; units: string; scale: number; rotation_deg: number[];
  source_triangles: number; dims_m: number[]; triangles: number; watertight: boolean; volume_m3: number | null;
  box_volume_m3: number; fill_ratio: number | null; top_face_fraction: number; collision_proxy: string;
  eligible: Record<Role, { ok: boolean; reasons: string[] }>;
  suggested?: { material: string; mass_kg: number; friction: number };
}

export interface ProjectInfo { id: string; source: "example" | "user"; name: string; description: string; parts: number; steps: number; saved_at?: string }

export class ApiError extends Error {
  code: string;
  constructor(code: string, message: string) {
    super(message);
    this.code = code;
  }
}

async function parse<T>(r: Response): Promise<T> {
  const data = await r.json().catch(() => ({}));
  if (!r.ok) {
    const e = (data as { error?: { code?: string; message?: string } })?.error ?? {};
    throw new ApiError(e.code ?? `HTTP_${r.status}`, e.message ?? r.statusText);
  }
  return data as T;
}

export async function post<T = Record<string, unknown>>(path: string, body: unknown = {}): Promise<T> {
  return parse<T>(await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) }));
}

export async function postBytes<T = Record<string, unknown>>(path: string, data: ArrayBuffer): Promise<T> {
  return parse<T>(await fetch(path, { method: "POST", headers: { "Content-Type": "application/octet-stream" }, body: data }));
}

export async function getJSON<T>(path: string): Promise<T> {
  return parse<T>(await fetch(path));
}

export const PHASES = ["observe", "plan", "approach", "descend", "close", "lift", "transport", "lower", "release", "retreat", "verify"];
export const PALETTE: Record<string, string> = { orange: "#f26b0f", blue: "#2f5df0", green: "#3bb346", magenta: "#cc29a6" };

export function mm(v: unknown, digits = 1): string {
  return typeof v === "number" && Number.isFinite(v) ? `${(v * 1000).toFixed(digits)} mm` : "unavailable";
}

export function newId(prefix: string, taken: Set<string>): string {
  const base = (prefix.toLowerCase().replace(/[^a-z0-9]+/g, "_").replace(/^_+|_+$/g, "") || "item").replace(/^[^a-z]+/, "") || "item";
  const b = base.slice(0, 18);
  if (!taken.has(b)) return b;
  for (let i = 2; i < 100; i++) if (!taken.has(`${b}_${i}`)) return `${b}_${i}`;
  return `${b}_${Date.now() % 10000}`;
}
