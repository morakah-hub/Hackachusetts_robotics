// Types and helpers for the local Armstrong API (same origin; see vite.config.ts for dev proxy).
// Backend conventions: meters, radians, +Z up, quaternions [w, x, y, z].

export type Status = "loading" | "idle" | "running" | "paused" | "completed" | "failed" | "stopped" | "error";

export interface Estimate {
  id: string;
  valid: boolean;
  reasons: string[];
  position_m: number[] | null;
  yaw_rad: number | null;
  frame_id?: number;
  scene_revision?: number;
  pixels?: number;
  top_points?: number;
  top_z_m?: number | null;
  top_extent_m?: number[] | null;
  source?: string;
  placement_error_xy_m?: number;
}

export interface PhaseEntry { phase: string; t_start: number; t_end: number | null; note: string }

export interface RunResult {
  run_id: string;
  outcome: "success" | "failure" | "stopped";
  failure_stage: string | null;
  failure_code: string | null;
  reason: string;
  perception_mode: string;
  actual_controller: string;
  wall_duration_s: number;
  sim_duration_s: number;
  phases: PhaseEntry[];
  estimate: Estimate | null;
  verify_estimate: Estimate | null;
  metrics: Record<string, number | string | boolean | null | string[] | Record<string, number>>;
  faults: Record<string, boolean>;
  scene_revision: number;
}

export interface SceneCfg {
  block: { x: number; y: number; yaw: number; size: number[]; mass: number; color: number[] };
  pad: { x: number; y: number; size: number[]; color: number[] };
  camera: { id: string; position: number[]; target: number[]; vertical_fov_deg: number; width: number; height: number; rate_hz: number };
}

export interface Snapshot {
  status: Status;
  error?: string;
  phase: string | null;
  run_id: string | null;
  scene_revision: number;
  sim_time_s: number;
  realtime_factor: number | null;
  controller: { name: string; key: string; gains: Record<string, number>; rate_hz: number; fallback: string | null; model: string };
  q: number[];
  qdot: number[];
  gripper_width_m: number;
  gripper_command: number;
  poses: Record<string, number[]>;
  truth: { block_position_m: number[]; block_yaw_rad: number; label: string };
  scene: SceneCfg;
  faults: { camera_masked: boolean; gripper_disabled: boolean };
  estimate: Estimate | null;
  plan: Record<string, unknown> | null;
  camera: { frame_id: number; scene_revision: number; sim_time_s: number; age_s: number; width: number; height: number; K: number[][]; masked: boolean } | null;
  events: { t_sim: number; t_wall: number; level: string; msg: string }[];
  result: RunResult | null;
  timing: { physics_dt_s: number; step_ms_mean: number | null; build_s: number };
  phases: PhaseEntry[];
}

export interface VisualGeom {
  id: string;
  body: string;
  kind: "box" | "plane" | "sphere" | "cylinder" | "capsule" | "mesh";
  size: number[];
  pos: number[];
  quat_wxyz: number[];
  rgba: number[];
  specular: number;
  mesh?: { offset: number; count: number };
}

export interface Visuals {
  geoms: VisualGeom[];
  bodies: string[];
  camera: { id: string; position: number[]; target: number[]; vertical_fov_deg: number };
  table: { top_z: number; center_xy: number[]; half_xy: number[] };
  workspace: { reach_min: number; reach_max: number; abs_y_max: number };
  mesh_bytes: number;
}

export class ApiError extends Error {
  code: string;
  constructor(code: string, message: string) {
    super(message);
    this.code = code;
  }
}

export async function post<T = Record<string, unknown>>(path: string, body: unknown = {}): Promise<T> {
  const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) {
    const e = data?.error ?? {};
    throw new ApiError(e.code ?? `HTTP_${r.status}`, e.message ?? r.statusText);
  }
  return data as T;
}

export async function getJSON<T>(path: string): Promise<T> {
  const r = await fetch(path);
  if (!r.ok) throw new ApiError(`HTTP_${r.status}`, r.statusText);
  return (await r.json()) as T;
}

export const PHASES = ["observe", "plan", "approach", "descend", "close", "lift", "transport", "lower", "release", "retreat", "verify"];

export function mm(v: unknown, digits = 1): string {
  return typeof v === "number" && Number.isFinite(v) ? `${(v * 1000).toFixed(digits)} mm` : "unavailable";
}
