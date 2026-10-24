import { useEffect, useRef, useState } from "react";
import { getJSON, type Estimate, type Snapshot, type Visuals } from "./api";
import { SceneView, type DragTarget } from "./sceneView";

interface Props {
  snap: Snapshot | null;
  editable: boolean;
  selected: DragTarget;
  showCamera: boolean;
  estimate: Estimate | null;
  estimateLabel: string;
  onSelect: (t: DragTarget) => void;
  onDragEnd: (t: DragTarget, x: number, y: number) => void;
  onLoaded: (ok: boolean, message?: string) => void;
}

export default function Viewport(p: Props) {
  const host = useRef<HTMLDivElement>(null);
  const view = useRef<SceneView | null>(null);
  const cbs = useRef(p);
  cbs.current = p;
  const [ready, setReady] = useState(false);
  const loadedRev = useRef<string>("");

  useEffect(() => {
    if (!host.current) return;
    const v = new SceneView(host.current, {
      onDragEnd: (t, x, y) => cbs.current.onDragEnd(t, x, y),
      onSelect: (t) => cbs.current.onSelect(t),
    });
    view.current = v;
    let cancelled = false;
    (async () => {
      try {
        const vis = await getJSON<Visuals>("/api/scene/visuals");
        const r = await fetch("/api/scene/meshes.bin");
        if (!r.ok) throw new Error(`meshes: HTTP ${r.status}`);
        const blob = await r.arrayBuffer();
        if (cancelled) return;
        if (blob.byteLength !== vis.mesh_bytes) throw new Error("mesh payload size mismatch");
        v.build(vis, blob);
        setReady(true);
        cbs.current.onLoaded(true);
      } catch (e) {
        if (!cancelled) cbs.current.onLoaded(false, String(e));
      }
    })();
    return () => {
      cancelled = true;
      v.dispose();
      view.current = null;
    };
  }, []);

  useEffect(() => {
    const v = view.current;
    if (!v || !ready || !p.snap) return;
    v.editable = p.editable;
    v.selected = p.selected;
    v.update(p.snap);
  }, [p.snap, p.editable, p.selected, ready]);

  useEffect(() => {
    if (!view.current || !ready) return;
    const key = `${p.estimate?.frame_id ?? "none"}:${p.estimate?.valid}:${p.estimateLabel}`;
    if (key === loadedRev.current) return;
    loadedRev.current = key;
    view.current.setEstimate(p.estimate, p.estimateLabel);
  }, [p.estimate, p.estimateLabel, ready]);

  useEffect(() => { view.current?.setShowCamera(p.showCamera); }, [p.showCamera, ready]);

  useEffect(() => {
    // backend accepted a new revision: drop the local drag preview
    view.current?.releaseHold();
  }, [p.snap?.scene_revision]);

  return (
    <div className="viewport" ref={host}>
      <div className="view-presets">
        <button onClick={() => view.current?.viewPreset("overview")}>Overview</button>
        <button onClick={() => view.current?.viewPreset("top")}>Top</button>
        <button onClick={() => view.current?.viewPreset("camera")}>Camera</button>
      </div>
    </div>
  );
}
