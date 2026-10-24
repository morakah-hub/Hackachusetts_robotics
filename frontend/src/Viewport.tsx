import { useEffect, useRef, useState } from "react";
import { getJSON, type Estimate, type Snapshot, type Visuals } from "./api";
import { SceneView, type DragTarget, type Marker } from "./sceneView";

interface Props {
  snap: Snapshot | null;
  editable: boolean;
  selected: DragTarget | null;
  showCamera: boolean;
  showProxies: boolean;
  estimates: Record<string, Estimate>;
  markers: Marker[];
  onSelect: (t: DragTarget) => void;
  onDragEnd: (t: DragTarget, x: number, y: number) => void;
  onLoaded: (ok: boolean, message?: string) => void;
}

export default function Viewport(p: Props) {
  const host = useRef<HTMLDivElement>(null);
  const view = useRef<SceneView | null>(null);
  const cbs = useRef(p);
  cbs.current = p;
  const [loadedRev, setLoadedRev] = useState(-1);
  const loading = useRef(false);
  const geometryRev = p.snap?.geometry_rev ?? -1;

  useEffect(() => {
    if (!host.current) return;
    const v = new SceneView(host.current, {
      onDragEnd: (t, x, y) => cbs.current.onDragEnd(t, x, y),
      onSelect: (t) => cbs.current.onSelect(t),
    });
    view.current = v;
    return () => {
      v.dispose();
      view.current = null;
    };
  }, []);

  // (Re)load geometry whenever the backend publishes a new compiled model.
  useEffect(() => {
    const v = view.current;
    if (!v || geometryRev < 0 || geometryRev === loadedRev || loading.current) return;
    loading.current = true;
    (async () => {
      try {
        const vis = await getJSON<Visuals>("/api/scene/visuals");
        const r = await fetch("/api/scene/meshes.bin");
        if (!r.ok) throw new Error(`meshes: HTTP ${r.status}`);
        const blob = await r.arrayBuffer();
        if (blob.byteLength !== vis.mesh_bytes || r.headers.get("X-Geometry-Rev") !== String(vis.geometry_rev))
          throw new Error("geometry changed while loading; retrying");
        v.build(vis, blob);
        setLoadedRev(vis.geometry_rev);
        cbs.current.onLoaded(true);
      } catch (e) {
        cbs.current.onLoaded(false, String(e));
        setTimeout(() => setLoadedRev((x) => x - 1000), 800);   // retry
      } finally {
        loading.current = false;
      }
    })();
  }, [geometryRev, loadedRev]);

  const ready = loadedRev >= 0 && loadedRev === geometryRev;

  useEffect(() => {
    const v = view.current;
    if (!v || !ready || !p.snap) return;
    v.editable = p.editable;
    v.selected = p.selected;
    v.update(p.snap);
  }, [p.snap, p.editable, p.selected, ready]);

  const estKey = Object.values(p.estimates).map((e) => `${e.id}:${e.frame_id}:${e.valid}`).join("|");
  useEffect(() => {
    if (!view.current || !ready) return;
    const names = new Map((p.snap?.scene.parts ?? []).map((x) => [x.id, x.name]));
    view.current.setEstimates(p.estimates, (id) => `${names.get(id) ?? id} · seen`);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [estKey, ready]);

  const markerKey = p.markers.map((m) => m.body + m.text).join("|");
  useEffect(() => {
    if (view.current && ready) view.current.setMarkers(p.markers);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [markerKey, ready]);

  useEffect(() => { view.current?.setShowCamera(p.showCamera); }, [p.showCamera, ready]);
  useEffect(() => { view.current?.setShowProxies(p.showProxies); }, [p.showProxies, ready]);
  useEffect(() => { view.current?.releaseHold(); }, [p.snap?.scene_revision]);

  return (
    <div className="viewport" ref={host}>
      <div className="view-presets">
        <button onClick={() => view.current?.viewPreset("overview")}>Overview</button>
        <button onClick={() => view.current?.viewPreset("top")}>Top</button>
        <button onClick={() => view.current?.viewPreset("camera")}>Camera</button>
      </div>
      {!ready && geometryRev >= 0 && loadedRev >= 0 && <div className="geo-updating">Updating scene geometry…</div>}
    </div>
  );
}
