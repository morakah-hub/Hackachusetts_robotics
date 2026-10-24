import { useEffect, useRef, useState } from "react";
import type { Snapshot } from "./api";

export type Connection = "connecting" | "connected" | "disconnected";

// Live backend state over a WebSocket (~30 Hz), reconnecting with backoff.
export function useSim(): { snap: Snapshot | null; conn: Connection } {
  const [snap, setSnap] = useState<Snapshot | null>(null);
  const [conn, setConn] = useState<Connection>("connecting");
  const retry = useRef(0);

  useEffect(() => {
    let ws: WebSocket | null = null;
    let timer: number | undefined;
    let closed = false;
    const connect = () => {
      const proto = location.protocol === "https:" ? "wss" : "ws";
      ws = new WebSocket(`${proto}://${location.host}/api/ws`);
      ws.onopen = () => {
        retry.current = 0;
        setConn("connected");
      };
      ws.onmessage = (ev) => {
        try {
          setSnap(JSON.parse(ev.data) as Snapshot);
        } catch {
          /* ignore malformed frame */
        }
      };
      ws.onclose = () => {
        if (closed) return;
        setConn("disconnected");
        retry.current = Math.min(retry.current + 1, 6);
        timer = window.setTimeout(connect, 300 * 2 ** retry.current);
      };
      ws.onerror = () => ws?.close();
    };
    connect();
    return () => {
      closed = true;
      if (timer) clearTimeout(timer);
      ws?.close();
    };
  }, []);

  return { snap, conn };
}
