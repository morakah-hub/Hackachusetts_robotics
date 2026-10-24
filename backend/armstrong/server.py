"""Entry point: `python -m armstrong.server [--port 8765]`.

The simulation worker (MuJoCo + OpenGL renderer contexts) runs on the main thread; Uvicorn
serves the API and the built frontend from a background thread, bound to 127.0.0.1 only.
"""

from __future__ import annotations

import argparse
import threading
import time

import uvicorn

from .api import create_app
from .sim import SimWorker


def main() -> None:
    ap = argparse.ArgumentParser(description="Armstrong local workcell server")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()
    if args.host not in ("127.0.0.1", "localhost"):
        raise SystemExit("Armstrong is a local single-user demo; bind to 127.0.0.1 only.")
    worker = SimWorker()
    app = create_app(worker)
    config = uvicorn.Config(app, host=args.host, port=args.port, log_level="warning", ws="auto")
    server = uvicorn.Server(config)
    server.install_signal_handlers = lambda: None  # signals are handled by the main thread below
    t = threading.Thread(target=server.run, name="armstrong-http", daemon=True)
    t.start()
    for _ in range(100):
        if server.started:
            break
        time.sleep(0.05)
    print(f"Armstrong is running at http://{args.host}:{args.port}/  (Ctrl+C to stop)", flush=True)
    try:
        worker.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.should_exit = True
        t.join(3)


if __name__ == "__main__":
    main()
