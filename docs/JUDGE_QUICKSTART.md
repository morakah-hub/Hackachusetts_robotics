# Armstrong — judge quickstart (Windows 10/11)

Armstrong runs entirely on your computer: a MuJoCo simulation of a Franka Panda arm, a simulated RGB-D camera that
locates parts from images, and a browser interface. No accounts, API keys, AI services or Kiro installation are
needed. The first start needs internet once to install Python packages (about 120 MB); after that it runs offline.

## 1. Download
- GitHub: **Code → Download ZIP**, or download the prepared release asset `Armstrong-0.2.0.zip` if one is published.
  The release ZIP already contains the built web interface; the source ZIP builds it on first start
  (this downloads the official portable Node.js into the project's `.tools` folder; no global install).

## 2. Extract
Right-click the ZIP → **Extract All…** to any folder (paths with spaces are fine). Do not run it from inside the ZIP.

## 3. Install Python 3.13 if needed
Armstrong is tested with **Python 3.13 (64-bit)**. If `py -3.13 --version` fails in a terminal, install it from
<https://www.python.org/downloads/> and tick **Add python.exe to PATH**. (3.12 and 3.14 have the required packages but
are untested; `start.ps1 -AnyPython` allows them.) A working OpenGL graphics driver is required for the camera.

## 4. Start
Double-click **`start.bat`** in the extracted folder. Or in PowerShell:
```powershell
cd "<extracted folder>"
powershell -ExecutionPolicy Bypass -File .\start.ps1
```
The first run creates `.venv` inside the folder and installs `requirements.txt` (a few minutes). Errors are shown
in the window and logged to `runtime\setup.log`.

## 5. Open the printed address
The window prints e.g. `Armstrong: open http://127.0.0.1:8765/`. If 8765 is busy, the next free port is used and
printed. Keep the window open; Ctrl+C (or closing it) stops Armstrong.

## 6. Run an example workflow (about 1 minute)
1. Left panel → **Project** → choose **Two-part kit** → **Open**.
2. Look at the scene: an orange block and a blue STL spacer; two kit slots. **Workflow** shows two step cards.
3. Press **Run** (bottom left). Watch step 1 and step 2: observe → plan → approach → … → verify. The right side shows
   the live camera (RGB/depth), the image-derived positions, and per-step measured results.
4. Try changes while stopped: drag a part, **Reset**, **Run** again. Or open **Sort by category**.
5. Import your own part: **Parts → Import STL…** → choose `examples/stl/spacer_40x60x45_mm.stl`, units **mm**,
   pick a free color, **Add to scene**, then add a step for it in **Workflow**.
6. Save: **Project → Save** (stored in `user_data/projects`), or **Export bundle** for a portable `.armstrong.zip`.

## 7. Troubleshooting
| Symptom | Fix |
|---|---|
| `BLOCKER: Python 3.13 was not found` | Install Python 3.13 64-bit with "Add to PATH", reopen the window. |
| `pip install ... failed` | Check the internet connection (first run only); see `runtime\setup.log`. |
| Browser shows "Connecting…" | The start window must stay open; check it for errors. |
| Camera panel says "masked" / run fails at observe | Developer → Faults → untick "Mask camera lens". |
| Run fails with ROUTE_BLOCKED / DESTINATION_OCCUPIED | Something is in the way or on the slot: move it and run again. |
| Black or empty camera image | Update the graphics driver (MuJoCo renders with OpenGL). |

## Current limits (honest scope)
- One Panda, one camera, one run at a time; up to 4 movable parts and 8 destinations.
- Movable parts are upright, boxlike, 20–65 mm across the grasp, 35–100 mm tall, up to 0.6 kg, each with its own
  camera color (orange, blue, green, magenta). Physics uses a box collision proxy of an imported mesh; this is an
  approximation of the true shape and is labelled as such. Concave/open meshes can only be visual references.
- Destinations are user-defined goal pads (not perceived). Table, floor and fixtures are known geometry.
- Workflows are fixed recipes (place, kit, grid, sort-by-category). No general assembly, insertion, fastening or
  arbitrary grasp synthesis. Simulated durations are not real-cell throughput claims.
