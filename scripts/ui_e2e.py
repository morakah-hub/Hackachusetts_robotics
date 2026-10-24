"""Browser end-to-end check with real clicks (Playwright + the installed Microsoft Edge, headless).

    .\\.venv\\Scripts\\python.exe scripts\\ui_e2e.py [base_url]

Flow: open the "Two-part kit" example from the Project panel -> import examples/stl/cube_45_ascii_mm.stl through the
Import dialog (units mm, green) -> add a workflow step for it -> Run -> wait for the result -> screenshots.
"""
import sys
import time
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent
BASE = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8765"
OUT = ROOT / "runtime"


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


with sync_playwright() as pw:
    browser = pw.chromium.launch(channel="msedge", headless=True, args=["--use-angle=swiftshader", "--enable-unsafe-swiftshader"])
    page = browser.new_page(viewport={"width": 1600, "height": 940})
    page.goto(BASE + "/")
    page.wait_for_selector("text=Connected", timeout=20000)
    log("connected")
    page.click(".acc-head:has-text('Project')")
    page.select_option(".acc-body select", "example:two_part_kit")
    page.click("button:has-text('Open')")
    page.wait_for_selector(".brand .sub:has-text('Two-part kit')", timeout=20000)
    log("opened Two-part kit")
    page.click(".acc-head:has-text('Parts')")
    page.click("button:has-text('Import STL')")
    page.set_input_files(".modal input[type=file]", str(ROOT / "examples" / "stl" / "cube_45_ascii_mm.stl"))
    page.wait_for_selector(".modal .kv:has-text('45.0 × 45.0 × 45.0 mm')", timeout=20000)
    log("import dialog shows 45 x 45 x 45 mm")
    page.screenshot(path=str(OUT / "e2e_import.png"))
    page.select_option(".modal .row2 select", "green")
    page.fill(".modal input[placeholder='Name']", "Green cube")
    page.fill(".modal .placement:has-text('x') >> nth=-1 >> input >> nth=0", "600")
    page.fill(".modal .placement:has-text('x') >> nth=-1 >> input >> nth=1", "150")
    page.click(".modal button:has-text('Add to scene')")
    page.wait_for_selector(".item:has-text('Green cube')", timeout=30000)
    log("Green cube added to the part list")
    page.click(".acc-head:has-text('Destinations')")
    page.click("button:has-text('+ Destination')")
    page.wait_for_selector(".item:has-text('Slot 3')", timeout=30000)
    page.click(".acc-head:has-text('Workflow')")
    page.click("button:has-text('+ Step')")
    page.wait_for_selector(".stepcard >> nth=2", timeout=15000)
    page.wait_for_selector(".badge.ok", timeout=15000)
    log("third step added; workflow valid")
    page.screenshot(path=str(OUT / "e2e_workflow.png"))
    page.click(".runbar button.primary")
    page.wait_for_selector(".pill.status-running", timeout=15000)
    log("running")
    time.sleep(20)
    page.screenshot(path=str(OUT / "e2e_running.png"))
    page.wait_for_selector(".outcome", timeout=150000)
    time.sleep(1.0)
    outcome = page.inner_text(".outcome")
    rows = page.inner_text(".steps-table tbody")
    log("outcome:", outcome)
    print(rows)
    page.screenshot(path=str(OUT / "e2e_done.png"))
    browser.close()
