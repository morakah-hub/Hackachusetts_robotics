"""STL import: formats, units, limits, malformed input, eligibility, project bundles."""
import io
import json
import struct
import zipfile
from pathlib import Path

import numpy as np
import pytest

from armstrong import assets as am
from armstrong import projects as pj

EX = Path(__file__).resolve().parents[2] / "examples" / "stl"


@pytest.fixture(autouse=True)
def private_store(tmp_path, monkeypatch):
    monkeypatch.setattr(am, "USER_ASSETS", tmp_path / "assets")
    monkeypatch.setattr(pj, "USER_PROJECTS", tmp_path / "projects")


def test_binary_and_ascii_examples_parse():
    b = am.import_stl((EX / "spacer_40x60x45_mm.stl").read_bytes(), "spacer.stl", "mm")
    a = am.import_stl((EX / "cube_45_ascii_mm.stl").read_bytes(), "cube.stl", "mm")
    assert b["source_format"] == "binary" and a["source_format"] == "ascii"
    assert np.allclose(b["dims_m"], [0.040, 0.060, 0.045], atol=1e-6)
    assert np.allclose(a["dims_m"], [0.045, 0.045, 0.045], atol=1e-6)
    assert b["watertight"] and b["fill_ratio"] > 0.95
    assert b["eligible"]["movable"]["ok"] and a["eligible"]["movable"]["ok"]
    r = am.import_stl((EX / "spacer_40x60x45_mm.stl").read_bytes(), "spacer.stl", "mm", rot_deg=(0, 0, 90))
    assert np.allclose(r["dims_m"], [0.060, 0.040, 0.045], atol=1e-6), "90 deg about z swaps x and y"
    assert r["id"] != b["id"]
    lay = am.import_stl((EX / "spacer_40x60x45_mm.stl").read_bytes(), "spacer.stl", "mm", rot_deg=(90, 0, 0))
    assert np.allclose(lay["dims_m"], [0.040, 0.045, 0.060], atol=1e-6)


def test_units_and_scale():
    data = (EX / "cube_45_ascii_mm.stl").read_bytes()
    cm = am.import_stl(data, "cube.stl", "cm")                      # 450 mm: valid file, not graspable
    assert np.allclose(cm["dims_m"], [0.45] * 3, atol=1e-6)
    assert not cm["eligible"]["movable"]["ok"] and not cm["eligible"]["fixture"]["ok"]
    with pytest.raises(am.AssetError, match="check the units"):
        am.import_stl(data, "cube.stl", "m")                         # 45 m
    half = am.import_stl(data, "cube.stl", "mm", scale=0.5)          # 22.5 mm: too short for the fingers
    assert np.allclose(half["dims_m"], [0.0225] * 3, atol=1e-6)
    assert any("height" in r for r in half["eligible"]["movable"]["reasons"])
    with pytest.raises(am.AssetError):
        am.import_stl(data, "cube.stl", "furlongs")
    with pytest.raises(am.AssetError):
        am.import_stl(data, "cube.stl", "mm", rot_deg=(45, 0, 0))


def test_concave_bracket_is_visual_only():
    m = am.import_stl((EX / "l_bracket_mm.stl").read_bytes(), "bracket.stl", "mm")
    assert m["watertight"] and m["fill_ratio"] < 0.4
    assert not m["eligible"]["movable"]["ok"] and not m["eligible"]["fixture"]["ok"]
    assert m["eligible"]["visual"]["ok"]
    assert any("fills only" in r for r in m["eligible"]["movable"]["reasons"])


def test_malformed_and_oversized_inputs_are_rejected():
    good = (EX / "spacer_40x60x45_mm.stl").read_bytes()
    with pytest.raises(am.AssetError):
        am.parse_stl(b"\x00\x01" * 400)                               # binary noise, size mismatch
    with pytest.raises(am.AssetError):
        am.parse_stl(good[:-7])                                       # truncated binary
    with pytest.raises(am.AssetError):
        am.parse_stl(b"solid x\nfacet normal 0 0 1\nendsolid x\n")    # no vertices
    with pytest.raises(am.AssetError):
        am.parse_stl(b"solid x\n vertex 0 0 0\n vertex 1 0 nan\n vertex 0 1 0\nendsolid\n" * 4)
    big = bytearray(84)
    struct.pack_into("<I", big, 80, am.MAX_TRIANGLES + 1)
    big += b"\x00" * (50 * (am.MAX_TRIANGLES + 1))
    with pytest.raises(am.AssetError, match="limit"):
        am.parse_stl(bytes(big))
    with pytest.raises(am.AssetError, match="larger"):
        am.parse_stl(b"solid " + b" " * (am.MAX_UPLOAD_BYTES + 10))
    assert am.safe_display_name("..\\..\\Windows\\evil.stl") == "evil.stl"
    with pytest.raises(am.AssetError):
        am.asset_path("../../etc/passwd")


def test_bundle_roundtrip_and_tamper_checks():
    meta = am.import_stl((EX / "spacer_40x60x45_mm.stl").read_bytes(), "spacer.stl", "mm", rot_deg=(0, 0, 90))
    proj = pj.load("example", "two_part_kit")
    proj["scene"]["parts"][1]["asset_id"] = meta["id"]
    proj["scene"]["parts"][1]["size"] = meta["dims_m"]
    data = pj.export_bundle(proj)
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        assert sorted(z.namelist()) == ["assets/" + meta["id"] + ".stl", "project.json"]
    (am.USER_ASSETS / f"{meta['id']}.stl").unlink()                   # "another machine" without the asset
    back = pj.import_bundle(data)
    assert back["scene"]["parts"][1]["asset_id"] == meta["id"]
    assert am.asset_path(meta["id"]).is_file()
    assert np.allclose(am.analyze(am.load_triangles(meta["id"]))["dims_m"], meta["dims_m"], atol=1e-6)

    def bundle(entries):
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w") as z:
            for k, v in entries.items():
                z.writestr(k, v)
        return buf.getvalue()
    blob = am.asset_path(meta["id"]).read_bytes()
    pjson = json.dumps(proj)
    with pytest.raises(pj.ProjectError, match="unexpected file"):
        pj.import_bundle(bundle({"project.json": pjson, "../evil.stl": blob}))
    tampered = blob[:-50] + bytes(50)
    with pytest.raises(am.AssetError, match="does not match"):
        pj.import_bundle(bundle({"project.json": pjson, f"assets/{meta['id']}.stl": tampered}))
    with pytest.raises(pj.ProjectError, match="not a ZIP"):
        pj.import_bundle(b"PK-not-really")
    with pytest.raises(pj.ProjectError, match="Armstrong project"):
        pj.import_bundle(bundle({"project.json": json.dumps({"format": "other"})}))


def test_save_and_reopen_user_project():
    proj = pj.load("example", "sort_by_category")
    proj["name"] = "My sorting cell"
    pid = pj.save_user(proj)
    listed = {(p["source"], p["id"]) for p in pj.list_projects()}
    assert ("user", pid) in listed and ("example", "two_part_kit") in listed
    again = pj.load("user", pid)
    assert again["recipe"] == proj["recipe"] and again["scene"] == proj["scene"]
    with pytest.raises(pj.ProjectError):
        pj.load("user", "../secret")
