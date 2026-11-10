#!/usr/bin/env python3
"""Printable sheets: one ArUco marker per page (exact size) + a checkerboard for camera calibration.

    python tools/make_print_sheets.py                          # the map's markers, A4 + checkerboard
    python tools/make_print_sheets.py --ids 13-32 --paper letter   # NEW markers (not in the map yet), US Letter

PRINT AT 100 % ("actual size", no "fit to page"), then check the 10 cm bar with a ruler.
Dictionary and size come from config/markers.yaml (DICT_6X6_50, 20 cm black square).
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

DPI = 300
MM = DPI / 25.4
A4 = (round(210 * MM), round(297 * MM))  # portrait, pixels
LETTER = (round(215.9 * MM), round(279.4 * MM))
PAGE = A4


def font(size_px):
    for name in ("arial.ttf", "DejaVuSans.ttf", "C:/Windows/Fonts/arial.ttf"):
        try:
            return ImageFont.truetype(name, size_px)
        except OSError:
            pass
    return ImageFont.load_default()


def marker_page(dictionary, marker_id, side_mm, dict_name):
    side = round(side_mm * MM)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    if hasattr(cv2.aruco, "generateImageMarker"):
        m = cv2.aruco.generateImageMarker(d, marker_id, side)
    else:
        m = cv2.aruco.drawMarker(d, marker_id, side)
    A = PAGE
    page = Image.new("L", A, 255)
    # a 20 cm marker barely fits the page width: put it near the top, cut guide only if there is room
    big = side > A[0] - round(60 * MM)
    x0, y0 = (A[0] - side) // 2, round((14 if big else 40) * MM)
    page.paste(Image.fromarray(m), (x0, y0))
    dr = ImageDraw.Draw(page)
    q = 0 if big else side // 6  # cut guide 1 module outside the marker (white quiet zone must stay)
    if q:
        dr.rectangle((x0 - q, y0 - q, x0 + side + q, y0 + side + q), outline=170, width=2)
    y = y0 + side + q + round((8 if big else 12) * MM)
    sz = 10 if big else 14
    dr.text((A[0] // 2, y), f"ID {marker_id}   ↑ top: keep upright", fill=0, font=font(round(sz * MM)), anchor="mt")
    dr.text((A[0] // 2, y + round((sz + 4) * MM)), f"{dict_name}  ·  black square = {side_mm:g} mm  ·  print at 100 %",
            fill=0, font=font(round(4.5 * MM)), anchor="mt")
    # 10 cm check bar
    by = y + round((sz + 12) * MM)
    bx0, bx1 = (A[0] - round(100 * MM)) // 2, (A[0] + round(100 * MM)) // 2
    dr.rectangle((bx0, by, bx1, by + round(2 * MM)), fill=0)
    dr.text((A[0] // 2, by + round(4 * MM)), "this bar must measure exactly 100 mm", fill=0, font=font(round(4 * MM)), anchor="mt")
    assert by + round(10 * MM) < A[1], "layout does not fit the page"
    return page.convert("1")


def checkerboard_page(cols=10, rows=7, square_mm=25):
    W, H = A4[1], A4[0]  # landscape
    page = Image.new("L", (W, H), 255)
    sq = round(square_mm * MM)
    x0, y0 = (W - cols * sq) // 2, (H - rows * sq) // 2 - round(5 * MM)
    dr = ImageDraw.Draw(page)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                dr.rectangle((x0 + c * sq, y0 + r * sq, x0 + (c + 1) * sq - 1, y0 + (r + 1) * sq - 1), fill=0)
    dr.text((W // 2, y0 + rows * sq + round(6 * MM)),
            f"Calibration checkerboard · {cols}×{rows} squares of {square_mm} mm · {cols - 1}×{rows - 1} inner corners · print at 100 %, glue on cardboard (flat!)",
            fill=0, font=font(round(4 * MM)), anchor="mt")
    return page.convert("1")


def main():
    root = Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--markers", default=str(root / "config" / "markers.yaml"))
    ap.add_argument("--ids", default="all", help="'all' (the map's markers) or a range, e.g. 13-32 (any IDs)")
    ap.add_argument("--paper", choices=["a4", "letter"], default="a4")
    ap.add_argument("--checkerboard", action="store_true", help="also write the checkerboard")
    ap.add_argument("--out", default=str(root / "docs" / "print"))
    a = ap.parse_args()
    global PAGE
    PAGE = LETTER if a.paper == "letter" else A4
    cfg = yaml.safe_load(open(a.markers, encoding="utf-8"))
    dict_name, size_mm = cfg.get("dictionary", "DICT_4X4_50"), float(cfg.get("marker_size", 0.15)) * 1000
    ids = [m["id"] for m in cfg["markers"]]
    if a.ids != "all":
        lo, hi = map(int, a.ids.split("-"))
        n_max = int(dict_name.rsplit("_", 1)[1])
        if hi >= n_max:
            raise SystemExit(f"{dict_name} has IDs 0-{n_max - 1}")
        clash = [i for i in range(lo, hi + 1) if i in ids]
        if clash:
            print(f"warning: IDs {clash} are already in the map (print them only to replace a damaged marker)")
        ids = list(range(lo, hi + 1))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    pages = [marker_page(None, i, size_mm, dict_name) for i in ids]
    name = f"aruco_markers_{'all' if a.ids == 'all' else a.ids}_{a.paper}.pdf"
    pages[0].save(out / name, save_all=True, append_images=pages[1:], resolution=DPI)
    if a.checkerboard:
        checkerboard_page().save(out / "checkerboard_A4.pdf", resolution=DPI)
    print(f"{len(pages)} marker pages ({size_mm:g} mm, {a.paper}) written to {out / name}")


if __name__ == "__main__":
    main()
