#!/usr/bin/env python3
"""Printable sheets: one ArUco marker per A4 page (exact size) + a checkerboard for camera calibration.

    python tools/make_print_sheets.py            # -> docs/print/aruco_markers_A4.pdf, docs/print/checkerboard_A4.pdf

PRINT AT 100 % ("actual size", no "fit to page"), then check the 10 cm bar with a ruler.
Marker IDs and size match config/markers.yaml (DICT_4X4_50, 15 cm black square).
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
    page = Image.new("L", A4, 255)
    x0, y0 = (A4[0] - side) // 2, round(40 * MM)
    page.paste(Image.fromarray(m), (x0, y0))
    dr = ImageDraw.Draw(page)
    # thin cut guide 1 module outside the marker (white quiet zone must stay)
    q = side // 6
    dr.rectangle((x0 - q, y0 - q, x0 + side + q, y0 + side + q), outline=170, width=2)
    y = y0 + side + q + round(12 * MM)
    dr.text((A4[0] // 2, y), f"ID {marker_id}", fill=0, font=font(round(14 * MM)), anchor="mt")
    dr.text((A4[0] // 2, y + round(18 * MM)), f"{dict_name}  ·  black square = {side_mm:g} mm  ·  print at 100 %",
            fill=0, font=font(round(4.5 * MM)), anchor="mt")
    # 10 cm check bar
    by = y + round(32 * MM)
    bx0, bx1 = (A4[0] - round(100 * MM)) // 2, (A4[0] + round(100 * MM)) // 2
    dr.rectangle((bx0, by, bx1, by + round(2 * MM)), fill=0)
    dr.text((A4[0] // 2, by + round(4 * MM)), "this bar must measure exactly 100 mm", fill=0, font=font(round(4 * MM)), anchor="mt")
    dr.text((A4[0] // 2, y0 - q - round(8 * MM)), "↑ top (keep upright on the wall)", fill=0, font=font(round(4.5 * MM)), anchor="mb")
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
    ap.add_argument("--ids", default="all", help="'all' or e.g. 0-11")
    ap.add_argument("--out", default=str(root / "docs" / "print"))
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.markers, encoding="utf-8"))
    dict_name, size_mm = cfg.get("dictionary", "DICT_4X4_50"), float(cfg.get("marker_size", 0.15)) * 1000
    ids = [m["id"] for m in cfg["markers"]]
    if a.ids != "all":
        lo, hi = map(int, a.ids.split("-"))
        ids = [i for i in ids if lo <= i <= hi]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    pages = [marker_page(None, i, size_mm, dict_name) for i in ids]
    pages[0].save(out / "aruco_markers_A4.pdf", save_all=True, append_images=pages[1:], resolution=DPI)
    checkerboard_page().save(out / "checkerboard_A4.pdf", resolution=DPI)
    print(f"{len(pages)} marker pages ({size_mm:g} mm) and the checkerboard written to {out}")


if __name__ == "__main__":
    main()
