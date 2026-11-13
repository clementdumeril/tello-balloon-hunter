# How the drone finds itself with ArUco markers

The drone has no GPS and no motion capture. Its only absolute position sensor is its forward camera looking at
printed markers on the walls. This page explains, step by step, how one camera image becomes the drone's position
and heading in the room. Code: `src/aruco_localization/` (`aruco_localizer.py`, `marker_map.py`).

```
image (960x720) ─► detect markers ─► look up their 3D corners in the map ─► solve the camera pose (PnP)
                   (ids + 4 corners)   (config/markers.yaml)                 (+ checks: reprojection, ambiguity,
                                                                               moved markers, jumps)
                ─► camera → drone body ─► /aruco/pose (+ uncertainty) ─► EKF
```

## 1. What a marker is

An ArUco marker is a square of black and white cells. We use the dictionary `DICT_6X6_50`: a 6 × 6 grid of bits
inside a black border, 50 different patterns (ids 0–49). The patterns are chosen far apart from each other (in
Hamming distance, the number of bits that differ), so a few misread bits are still recognised as the right id, and
no pattern looks like another one rotated by 90°. Each marker therefore gives us its **id** and its **orientation**
(which corner is "top-left").

Our printed markers are 18.89 cm wide (black square). They were meant to be 20 cm, but the printer shrank them by
~6 %; the room scan caught it (see the marker map below), and every distance in the system depends on that number.

## 2. Detecting markers in the image (OpenCV)

`cv2.aruco.detectMarkers` (OpenCV's implementation) does, on the grayscale image:

1. **Adaptive threshold**: black/white image computed against the local brightness (several window sizes), so it
   works with uneven light.
2. **Contours → quadrilaterals**: keep the closed contours that are convex four-sided polygons of reasonable size.
3. **Remove the perspective**: warp each candidate to a front-facing square.
4. **Read the bits**: split the square into the 8 × 8 cells (6 × 6 + border), threshold each cell, check the border
   is all black.
5. **Identify**: compare the 36 bits with the dictionary in the 4 rotations; accept if close enough (few bit
   errors). This gives the id and the corner order.
6. **Refine the corners** to sub-pixel precision (`CORNER_REFINE_SUBPIX`): the pose is only as good as the corners.

Output per marker: its id and its 4 image corners (top-left, top-right, bottom-right, bottom-left), in pixels.

**How small can a marker be?** With the calibrated camera (focal length $f \approx 910$ px), a marker of side
$\ell = 0.189$ m at distance $d$ appears about $f \ell / d$ pixels wide: 170 px at 1 m, 86 px at 2 m, 57 px at 3 m.
Detection is reliable down to a few tens of pixels; precision drops with distance.

## 3. The marker map

`config/markers.yaml` gives, for every marker, its centre $t$ (m) and orientation $R$ (a quaternion) in the room
frame `map` (origin on the floor below marker 49, z up). It was measured once with `tools/room_mapper.py`: ~250
photos of the walls, then a bundle adjustment that finds the marker poses explaining all the photos at once
(reprojection RMS 1.1 px). The scale comes from the printed marker size, cross-checked against the phone scan of the
room (ceiling height 2.80 m, height of marker 49 measured with a tape).

Marker frame (OpenCV convention): origin at the centre, $x$ to the right, $y$ up, $z$ out of the wall towards the
camera. Its 4 corners in the room, in the same order as the detection:

$$
X_i = t + R \begin{bmatrix} \mp \ell/2 \\ \pm \ell/2 \\ 0 \end{bmatrix}
\qquad (\text{top-left} = (-\ell/2, +\ell/2), \ \text{top-right} = (+\ell/2, +\ell/2), \ \dots)
$$

So every detected corner (a 2D point in the image) is matched to a known 3D point in the room.

## 4. From corners to the camera pose (PnP)

The camera is a pinhole with intrinsics $K$ (from `config/tello_camera.yaml`, calibrated with a checkerboard,
RMS 0.24 px) and lens distortion. A room point $X$ projects to the pixel

$$
s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \left( R_{cm} X + t_{cm} \right),
\qquad
K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
$$

where $(R_{cm}, t_{cm})$ is the unknown transform map → camera. **Perspective-n-Point (PnP)** finds it by
minimising the reprojection error over all the corners:

$$
\min_{R_{cm},\, t_{cm}} \ \sum_{\text{corners } i} \left\lVert \begin{bmatrix} u_i \\ v_i \end{bmatrix} - \pi\!\left(K (R_{cm} X_i + t_{cm})\right) \right\rVert^2
$$

**All visible markers in one problem.** With 5 markers in view we solve one PnP with 20 corners, not 5 separate
poses averaged. It is much more precise (the corners are spread over the whole image) and removes the ambiguity of a
single marker (next section). Solver: SQPnP (globally optimal), then Levenberg–Marquardt refinement
(`solvePnPRefineLM`).

**Reprojection error** = the RMS distance (px) between the detected corners and the corners re-projected with the
solved pose. It measures how well the map and the image agree: a fraction of a pixel when all is right.

## 5. One marker alone: the mirror ambiguity

A single small square seen from the front has **two** poses that project almost the same way (the square tilted
one way or the other, "flipped"). OpenCV's IPPE solver returns both. We pick one with these rules, in order:

1. drop poses where the camera would be tilted more than 30° (the Tello camera is level, the drone tilts < 15°);
2. if one pose reprojects clearly better (error ratio ≥ 3), take it;
3. if both put the camera at almost the same place (< 15 cm), take the better one (a head-on view);
4. else take the one closest to the last pose (if < 1 s old and < 0.4 m away);
5. else reject the image.

## 6. Checks before a pose is published

| Check | Rule | Why |
|---|---|---|
| Reprojection error | ≤ 3 px for one marker, ≤ 8 px for several | A 1 cm map error is already ~6 px at 1.6 m; a wrong pose is tens of px off |
| Moved marker | Markers disagree (error > 8 px) and removing one of them fixes it → that one is set aside; after 3 times it is ignored for the run, with a message | Nov 13: marker 39 had fallen and been put back elsewhere; with it in view every pose was rejected (438 px) and the drone flew blind |
| Jump | A single-marker pose more than 0.5 m away from a pose less than 0.5 s old is rejected | Catches a flip the rules of section 5 missed |

## 7. From the camera to the drone, and what is published

The pose solved is the camera's. The drone body frame (`base_link`: $x$ forward, $y$ left, $z$ up) is related to
the camera optical frame ($z$ forward, $x$ right, $y$ down, 5 cm ahead of the body) by a fixed mount:

$$
T^{map}_{cam} = \begin{bmatrix} R_{cm}^\top & -R_{cm}^\top t_{cm} \\ 0 & 1 \end{bmatrix},
\qquad
T^{map}_{base} = T^{map}_{cam} \left(T^{base}_{cam}\right)^{-1},
\qquad
R^{base}_{cam} = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}
$$

Published on `/aruco/pose` (`PoseWithCovarianceStamped`, frame `map`) with an uncertainty that grows with the
reprojection error $e$ (px) and the mean distance $d$ to the markers, and shrinks with their number $n$:

$$
\sigma_p = \max\left(0.01,\ \frac{1.5\, \max(e, 0.3)\, d}{f_x \sqrt{n}}\right) \ \text{m},
\qquad
\sigma_\psi = \max\left(0.5^\circ,\ \frac{\sigma_p}{d}\right)
$$

(typically the 1 cm floor: e.g. $e = 0.5$ px, $d = 2$ m, $n = 4$ gives 0.8 mm before the floor). The EKF uses it to
weigh the marker pose against its prediction.

Also published:
- `/aruco/nearest_marker`: distance to the closest marker in view from its size in the image only,
  $d \approx f \ell / (\text{longest side in px})$. No map, no pose needed: the driver's wall guard uses it.
- `/aruco/debug_image`: the image with the detections drawn and "OK" / "REJECTED" with the error (also saved once
  per second in each flight session, `frames/`).
- TF `map → base_link_aruco`, `/aruco/path` (trail) and `/aruco/sight_lines` (camera → markers used), for RViz.

## 8. Timing

Only the most recent image is processed (queue of 1: never pile up old frames). The image is stamped when it
reaches the PC, so the pose describes the drone ~0.2 s in the past (video + detection, $L_v$). The EKF knows it:
it applies each marker pose at the time the image was taken and re-propagates to now.

## 9. What makes it fail, and what to do

| Symptom | Cause | Fix |
|---|---|---|
| No marker found | Too far, too blurred (fast turn), too dark, or out of the camera's 55° field | Fly facing a marker wall, slower turns, more light |
| "REJECTED" with a large error, several markers in view | A marker moved since the map was made | Handled automatically (section 6); then re-locate it from a flight or re-run `tools/room_mapper.py` |
| Pose jumps between two places with one marker in view | Mirror ambiguity | Keep 2+ markers in view (handled by the rules of section 5) |
| All distances a few % off | Wrong printed marker size in the map | Measure the black square; the map's `marker_size` must match |
| Height disagrees with the distance sensor | Map error (a marker's height), or furniture under the drone | `tools/flight_report.py` reports the ArUco height minus the sensor |
