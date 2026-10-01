"""
V3 camera-LiDAR spatial association (pure Python / numpy, no ROS).

Replaces only the association step of the V2.6 fusion node (the "closest cluster in a bearing
window" rule). Design, with measured motivation in V3_FUSION_VALIDATION_REPORT.md:

1. Projection, not bearing windows. Every LiDAR return is transformed with the calibrated
   extrinsics (URDF: laser_link (0.10, 0, 0.175), camera_link (0.22, 0, 0.35) rpy (0, -0.05, 0)
   in base_link; optical frame z-forward) and projected with the camera intrinsics
   (fx = fy = 493.79, cx = 320, cy = 240 from camera_info). A return is a candidate for a box only if
   its projection lies inside the box horizontally (with a small inward margin) AND vertically.
2. Segments, then depth hypotheses. Candidates are split into angle-contiguous segments (range
   jump > SEG_GAP breaks a segment), and segments at similar depth are merged into one hypothesis,
   so the separate legs / wheels / poles of a sparse object form ONE hypothesis instead of being
   rejected or out-voted.
3. Ground-contact prior (camera-only). Objects of the V3 classes stand on the floor. The ray through
   the bottom-centre pixel of the box intersected with the floor plane gives an independent range
   r_g. Its uncertainty grows with range (sigma = SIG0 + SIG2 * r_g^2). When the box touches the
   bottom image border (object truncated), the floor contact lies below the image, so the object is
   NEARER than where the bottom-row ray meets the floor: this gives an upper-bound prior instead.
4. Scoring, not "closest". Each hypothesis is scored by
     prior agreement  x  horizontal coverage of the box  x  point support  x  background penalty
   where the background penalty applies to surfaces that continue beyond both box edges at the same
   depth (walls, large furniture seen through gaps). The best hypothesis above MIN_SCORE is taken.
5. Outputs: range (20th percentile of the hypothesis = near surface, as V2.6), bearing (median
   beam angle), base_link x/y, association confidence (the score), range_source:
     "lidar"         associated LiDAR hypothesis
     "ground_plane"  no acceptable LiDAR hypothesis, camera ground-contact range used (low confidence)
     "visual_only"   box does not intersect the LiDAR plane (signs, elevated markers)
     "none"          nothing usable
Gazebo ground truth is never an input.
"""
import math
from dataclasses import dataclass

import numpy as np

# ---- calibration (URDF + camera_info) ----
FX = FY = 493.79245758056641
CX, CY = 320.0, 240.0
IMG_W, IMG_H = 640, 480
CAM_T_BASE = np.array([0.22, 0.0, 0.35])
CAM_PITCH = -0.05
LIDAR_T_BASE = np.array([0.10, 0.0, 0.175])
BASE_Z_AGL = 0.06  # base_link height above the floor (base_footprint joint)

# ---- association parameters (chosen on the dev corpus, seed 606; see report) ----
MARGIN_FRAC = 0.0       # inward horizontal margin of the box (fraction of width, >= 2 px)
V_TOL_PX = 4.0           # vertical tolerance for the projected LiDAR-plane point
SEG_GAP = 0.25           # range jump that breaks an angle-contiguous segment (m)
MERGE_GAP = 0.40         # segments whose median ranges differ less than this form one hypothesis (m)
SIG0, SIG2 = 0.15, 0.03  # ground-prior sigma = SIG0 + SIG2 * r^2 (m)
BORDER_PX = 3            # box bottom this close to the image border -> ground prior unavailable
BOUND_TOL, BOUND_SIG = 0.25, 0.20  # truncated boxes: hypotheses beyond bound + tol decay with this sigma
CONTINUE_PX = 18         # look this far outside the box for the same surface
CONTINUE_DR = 0.20       # same-surface tolerance outside the box (m)
MIN_SCORE = 0.15
SPARSE_CLASSES = {"iv_stand", "wheelchair", "surgical_trolley", "cart", "hospital_bed", "person"}
EXPECTED_POINTS = {"iv_stand": 1, "wheelchair": 2, "surgical_trolley": 2, "cart": 2, "hospital_bed": 3, "person": 2}


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


R_BASE_CAM = _rot_y(CAM_PITCH)          # camera_link orientation in base_link
R_OPT_LINK = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])  # camera_link -> optical


def lidar_to_optical(pts_lidar):
    """(N,3) laser_link points -> (N,3) camera optical frame."""
    p_base = pts_lidar + LIDAR_T_BASE
    p_link = (p_base - CAM_T_BASE) @ R_BASE_CAM  # R^T (p - t)
    return p_link @ R_OPT_LINK.T


def project(p_opt):
    z = p_opt[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = CX + FX * p_opt[:, 0] / z
        v = CY + FY * p_opt[:, 1] / z
    return u, v, z


def pixel_ray_base(u, v):
    """Unit direction (base_link) of the optical ray through pixel (u, v)."""
    d_opt = np.array([(u - CX) / FX, (v - CY) / FY, 1.0])
    d_link = R_OPT_LINK.T @ d_opt
    d_base = R_BASE_CAM @ d_link
    return d_base / np.linalg.norm(d_base)


def _floor_point_range(u, v):
    d = pixel_ray_base(u, v)
    if d[2] >= -1e-3:
        return None
    t = (-BASE_Z_AGL - CAM_T_BASE[2]) / d[2]
    rel = CAM_T_BASE + t * d - LIDAR_T_BASE
    return float(math.hypot(rel[0], rel[1])), float(math.atan2(rel[1], rel[0]))


def truncation_bound(box):
    """For a box cut by the bottom image border the object's floor contact lies below the image, i.e.
    NEARER than where the bottom-row ray meets the floor: an upper bound on its range (or None)."""
    x0, y0, x1, y1 = box
    if y1 < IMG_H - BORDER_PX:
        return None
    r = _floor_point_range((x0 + x1) / 2.0, IMG_H - 1)
    return None if r is None else r[0]


def ground_prior(box):
    """Camera-only range (from the LiDAR origin) of the box's floor-contact point, or None."""
    x0, y0, x1, y1 = box
    if y1 >= IMG_H - BORDER_PX:
        return None
    d = pixel_ray_base((x0 + x1) / 2.0, y1)
    if d[2] >= -1e-3:
        return None
    t = (-BASE_Z_AGL - CAM_T_BASE[2]) / d[2]  # floor plane z_base = -0.06
    p = CAM_T_BASE + t * d
    rel = p - LIDAR_T_BASE
    return float(math.hypot(rel[0], rel[1])), float(math.atan2(rel[1], rel[0]))


@dataclass
class ScanPoints:
    angles: np.ndarray
    ranges: np.ndarray
    u: np.ndarray
    v: np.ndarray
    z: np.ndarray

    @staticmethod
    def from_scan(ranges, angle_min, angle_inc, range_min, range_max):
        r = np.array([np.nan if x is None else x for x in ranges], dtype=float)
        a = angle_min + angle_inc * np.arange(len(r))
        ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
        r, a = r[ok], a[ok]
        pts = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
        u, v, z = project(lidar_to_optical(pts))
        front = z > 0.05
        return ScanPoints(a[front], r[front], u[front], v[front], z[front])


def scan_points_from_lidar_xy(pts_xy):
    """ScanPoints from (N,2) laser_link-plane points (e.g. after motion compensation)."""
    pts_xy = np.asarray(pts_xy, dtype=float).reshape(-1, 2)
    r = np.hypot(pts_xy[:, 0], pts_xy[:, 1])
    a = np.arctan2(pts_xy[:, 1], pts_xy[:, 0])
    order = np.argsort(a)
    r, a = r[order], a[order]
    pts = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
    u, v, z = project(lidar_to_optical(pts))
    front = z > 0.05
    return ScanPoints(a[front], r[front], u[front], v[front], z[front])


def _segments(idx, angles, ranges):
    """Angle-contiguous segments (list of index arrays) among candidate indices."""
    if len(idx) == 0:
        return []
    order = idx[np.argsort(angles[idx])]
    segs, cur = [], [order[0]]
    step = np.median(np.diff(np.sort(angles))) if len(angles) > 1 else 0.0087
    for i in order[1:]:
        if abs(ranges[i] - ranges[cur[-1]]) > SEG_GAP or abs(angles[i] - angles[cur[-1]]) > 3.5 * step:
            segs.append(np.array(cur))
            cur = [i]
        else:
            cur.append(i)
    segs.append(np.array(cur))
    return segs


def _hypotheses(segs, ranges):
    """Merge segments at similar depth into hypotheses (list of index arrays)."""
    segs = sorted(segs, key=lambda s: np.median(ranges[s]))
    hyps = []
    for s in segs:
        m = np.median(ranges[s])
        if hyps and abs(m - np.median(ranges[hyps[-1]])) <= MERGE_GAP:
            hyps[-1] = np.concatenate([hyps[-1], s])
        else:
            hyps.append(s)
    return hyps


def associate(box, cls, sp: ScanPoints, with_debug=False):
    x0, y0, x1, y1 = box
    w = x1 - x0
    m = max(2.0, MARGIN_FRAC * w)
    out = {"range_source": "none", "range": None, "bearing": None, "x": None, "y": None,
           "association_confidence": 0.0}
    prior = ground_prior(box)
    bound = truncation_bound(box) if prior is None else None
    out["ground_prior_range"] = None if prior is None else round(prior[0], 3)
    out["truncation_bound"] = None if bound is None else round(bound, 3)

    in_h = (sp.u >= x0 + m) & (sp.u <= x1 - m)
    # a box cut by an image border extends beyond it: projections past that border stay consistent
    lo_v = -np.inf if y0 <= BORDER_PX else y0 - V_TOL_PX
    hi_v = np.inf if y1 >= IMG_H - BORDER_PX else y1 + V_TOL_PX
    in_v = (sp.v >= lo_v) & (sp.v <= hi_v)
    cand = np.nonzero(in_h & in_v)[0]
    horiz_only = np.nonzero(in_h & ~in_v)[0]
    if with_debug:
        out["_cand"] = cand
        out["_rejected_vertical"] = horiz_only

    # LiDAR plane entirely outside the box vertically for every depth -> visual-only object
    if len(cand) == 0 and len(np.nonzero(in_h)[0]) > 0:
        v_at = sp.v[in_h]
        if np.all(v_at > hi_v) or np.all(v_at < lo_v):
            plane_above = np.all(v_at > hi_v)  # box lies entirely above the LiDAR plane
            if plane_above:
                out["range_source"] = "visual_only"
                return out

    best, best_score, scored = None, 0.0, []
    if len(cand):
        hyps = _hypotheses(_segments(cand, sp.angles, sp.ranges), sp.ranges)
        cols = np.arange(math.floor(x0 + m), math.ceil(x1 - m) + 1, 8)
        for h in hyps:
            r_h = np.percentile(sp.ranges[h], 20)
            if prior is not None:
                sig = SIG0 + SIG2 * prior[0] ** 2
                s_prior = math.exp(-0.5 * ((r_h - prior[0]) / sig) ** 2)
            elif bound is not None:
                over = r_h - (bound + BOUND_TOL)
                s_prior = 1.0 if over <= 0 else math.exp(-0.5 * (over / BOUND_SIG) ** 2)
            else:
                s_prior = 0.5
            covered = sum(1 for c in cols if np.any(np.abs(sp.u[h] - c) <= 4))
            s_cov = covered / max(1, len(cols))
            exp_n = EXPECTED_POINTS.get(cls, 3)
            s_sup = min(1.0, len(h) / exp_n)
            # same surface continuing beyond BOTH box edges -> background / wall-like
            med = np.median(sp.ranges[h])
            near = np.abs(sp.ranges - med) <= CONTINUE_DR
            left = np.any(near & (sp.u < x0) & (sp.u >= x0 - CONTINUE_PX))
            right = np.any(near & (sp.u > x1) & (sp.u <= x1 + CONTINUE_PX))
            s_bg = 0.5 if (left and right) else 1.0
            score = s_prior * (0.4 + 0.6 * s_cov) * (0.5 + 0.5 * s_sup) * s_bg
            scored.append((score, h, r_h))
            if score > best_score:
                best, best_score = (h, r_h), score
        if with_debug:
            out["_hypotheses"] = [(float(s), idx, float(r)) for s, idx, r in scored]

    if best is not None and best_score >= MIN_SCORE:
        h, r_h = best
        b = float(np.median(sp.angles[h]))
        out.update(range_source="lidar", range=float(r_h), bearing=b,
                   x=float(LIDAR_T_BASE[0] + r_h * math.cos(b)), y=float(r_h * math.sin(b)),
                   association_confidence=round(float(best_score), 3), n_points=int(len(h)))
        if with_debug:
            out["_selected"] = h
        return out
    if prior is not None:
        r, b = prior
        out.update(range_source="ground_plane", range=r, bearing=b,
                   x=float(LIDAR_T_BASE[0] + r * math.cos(b)), y=float(r * math.sin(b)),
                   association_confidence=round(0.3 * (best_score if best is not None else 0.0) + 0.1, 3))
        return out
    return out
