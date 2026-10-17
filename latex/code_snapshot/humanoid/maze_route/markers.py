#!/usr/bin/env python3
"""Detector for the three maze markers (not a standard ArUco/AprilTag family).

Each marker is a 9x9 grid: a black border two cells wide around 5x5 data cells.
The black square is 14 cm, so one cell is 14/9 cm. Data are matched against the
three known patterns in all four rotations, allowing one wrong cell (the
patterns differ in at least four cells under any rotation).

    python3 markers.py IMAGE...      # prints detections, writes *_markers.jpg
"""
import math

import cv2
import numpy as np

SIZE_CM = 14.0
GRID, BORDER = 9, 2
CELL_PX = 10                       # warped cell size
MAX_DATA_ERR = 1
MAX_BORDER_WHITE = 3               # border cells allowed to read white (glare, crease)

# white = 1, rows top to bottom, as photographed upright on the wall
TEMPLATES = {
    'A': '0110101101101011101111011',   # first wall: walk to it, turn LEFT
    'B': '1101111011101011111111111',   # second wall: walk to it, turn RIGHT
    'C': '0010111110101011101111011',   # last wall: walk to it and stop
}
_TPL = {k: np.array(list(v), np.uint8).reshape(5, 5) for k, v in TEMPLATES.items()}
_ROT = {k: [np.rot90(t, r) for r in range(4)] for k, t in _TPL.items()}


def _order(q):
    """Corners clockwise from top-left (image coordinates)."""
    c = q.mean(0)
    ang = np.arctan2(q[:, 1] - c[1], q[:, 0] - c[0])
    q = q[np.argsort(ang)]                     # clockwise in image coords (y down)
    start = np.argmin(q.sum(1))                # top-left has the smallest x + y
    return np.roll(q, -start, axis=0)


def _read(gray, quad):
    n = GRID * CELL_PX
    M = cv2.getPerspectiveTransform(quad.astype(np.float32),
                                    np.float32([[0, 0], [n, 0], [n, n], [0, n]]))
    w = cv2.warpPerspective(gray, M, (n, n))
    _, b = cv2.threshold(w, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    m = CELL_PX // 4
    cells = np.array([[b[r*CELL_PX + m:(r+1)*CELL_PX - m, c*CELL_PX + m:(c+1)*CELL_PX - m].mean() > .5
                       for c in range(GRID)] for r in range(GRID)], np.uint8)
    ring = np.ones((GRID, GRID), bool)
    ring[BORDER:GRID - BORDER, BORDER:GRID - BORDER] = False
    return cells, int(cells[ring].sum())


class MarkerDetector:
    def __init__(self, K, dist, min_side_px=18):
        self.K, self.dist = K, dist
        self.min_side = min_side_px
        h = SIZE_CM / 2
        self.obj = np.float32([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])  # TL TR BR BL, y up

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        g = cv2.GaussianBlur(gray, (3, 3), 0)
        bw = cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 31, 7)
        contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        found = {}
        for c in contours:
            peri = cv2.arcLength(c, True)
            if peri < 4 * self.min_side:
                continue
            ap = cv2.approxPolyDP(c, 0.04 * peri, True)
            if len(ap) != 4 or not cv2.isContourConvex(ap):
                continue
            q = _order(ap.reshape(4, 2).astype(np.float32))
            sides = [np.linalg.norm(q[i] - q[(i + 1) % 4]) for i in range(4)]
            if min(sides) < self.min_side or max(sides) > 4 * min(sides):
                continue
            cells, border_white = _read(gray, q)
            if border_white > MAX_BORDER_WHITE:
                continue
            data = cells[BORDER:GRID - BORDER, BORDER:GRID - BORDER]
            best = None
            for mid, rots in _ROT.items():
                for r, t in enumerate(rots):
                    err = int((data != t).sum())
                    if best is None or err < best[0]:
                        best = (err, mid, r)
            err, mid, rot = best
            if err > MAX_DATA_ERR:
                continue
            m = self._measure(q)
            m.update(id=mid, err=err, rot=rot * 90, corners=q, side_px=float(np.mean(sides)))
            # several contours (inner/outer edge) can hit one marker: keep the best
            if mid not in found or (err, -m['side_px']) < (found[mid]['err'], -found[mid]['side_px']):
                found[mid] = m
        return found

    def _measure(self, q):
        ok, rvec, tvec = cv2.solvePnP(self.obj, q.reshape(-1, 1, 2), self.K, self.dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        cx, cy = q.mean(0)
        # bearing from the undistorted image position of the marker centre
        u = cv2.undistortPoints(np.float32([[[cx, cy]]]), self.K, self.dist).ravel()
        bearing = math.degrees(math.atan(u[0]))
        t = tvec.ravel()
        R, _ = cv2.Rodrigues(rvec)
        normal = R[:, 2]                      # marker normal (towards the camera) in camera coords
        # Robot heading against the wall the marker hangs on: 0 = square to it,
        # + = robot turned RIGHT of square. Uses only the normal's horizontal part
        # in camera terms so the head pitch does not inflate it.
        yaw = -math.degrees(math.atan2(-normal[0], math.hypot(normal[1], normal[2]))) if ok else None
        return dict(dist_cm=float(np.linalg.norm(t)) if ok else None, x_cm=float(t[0]), z_cm=float(t[2]),
                    bearing_deg=bearing, yaw_deg=yaw, centre=(float(cx), float(cy)))


def draw(frame, found, target=None):
    for mid, m in found.items():
        colour = (0, 255, 0) if mid == target else (0, 200, 255)
        q = m['corners'].astype(int)
        cv2.polylines(frame, [q], True, colour, 3)
        cx, cy = map(int, m['centre'])
        cv2.circle(frame, (cx, cy), 5, colour, -1)
        txt = '%s %.0fcm %+.1fdeg' % (mid, m['dist_cm'] or -1, m['bearing_deg'])
        cv2.putText(frame, txt, (q[:, 0].min(), max(18, q[:, 1].min() - 8)), cv2.FONT_HERSHEY_SIMPLEX, .6, colour, 2)
    return frame


if __name__ == '__main__':
    import sys
    # nominal 640x480 intrinsics if no calibration is given (phone tests)
    for path in sys.argv[1:]:
        img = cv2.imread(path)
        s = 640 / img.shape[1]
        img = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        h, w = img.shape[:2]
        K = np.float64([[w * .8, 0, w / 2], [0, w * .8, h / 2], [0, 0, 1]])
        det = MarkerDetector(K, np.zeros(5))
        found = det.detect(img)
        print(path, {k: (v['err'], v['rot'], round(v['side_px']), round(v['dist_cm'] or -1)) for k, v in found.items()} or 'none')
        cv2.imwrite(path.rsplit('.', 1)[0] + '_markers.jpg', draw(img, found))
