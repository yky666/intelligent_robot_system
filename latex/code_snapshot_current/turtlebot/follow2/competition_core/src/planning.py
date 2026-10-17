"""Geometry and fail-closed guards independent of ROS and hardware."""
import math


def finite(value):
    return not math.isnan(value) and not math.isinf(value)


def fresh(stamp, now, timeout):
    return stamp is not None and 0 <= now - stamp <= timeout


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Trail(object):
    """Keep a bounded metric trail; choose a point behind the measured target.

    Arc-length spacing preserves corners, unlike offsetting along instantaneous
    target yaw. Yaw of the AR marker is deliberately not used.
    """
    def __init__(self, spacing=0.08, max_length=20.0, jump_limit=0.8):
        self.spacing = spacing
        self.max_length = max_length
        self.jump_limit = jump_limit
        self.points = []

    def add(self, x, y):
        p = (float(x), float(y))
        if not all(finite(v) for v in p):
            return False
        if self.points:
            step = distance(self.points[-1], p)
            if step > self.jump_limit:
                return False
            if step < self.spacing:
                return True
        self.points.append(p)
        length = sum(distance(a, b) for a, b in zip(self.points, self.points[1:]))
        while len(self.points) > 2 and length > self.max_length:
            length -= distance(self.points[0], self.points[1])
            self.points.pop(0)
        return True

    def behind(self, gap):
        if gap < 0:
            raise ValueError('gap must be nonnegative')
        remaining = gap
        for i in range(len(self.points) - 1, 0, -1):
            a, b = self.points[i - 1], self.points[i]
            length = distance(a, b)
            if length >= remaining and length > 0:
                ratio = remaining / length
                return (b[0] + ratio * (a[0] - b[0]),
                        b[1] + ratio * (a[1] - b[1]),
                        math.atan2(b[1] - a[1], b[0] - a[0]))
            remaining -= length
        return None  # do not drive toward a target before enough trail exists


def grid_clear(data, width, height, resolution, origin_x, origin_y,
               origin_yaw, x, y, radius, threshold=65):
    """Conservative circular footprint check including cell area, unknown = blocked.

    OccupancyGrid values are 0..100 (not costmap's internal 0..255).
    """
    if resolution <= 0 or width <= 0 or height <= 0 or len(data) != width * height:
        return False
    dx, dy = x - origin_x, y - origin_y
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    gx, gy = c * dx + s * dy, -s * dx + c * dy
    padding = radius + resolution * math.sqrt(2) / 2
    ix0, ix1 = int(math.floor((gx - padding) / resolution)), int(math.floor((gx + padding) / resolution))
    iy0, iy1 = int(math.floor((gy - padding) / resolution)), int(math.floor((gy + padding) / resolution))
    if ix0 < 0 or iy0 < 0 or ix1 >= width or iy1 >= height:
        return False
    for iy in range(iy0, iy1 + 1):
        for ix in range(ix0, ix1 + 1):
            cx, cy = (ix + 0.5) * resolution, (iy + 0.5) * resolution
            if math.hypot(cx - gx, cy - gy) <= padding:
                value = data[iy * width + ix]
                if value < 0 or value >= threshold:
                    return False
    return True


def localization_ok(covariance, xy_limit, yaw_limit):
    return len(covariance) == 36 and all(
        finite(covariance[i]) and 0 <= covariance[i] <= limit
        for i, limit in ((0, xy_limit), (7, xy_limit), (35, yaw_limit)))


def scan_clear(ranges, angle_min, angle_increment, range_min, range_max,
               stop_distance, half_angle, min_samples=3):
    valid = []
    for index, value in enumerate(ranges):
        angle = angle_min + index * angle_increment
        if abs(angle) <= half_angle and finite(value) and range_min <= value <= range_max:
            valid.append(value)
    # NaN/Inf-only sectors are treated as missing data, not clear space.
    return len(valid) >= min_samples and min(valid) > stop_distance


def clamp(value, bound):
    return max(-bound, min(bound, value))
