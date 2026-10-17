"""Depth ROI diagnostics. An ROI is NOT an identified or tracked robot."""
import math


def finite(value):
    return not math.isnan(value) and not math.isinf(value)


def roi_bounds(roi, width, height):
    if (width <= 0 or height <= 0 or len(roi) != 4 or
            not all(finite(v) for v in roi) or
            not 0 <= roi[0] < roi[2] <= 1 or not 0 <= roi[1] < roi[3] <= 1):
        raise ValueError('ROI requires normalized [left, top, right, bottom] inside 0..1')
    x0, y0 = int(roi[0] * width), int(roi[1] * height)
    x1, y1 = int(roi[2] * width), int(roi[3] * height)
    if x1 <= x0 or y1 <= y0:
        raise ValueError('ROI is smaller than one pixel')
    return x0, y0, x1, y1


def check_pair(stamps, now, max_age, slop, frames, sizes, encoding):
    """Reject stale/unsynchronized and visibly incompatible registered input.

    Matching frames and dimensions cannot prove geometric registration.
    The operator must inspect object/depth edges before using the measurements.
    """
    if not all(finite(s) and s > 0 and 0 <= now - s <= max_age for s in stamps):
        raise ValueError('stale_or_future_input')
    if max(stamps) - min(stamps) > slop:
        raise ValueError('unsynchronized_input')
    if not frames[0] or len(set(f.lstrip('/') for f in frames)) != 1:
        raise ValueError('registered_frame_mismatch')
    if len(set(sizes)) != 1 or min(sizes[0]) <= 0:
        raise ValueError('registered_size_mismatch')
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError('unsupported_depth_encoding')


def summarize(values, encoding, min_depth=0.4, max_depth=4.0,
              min_samples=30, min_ratio=0.5, max_spread=0.25):
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError('unsupported_depth_encoding')
    if (not all(finite(v) for v in (min_depth, max_depth, min_ratio, max_spread)) or
            not 0 < min_depth < max_depth or not 0 < min_ratio <= 1 or
            min_samples < 1 or max_spread <= 0):
        raise ValueError('invalid_depth_thresholds')
    scale = 0.001 if encoding == '16UC1' else 1.0
    total = 0
    valid = []
    for value in values:
        total += 1
        z = float(value) * scale
        if finite(z) and min_depth <= z <= max_depth:
            valid.append(z)
    valid.sort()
    ratio = len(valid) / float(total) if total else 0.0
    result = dict(samples=total, valid_samples=len(valid), valid_ratio=ratio,
                  median_m=None, spread_m=None, usable=False, reason='insufficient_depth')
    if valid:
        def quantile(q):
            index = (len(valid) - 1) * q
            lo, hi = int(math.floor(index)), int(math.ceil(index))
            return valid[lo] + (valid[hi] - valid[lo]) * (index - lo)
        result['median_m'] = quantile(0.5)
        result['spread_m'] = quantile(0.9) - quantile(0.1)
    if len(valid) >= min_samples and ratio >= min_ratio:
        result['usable'] = result['spread_m'] <= max_spread
        result['reason'] = 'roi_depth_ok' if result['usable'] else 'mixed_depths'
    return result
