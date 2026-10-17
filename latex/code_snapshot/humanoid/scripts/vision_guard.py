"""Monocular floor-appearance guard. Detects image occupancy, not metric range."""
import time
from task_feedback import CollisionBlocked
from color_mask import threshold


def floor_obstacle(frame, config, blue, held_roi, phase):
    import cv2
    import numpy as np
    if config.get('panel_geometry'):
        from lane_control import observe
        observation=observe(frame,config['panel_geometry'],held_roi)
        if observation['visible'] < config['min_visible_fraction']:
            raise CollisionBlocked('Panel view obscured; no reliable corridor observation')
        near=config.get('panel_stop_bottom',.85)
        side_close=any(.28<x<.72 and y>=near for x,y in observation['posts'])
        cone=observation.get('cone')
        blocked=(observation['front_bottom']>=near or side_close or
                 (cone is not None and cone['bottom']>=config.get('cone_stop_bottom',.92)))
        # Floor colours, markings and floor-only edges do not trigger this guard.
        mask=np.zeros((240,320),np.uint8)
        for x,y in observation['posts']: cv2.circle(mask,(int(x*320),int(y*240)),4,255,-1)
        return blocked, float(blocked), mask
    image = cv2.resize(frame, (320, 240))
    hsv = cv2.cvtColor(cv2.GaussianBlur(image, (3, 3), 0), cv2.COLOR_BGR2HSV)
    selected = np.zeros((240, 320), dtype=np.uint8)
    def rectangle(roi):
        x0, y0, x1, y1 = roi
        return slice(int(y0 * 240), int(y1 * 240)), slice(int(x0 * 320), int(x1 * 320))
    selected[rectangle(config['roi'])] = 255
    if held_roi:
        selected[rectangle(held_roi)] = 0
    floor = cv2.inRange(hsv, np.array(config['floor_lower_hsv'], dtype=np.uint8),
                        np.array(config['floor_upper_hsv'], dtype=np.uint8))
    unknown = cv2.bitwise_and(cv2.bitwise_not(floor), selected)
    usable = selected.copy()
    # Explicit, local interaction exception: never ignore blue throughout the image.
    interaction = config.get('interaction_roi')
    if interaction and phase in ('approach', 'align_left', 'align_right', 'grab',
                                  'retry_reset', 'release', 'head'):
        space = blue.get('color_space', 'HSV').upper()
        converted = hsv if space == 'HSV' else cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        target = threshold(converted, blue)
        allowed = np.zeros_like(selected)
        allowed[rectangle(interaction)] = target[rectangle(interaction)]
        unknown[allowed > 0] = 0
        usable[allowed > 0] = 0
    visible = cv2.countNonZero(selected)
    total_roi = np.zeros_like(selected)
    total_roi[rectangle(config['roi'])] = 255
    if visible < config['min_visible_fraction'] * cv2.countNonZero(total_roi) or visible == 0:
        raise CollisionBlocked('Camera corridor obscured by held/body ROI; no reliable floor view')
    unknown = cv2.morphologyEx(unknown, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours = cv2.findContours(unknown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    largest = max([cv2.contourArea(c) for c in contours] or [0]) / visible
    occupancy = cv2.countNonZero(unknown) / float(visible)
    # Grayscale silhouettes may match floor colour. Edges provide another cue,
    # but floor seams/shadows can also trigger it; field calibration is required.
    edges = cv2.Canny(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 60, 140)
    edges = cv2.bitwise_and(edges, usable)
    edge_fraction = cv2.countNonZero(edges) / float(visible)
    contours = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    roi_height = max(1, int((config['roi'][3] - config['roi'][1]) * 240))
    tall_edge = any(cv2.boundingRect(c)[3] / float(roi_height) >= config['edge_contour_height']
                    and len(c) >= 4 for c in contours)
    blocked = (largest >= config['contour_fraction'] or occupancy >= config['occupancy_fraction']
               or edge_fraction >= config['edge_fraction'] or tall_edge)
    return blocked, occupancy, unknown


class VisionGuard:
    def __init__(self, config, camera, blue, held_roi):
        self.config, self.camera, self.blue, self.held_roi = config, camera, blue, held_roi
        self.phase, self.last_stamp, self.blocked = 'approach', None, True

    def check(self):
        frame, stamp = self.camera.read_after(float('-inf'))
        if frame is None or stamp is None or not 0 <= time.monotonic() - stamp <= self.config['max_age']:
            raise CollisionBlocked('Camera missing/stale; motion blocked')
        # Re-evaluate after a phase change even when reusing a fresh frame.
        key = (stamp, self.phase)
        if key != self.last_stamp:
            self.blocked, self.occupancy, unused = floor_obstacle(
                frame, self.config, self.blue, self.held_roi, self.phase)
            self.last_stamp = key
        if self.blocked:
            raise CollisionBlocked('Non-floor appearance in camera corridor: %.1f%% occupancy' % (100 * self.occupancy))
