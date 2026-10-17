"""Main-thread OpenCV display; never issues hardware commands."""
import time
from vision_guard import floor_obstacle


def render_frame(frame, boxes, config, state, action='', info=''):
    import cv2
    display = frame.copy()
    height, width = display.shape[:2]
    def text(value, x, y, color=(255, 255, 255)):
        cv2.putText(display, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(display, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    for box in boxes:
        x, y, w, h = box['bbox']
        left, top = int(x * width), int(y * height)
        right, bottom = int((x + w) * width), int((y + h) * height)
        cv2.rectangle(display, (left, top), (right, bottom), (255, 180, 0), 2)
        cv2.circle(display, ((left + right) // 2, (top + bottom) // 2), 3, (255, 180, 0), -1)
        text('%s target area=%.3f' % (config['vision'].get('target_color', 'target').upper(), box['area']), left, max(12, top - 5), (255, 180, 0))
    rois = [('held', config['feedback'].get('held_roi'), (0, 255, 0)),
            ('placed', config['feedback'].get('placed_roi'), (0, 255, 255)),
            ('corridor', config['obstacle_guard'].get('roi'), (0, 0, 255)),
            ('interaction', config['obstacle_guard'].get('interaction_roi'), (255, 0, 255))]
    for name, roi, color in rois:
        if isinstance(roi, list) and len(roi) == 4:
            x0, y0, x1, y1 = roi
            left, top = int(x0 * width), int(y0 * height)
            cv2.rectangle(display, (left, top), (int(x1 * width), int(y1 * height)), color, 1)
            text(name, left, min(height - 4, top + 14), color)
    vision = config['vision']
    if config.get('lane_following', {}).get('enabled'):
        from lane_control import observe
        lane = observe(frame, config['lane_following'], config['feedback'].get('held_roi'))
        for x0, y0, x1, y1 in lane['boundaries']:
            cv2.line(display, (int(x0*width), int(y0*height)), (int(x1*width), int(y1*height)), (0, 255, 255), 2)
        if lane.get('cone'):
            x,y,w,h=lane['cone']['bbox']
            cv2.rectangle(display,(int(x*width),int(y*height)),(int((x+w)*width),int((y+h)*height)),(0,255,255),2)
            text('YELLOW CONE candidate bottom=%.2f'%lane['cone']['bottom'],int(x*width),max(12,int(y*height)-5),(0,255,255))
        text('LANE valid=%s front=%s error=%s' % (lane['valid'], lane['front'], lane.get('error')), 5, height-12)
    center = int(vision['center_x'] * width)
    cv2.line(display, (center, 120), (center, height - 1), (255, 255, 255), 1)
    grab_y = int(vision['grab_bottom'] * height)
    cv2.line(display, (0, grab_y), (width - 1, grab_y), (255, 255, 255), 1)
    text('STATE: ' + state, 5, 16)
    text('ACTION: ' + (action or '-'), 5, 34)
    import textwrap
    lines = []
    for part in (info or 'Waiting for observation').split(' | '):
        lines.extend(textwrap.wrap(part, max(20, width // 7)))
    for index, line in enumerate(lines[:3]):
        text(line, 5, 52 + 18 * index)
    text('q / ESC: stop | click: HSV + xy', 5, 106)
    return display


class DebugView:
    WINDOW = 'TonyPi competition - camera'

    def __init__(self, camera, config, heartbeat, motion, detector, stop_exception):
        self.camera, self.config, self.heartbeat, self.motion = camera, config, heartbeat, motion
        self.detector, self.stop_exception = detector, stop_exception
        self.last_poll, self.latest, self.callback_ready = 0, None, False
        self.info = ''

    def update(self, info):
        self.info = info

    def click(self, event, x, y, flags, unused):
        cv2 = self.camera.cv2
        if event == cv2.EVENT_LBUTTONDOWN and self.latest is not None:
            height, width = self.latest.shape[:2]
            if 0 <= x < width and 0 <= y < height:
                hsv = cv2.cvtColor(self.latest, cv2.COLOR_BGR2HSV)
                print('[pixel HSV / normalized xy]', hsv[y, x].tolist(),
                      [round(x / float(width), 3), round(y / float(height), 3)])

    def poll(self):
        if time.monotonic() - self.last_poll < .1:
            return
        self.last_poll = time.monotonic()
        frame, stamp = self.camera.read_after(float('-inf'))
        if frame is None:
            return
        observation, mask, boxes = self.detector(frame, self.config['vision'])
        self.latest = frame
        state = self.heartbeat.state
        age = max(0, time.monotonic() - stamp)
        info = self.info + ' | frame age=%.2fs' % age
        guard = self.config['obstacle_guard']
        if guard.get('mode') == 'vision' and isinstance(guard.get('roi'), list):
            try:
                blocked, occupancy, unknown = floor_obstacle(frame, guard, self.config['vision'],
                    self.config['feedback'].get('held_roi'), getattr(self.motion, 'current_kind', 'forward'))
                self.camera.cv2.imshow('TonyPi non-floor mask', unknown)
                info += ' | corridor=%s %.0f%%' % ('BLOCKED' if blocked else 'clear', 100 * occupancy)
            except RuntimeError as exc:
                info += ' | corridor view insufficient'
        if age > self.config['camera_timeout']:
            info = 'STALE CAMERA | ' + info
        display = render_frame(frame, boxes, self.config, state,
                               getattr(self.motion, 'current_action', ''), info)
        cv2 = self.camera.cv2
        cv2.imshow(self.WINDOW, display)
        cv2.imshow('TonyPi target mask', mask)
        if not self.callback_ready:
            cv2.setMouseCallback(self.WINDOW, self.click)
            self.callback_ready = True
        # GTK on the team's Pi returns -1 for unsupported WND_PROP_VISIBLE.
        # That value does not mean the window was closed.
        key = cv2.waitKey(1) & 0xff
        visible = cv2.getWindowProperty(self.WINDOW, cv2.WND_PROP_VISIBLE)
        if key in (27, ord('q')) or visible == 0:
            raise self.stop_exception('debug window stop requested')
