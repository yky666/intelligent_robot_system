"""Image-space corridor guidance. Requires field calibration; no metric range."""
import math
import time


def cone_candidate(frame, config):
    """Yellow upright tapered object; excludes flat stripe fragments."""
    import cv2
    import numpy as np
    image = cv2.resize(frame, (320,240))
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(config['lower_hsv'],np.uint8), np.array(config['upper_hsv'],np.uint8))
    contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    candidates = []
    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)/76800.
        if not (config['min_area'] <= area <= config['max_area'] and
                h >= config['min_height']*240 and .15 <= w/float(h) <= 1.2):
            continue
        body = np.zeros_like(mask); cv2.drawContours(body,[contour],-1,255,-1)
        rows=[]
        for fraction in (.25,.7):
            yy=min(239,y+int(h*fraction)); xs=np.flatnonzero(body[yy,x:x+w])
            rows.append((len(xs),float(xs.mean()) if len(xs) else 0))
        upper,lower=rows
        if not lower[0] or upper[0] > config['taper_ratio']*lower[0] or abs(upper[1]-lower[1]) > .22*w:
            continue
        center=(x+w/2.)/320.
        if abs(center-.5) > config['center_tolerance']:
            continue
        candidates.append(dict(bbox=[x/320.,y/240.,w/320.,h/240.],area=area,bottom=(y+h)/240.))
    return max(candidates,key=lambda item:item['area']) if candidates else None


def observe(frame, config, held_roi=None):
    import cv2
    import numpy as np
    image = cv2.resize(frame, (320, 240))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    saturation = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:,:,1]
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), *config['canny'])
    roi = config['roi']
    selected = np.zeros_like(edges)
    def rect(r):
        return (slice(int(r[1]*240), int(r[3]*240)), slice(int(r[0]*320), int(r[2]*320)))
    selected[rect(roi)] = 255
    if held_roi:
        selected[rect(held_roi)] = 0
    visible = float(cv2.countNonZero(selected)) / max(1, int((roi[2]-roi[0])*320*(roi[3]-roi[1])*240))
    edges &= selected
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, config['hough_votes'],
                            minLineLength=config['min_line_pixels'], maxLineGap=12)
    segments = [] if lines is None else lines.reshape(-1,4)
    posts = [(float(x1 if y1>y0 else x0),float(max(y0,y1)),float(x1-x0),float(y1-y0)) for x0,y0,x1,y1 in segments
             if abs(y1-y0) >= config.get('post_min_height',30) and abs(x1-x0) < .15*abs(y1-y0)]
    def panel_supported(x0,y0,x1,y1):
        if not config.get('require_panel_support',False): return True
        dx,dy=float(x1-x0),float(y1-y0); norm=math.hypot(dx,dy)
        if not norm: return False
        radius=config.get('post_base_radius',14)
        attached=any(abs(dx*(py-y0)-dy*(px-x0))/norm<=radius and
                     min(x0,x1)-radius <= px <= max(x0,x1)+radius and
                     min(y0,y1)-radius <= py <= max(y0,y1)+radius and
                     abs(dx*pdy-dy*pdx)/(norm*math.hypot(pdx,pdy))>.35 for px,py,pdx,pdy in posts)
        if not attached: return False
        # A board has a comparatively smooth surface ABOVE its bottom edge.
        samples=[]
        for t in (.2,.4,.6,.8):
            x=int(x0+t*dx); y=int(y0+t*dy)
            patch=gray[max(0,y-25):max(0,y-5),max(0,x-4):min(320,x+5)]
            sat = saturation[max(0,y-25):max(0,y-5),max(0,x-4):min(320,x+5)]
            if patch.size:
                samples.append(float(patch.std()) <= config.get('panel_max_std',18) and
                               float(np.median(sat)) <= config.get('panel_max_saturation',65))
        return bool(samples) and sum(samples)/float(len(samples))>=.75
    near_y, far_y = config['near_y']*240, config['far_y']*240
    left, right, front = [], [], False
    front_bottom = 0.
    panel_posts=[]
    def attach_posts(x0,y0,x1,y1):
        dx,dy=float(x1-x0),float(y1-y0); norm=math.hypot(dx,dy)
        radius=config.get('post_base_radius',14)
        for px,py,pdx,pdy in posts:
            if abs(dx*pdy-dy*pdx)/(max(1,norm)*math.hypot(pdx,pdy))<=.35: continue
            if norm and abs(dx*(py-y0)-dy*(px-x0))/norm<=radius and min(x0,x1)-radius<=px<=max(x0,x1)+radius and min(y0,y1)-radius<=py<=max(y0,y1)+radius:
                panel_posts.append((px/320.,py/240.))
    for x0, y0, x1, y1 in segments:
        dx, dy = float(x1-x0), float(y1-y0)
        length = math.hypot(dx, dy)
        if not panel_supported(x0,y0,x1,y1): continue
        if abs(dy) < .2 * max(1, abs(dx)):
            if (config['front_y'][0]*240 <= (y0+y1)/2. <= config['front_y'][1]*240
                    and abs(dx) >= config['front_span']*320 and min(x0,x1)<160<max(x0,x1)):
                front = True
                front_bottom=max(front_bottom,(y0+y1)/480.)
                attach_posts(x0,y0,x1,y1)
            continue
        xn = x0 + (near_y-y0)*dx/dy
        xf = x0 + (far_y-y0)*dx/dy
        if not (0 <= xn <= 320 and -80 <= xf <= 400):
            continue
        # A calibrated ground-contact boundary converges towards the corridor.
        if xn < 160 and xf > xn:
            left.append((length, xn, xf, min(y0,y1)/240.))
            attach_posts(x0,y0,x1,y1)
        elif xn > 160 and xf < xn:
            right.append((length, xn, xf, min(y0,y1)/240.))
            attach_posts(x0,y0,x1,y1)
    l = max(left) if left else None
    r = max(right) if right else None
    result = dict(valid=False, front=front, left_open=l is None and r is not None,
                  right_open=r is None and l is not None, visible=visible, boundaries=[],
                  posts=panel_posts,front_bottom=front_bottom,
                  image_quality=(bool(np.any(selected)) and float(gray[selected > 0].std()) >= config.get('min_image_std', 3)))
    # At a corner, the near side board can remain visible: its far endpoint
    # stops BEFORE the front board, leaving an opening beyond it.
    if front:
        gap=config.get('branch_gap_y',.06)
        if l and l[3]-front_bottom>=gap: result['left_open']=True
        if r and r[3]-front_bottom>=gap: result['right_open']=True
    if l: result['boundaries'].append((l[1]/320., config['near_y'], l[2]/320., config['far_y']))
    if r: result['boundaries'].append((r[1]/320., config['near_y'], r[2]/320., config['far_y']))
    cone = cone_candidate(frame,config['cone']) if config.get('cone') else None
    result['cone'] = cone
    if l and r and visible >= config['min_visible_fraction']:
        width = (r[1]-l[1])/320.
        if config['width_range'][0] <= width <= config['width_range'][1] and l[2] < r[2]:
            center = (l[1]+r[1])/640.
            far_center = (l[2]+r[2])/640.
            result.update(valid=True, error=center-config['center_x'], heading=far_center-center)
    return result


class Planner:
    """Known L/R route; fresh corridor observations gate each single action."""
    def __init__(self, config):
        self.config = config
        self.stage, self.turning, self.turn_cycles = 0, False, 0
        self.stable, self.corner_stable, self.advance = 0, 0, 0
        self.endpoint_stable = 0

    def decide(self, observation, endpoint=False):
        c = self.config
        cone=observation.get('cone')
        cone_ready=(c['endpoint_mode']=='yellow_cone' and self.stage==len(c['turns']) and
                    not self.turning and self.advance>=c['endpoint_min_forward_cycles'] and
                    cone is not None and cone['bottom']>=c['cone']['stop_bottom'] and
                    observation.get('visible',0)>=c['min_visible_fraction'])
        if c['endpoint_mode']=='yellow_cone':
            self.endpoint_stable=self.endpoint_stable+1 if cone_ready else 0
            if self.endpoint_stable>=c['endpoint_stable_frames']: return 'done'
            if cone_ready: return 'wait'  # stop advancing during endpoint confirmation
        exit_view = (self.stage == len(c['turns']) and not self.turning and
                     self.advance >= c['endpoint_min_forward_cycles'] and
                     not observation.get('boundaries') and not observation.get('front') and
                     observation.get('visible', 0) >= c['min_visible_fraction'] and
                     observation.get('image_quality', False))
        if c['endpoint_mode']!='yellow_cone':
            self.endpoint_stable = self.endpoint_stable+1 if exit_view else 0
        if c['endpoint_mode'] == 'boundaries_disappear' and self.endpoint_stable >= c['endpoint_stable_frames']:
            return 'done'
        if c['endpoint_mode']=='placement_file' and self.stage == len(c['turns']) and endpoint and not self.turning:
            return 'done'
        if self.turning:
            aligned = (observation.get('valid') and not observation['front'] and
                       abs(observation['error']) <= c['center_tolerance'] and
                       abs(observation['heading']) <= c['heading_tolerance'])
            self.stable = self.stable+1 if aligned else 0
            side = c['turns'][self.stage]
            if self.turn_cycles >= c['turn_min_cycles'][side] and self.stable >= c['stable_frames']:
                self.stage += 1
                self.turning = False
                self.advance = self.stable = self.corner_stable = 0
                return 'wait'
            if aligned and self.turn_cycles >= c['turn_min_cycles'][side]:
                return 'wait'  # confirm distinct frames without further rotation
            if self.turn_cycles >= c['turn_max_cycles'][side]:
                raise RuntimeError('New corridor not aligned within turn limit')
            self.turn_cycles += 1
            return side
        side = c['turns'][self.stage] if self.stage < len(c['turns']) else None
        corner = (side is not None and observation.get('front') and
                  observation.get(side+'_open') and observation.get('visible', 0) >= c['min_visible_fraction'])
        self.corner_stable = self.corner_stable+1 if corner else 0
        if corner and self.advance >= c['min_forward_cycles']:
            if self.corner_stable >= c['stable_frames']:
                self.turning, self.turn_cycles, self.stable = True, 1, 0
                return side
            return 'wait'
        if not observation.get('valid') or observation.get('front'):
            return 'wait'
        error = observation['error'] + c['heading_gain']*observation['heading']
        if abs(error) > c['center_tolerance']:
            return 'right' if error > 0 else 'left'
        self.advance += 1
        return 'forward'


def validate(config):
    if config.get('calibrated') is not True:
        raise ValueError('Calibrate corridor ROI, boundaries, corner cues and loaded turn limits first')
    if config['turns'] != ['left', 'right']:
        raise ValueError('Expected left then right route')
    if config['endpoint_mode'] not in ('placement_file', 'boundaries_disappear','yellow_cone'):
        raise ValueError('Unsupported endpoint mode')
    roi = config['roi']
    if (len(roi) != 4 or not all(math.isfinite(v) and 0 <= v <= 1 for v in roi)
            or not roi[0] < roi[2] or not roi[1] < roi[3]):
        raise ValueError('Invalid lane ROI')
    if not roi[1] <= config['far_y'] < config['near_y'] <= roi[3]:
        raise ValueError('Lane sample rows must be inside ROI, far before near')
    for key in ['timeout', 'lost_timeout', 'settle_seconds', 'center_tolerance', 'heading_tolerance']:
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError('Invalid lane ' + key)
    for key in ['stable_frames','endpoint_stable_frames','min_forward_cycles','endpoint_min_forward_cycles']:
        if isinstance(config[key], bool) or not isinstance(config[key],int) or config[key] < 1:
            raise ValueError('Invalid lane ' + key)
    for side in config['turns']:
        low, high = config['turn_min_cycles'][side], config['turn_max_cycles'][side]
        if isinstance(low, bool) or not isinstance(low, int) or not 1 <= low <= high <= 100:
            raise ValueError('Measure bounded loaded turn cycle counts')


def run(config, motion, feedback, heartbeat):
    from task_feedback import read_sample, FeedbackUnavailable
    planner = Planner(config)
    camera = feedback.camera
    start = time.time()
    deadline = time.monotonic() + config['timeout']
    last = time.monotonic()
    lost_since = None
    endpoint_since = None
    while time.monotonic() < deadline:
        motion.check_safety()
        frame, stamp = camera.read_after(last)
        if frame is None:
            if time.monotonic()-last > feedback.config['camera_timeout']:
                raise RuntimeError('Camera stale during corridor route')
            motion.wait(.02)
            continue
        last = stamp
        obs = observe(frame, config, feedback.config.get('held_roi'))
        endpoint = False
        if planner.stage == len(config['turns']) and config['endpoint_mode'] == 'placement_file':
            spec = feedback.config['placement_ready']
            try:
                endpoint = read_sample(spec['file'], spec['max_age'], after=start).get('state') == 'placement_ready'
            except FeedbackUnavailable:
                pass
        decision = planner.decide(obs, endpoint)
        if decision == 'done' and config['endpoint_mode'] in ('boundaries_disappear','yellow_cone'):
            endpoint_since = time.monotonic() if endpoint_since is None else endpoint_since
            if time.monotonic()-endpoint_since < config.get('endpoint_hold_seconds', 1.0):
                decision = 'wait'
        else:
            endpoint_since = None
        view = getattr(motion, 'debug_view', None)
        if view:
            view.update('Lane stage=%d turn=%s decision=%s error=%s' %
                        (planner.stage, planner.turning, decision, obs.get('error')))
        if decision == 'done':
            return True
        if decision == 'wait':
            lost_since = time.monotonic() if lost_since is None else lost_since
            if time.monotonic()-lost_since > config['lost_timeout']:
                raise RuntimeError('Corridor/corner uncertain; no blind forward motion')
            motion.wait(.05)
            continue
        lost_since = None
        heartbeat.set('carrying')
        motion.action(decision)
        motion.wait(config['settle_seconds'])
        if feedback.config.get('verify_carry', True) and not feedback.confirm('held'):
            raise RuntimeError('Holding lost during lane following')
        # Neither the planner nor feedback may reuse a pre-motion frame.
        last = time.monotonic()
    raise RuntimeError('Corridor route timeout')
