#!/usr/bin/env python3
# coding: utf-8
"""
wonderpi_soccer_kick_competition.py

比赛版：识别足球 -> 稳定跟踪 -> 规划接近/横移 -> 强制左侧对齐 -> 强制左脚射门。
新增：阻塞式开环微调逻辑。达到阈值后彻底屏蔽视觉干扰，直接完成微调+射门连招。
默认适配幻尔/Hiwonder TonyPi 系列的 Python SDK 与 ActionGroup：
  /home/pi/TonyPi/Functions
  python3 wonderpi_soccer_kick_competition.py --color green --debug 1
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time
import threading
import queue
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Optional, Tuple, Any

import cv2
import numpy as np

SDK_PATHS = (
    "/home/pi/TonyPi/HiwonderSDK/build/lib",
    "/home/pi/TonyPi/HiwonderSDK/build/lib/hiwonder",
    "/home/pi/TonyPi/HiwonderSDK",
    "/home/pi/TonyPi/Functions",
    "/home/pi/WonderPi/Functions",
    "/home/pi/TonyPi",
)
for sdk_path in SDK_PATHS:
    if os.path.exists(sdk_path) and sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)

HIWONDER_IMPORT_ERROR = None
Camera = None
Misc = None
PID = None
rrc = None
AGC = None
yaml_handle = None
Controller = None
Board = None

try:
    import hiwonder.Camera as Camera
except Exception as e:
    HIWONDER_IMPORT_ERROR = e

try:
    import hiwonder.Misc as Misc
except Exception as e:
    HIWONDER_IMPORT_ERROR = e

try:
    import hiwonder.PID as PID
except Exception as e:
    HIWONDER_IMPORT_ERROR = e

try:
    import hiwonder.ros_robot_controller_sdk as rrc
    from hiwonder.Controller import Controller
except Exception as e:
    HIWONDER_IMPORT_ERROR = e

try:
    import hiwonder.Board as Board
except Exception:
    try:
        from hiwonder import Board
    except Exception:
        try:
            import Board
        except Exception as e:
            HIWONDER_IMPORT_ERROR = e

try:
    import hiwonder.ActionGroupControl as AGC
except Exception:
    try:
        from hiwonder import ActionGroupControl as AGC
    except Exception:
        try:
            import ActionGroupControl as AGC
        except Exception as e:
            HIWONDER_IMPORT_ERROR = e

try:
    import hiwonder.yaml_handle as yaml_handle
except Exception as e:
    HIWONDER_IMPORT_ERROR = e

HIWONDER_OK = AGC is not None and ((rrc is not None and Controller is not None) or Board is not None)


# -----------------------------
# 参数
# -----------------------------

@dataclass
class VisionConfig:
    color_name: str = "green"
    process_size: Tuple[int, int] = (320, 240)
    min_area: int = 60
    max_area_ratio: float = 0.35
    min_circularity: float = 0.45
    morph_kernel: int = 3
    blur_kernel: int = 3
    ema_alpha: float = 0.55
    lost_keep_frames: int = 6
    lower_lab: Optional[Tuple[int, int, int]] = None
    upper_lab: Optional[Tuple[int, int, int]] = None


@dataclass
class MotionConfig:
    pan_servo_id: int = 2
    tilt_servo_id: int = 1
    pan_limit: int = 400
    tilt_min: int = 0          # 会在读取 servo_config 后设为 servo1
    tilt_max: int = 1200
    pan_pid_p: float = 0.065
    pan_pid_i: float = 0.0
    pan_pid_d: float = 0.0
    tilt_pid_p: float = 0.055
    tilt_pid_i: float = 0.0
    tilt_pid_d: float = 0.0
    servo_scale_ms: float = 6.0
    head_x_deadband_px: int = 35
    head_y_deadband_px: int = 25
    head_max_step: int = 10
    head_min_interval: float = 0.08


@dataclass
class PlannerConfig:
    align_pan_deadband: int = 150
    align_x_deadband_px: int = 50
    kick_x_tolerance_px: int = 45
    # kick_y_ratio: 球心落到画面此比例时起脚，默认 0.90
    kick_y_ratio: float = 0.90
    slow_y_ratio: float = 0.58
    fast_y_ratio: float = 0.35
    kick_offset_px: int = 45       
    action_cooldown: float = 0.16
    lost_search_after_s: float = 0.45
    max_no_ball_before_turn_s: float = 2.2
    debug_draw: bool = False


@dataclass
class BallObservation:
    x: int
    y: int
    radius: int
    area: float
    circularity: float
    confidence: float
    timestamp: float


class RobotState(Enum):
    INIT = auto()
    SEARCH = auto()
    ALIGN_BODY = auto()
    APPROACH_FAST = auto()
    APPROACH_SLOW = auto()
    LATERAL_ALIGN = auto()
    KICK = auto()
    RECOVER = auto()


# -----------------------------
# 工具函数
# -----------------------------

def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))

def parse_lab_triplet(s: Optional[str]) -> Optional[Tuple[int, int, int]]:
    if not s: return None
    parts = [int(x.strip()) for x in s.split(",")]
    if len(parts) != 3: raise ValueError("LAB 参数格式应为 L,A,B，例如 20,140,130")
    return tuple(clamp(x, 0, 255) for x in parts)

def load_yaml_dict(path: str) -> Dict[str, Any]:
    if not os.path.exists(path): return {}
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return data if isinstance(data, dict) else {}
    except Exception: return {}

def load_servo_config() -> Dict[str, int]:
    default = {"servo1": 1500, "servo2": 1500}
    if yaml_handle is not None:
        try:
            data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)
            if isinstance(data, dict):
                default.update({k: int(v) for k, v in data.items() if k in ("servo1", "servo2")})
        except Exception: pass
    for path in (
        "/home/pi/TonyPi/servo_config.yaml",
        "/home/pi/TonyPi/Functions/../servo_config.yaml",
        "servo_config.yaml",
        "../servo_config.yaml",
    ):
        data = load_yaml_dict(path)
        if data:
            default.update({k: int(v) for k, v in data.items() if k in ("servo1", "servo2")})
            break
    return default

def load_lab_config(color_name: str) -> Optional[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]:
    candidates = []
    if yaml_handle is not None:
        for attr in ("lab_file_path", "lab_file_path_yaml"):
            p = getattr(yaml_handle, attr, None)
            if p: candidates.append(p)
    candidates += [
        "/home/pi/TonyPi/lab_config.yaml",
        "/home/pi/TonyPi/Functions/../lab_config.yaml",
        "lab_config.yaml",
        "../lab_config.yaml",
    ]
    for path in candidates:
        data = load_yaml_dict(path)
        if not data: continue
        node = data.get(color_name)
        if isinstance(node, dict) and "min" in node and "max" in node:
            lo = tuple(int(x) for x in node["min"][:3])
            hi = tuple(int(x) for x in node["max"][:3])
            return lo, hi
    fallback = {
        "red": ((0, 145, 120), (255, 210, 190)),
        "orange": ((0, 130, 140), (255, 190, 210)),
        "blue": ((0, 110, 80), (255, 150, 135)),
        "green": ((20, 80, 105), (255, 150, 170)),
    }
    return fallback.get(color_name)

def get_area_max_contour(contours, min_area: int, max_area: float):
    best = None
    best_area = 0.0
    for c in contours:
        area = abs(cv2.contourArea(c))
        if min_area <= area <= max_area and area > best_area:
            best_area = area
            best = c
    return best, best_area


# -----------------------------
# 视觉识别
# -----------------------------

class BallDetector:
    def __init__(self, cfg: VisionConfig):
        self.cfg = cfg
        if cfg.lower_lab and cfg.upper_lab:
            self.lower_lab, self.upper_lab = cfg.lower_lab, cfg.upper_lab
        else:
            loaded = load_lab_config(cfg.color_name)
            if loaded is None: raise RuntimeError(f"未找到颜色 {cfg.color_name} 阈值")
            self.lower_lab, self.upper_lab = loaded

        k = cfg.morph_kernel if cfg.morph_kernel % 2 == 1 else cfg.morph_kernel + 1
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        self.last_obs: Optional[BallObservation] = None
        self.lost_frames = 0

    def detect(self, frame_bgr: np.ndarray) -> Tuple[Optional[BallObservation], np.ndarray]:
        now = time.time()
        h, w = frame_bgr.shape[:2]
        proc_w, proc_h = self.cfg.process_size
        small = cv2.resize(frame_bgr, (proc_w, proc_h), interpolation=cv2.INTER_NEAREST)

        blur_k = self.cfg.blur_kernel if self.cfg.blur_kernel % 2 == 1 else self.cfg.blur_kernel + 1
        small_blur = cv2.GaussianBlur(small, (blur_k, blur_k), 0)
        lab = cv2.cvtColor(small_blur, cv2.COLOR_BGR2LAB)

        mask = cv2.inRange(lab, np.array(self.lower_lab, np.uint8), np.array(self.upper_lab, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        max_area = proc_w * proc_h * self.cfg.max_area_ratio
        contour, area = get_area_max_contour(contours, self.cfg.min_area, max_area)

        obs = None
        if contour is not None:
            perimeter = cv2.arcLength(contour, True)
            circularity = 0.0 if perimeter <= 1e-6 else 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity >= self.cfg.min_circularity:
                (cx, cy), r = cv2.minEnclosingCircle(contour)
                x, y, radius = int(cx / proc_w * w), int(cy / proc_h * h), int(r / proc_w * w)
                confidence = float(min(1.0, 0.55 * circularity + 0.45 * min(1.0, area / (proc_w * proc_h * 0.08))))
                raw = BallObservation(x, y, radius, area, circularity, confidence, now)
                obs = self._smooth(raw)

        if obs is None:
            self.lost_frames += 1
            if self.last_obs is not None and self.lost_frames <= self.cfg.lost_keep_frames:
                obs = BallObservation(
                    self.last_obs.x, self.last_obs.y, self.last_obs.radius,
                    self.last_obs.area, self.last_obs.circularity,
                    max(0.0, self.last_obs.confidence - 0.15 * self.lost_frames), now,
                )
            else: self.last_obs = None
        else: self.lost_frames = 0
        return obs, mask

    def _smooth(self, raw: BallObservation) -> BallObservation:
        if self.last_obs is None:
            self.last_obs = raw
            return raw
        a = self.cfg.ema_alpha
        smoothed = BallObservation(
            x=int(a * raw.x + (1 - a) * self.last_obs.x),
            y=int(a * raw.y + (1 - a) * self.last_obs.y),
            radius=int(a * raw.radius + (1 - a) * self.last_obs.radius),
            area=a * raw.area + (1 - a) * self.last_obs.area,
            circularity=a * raw.circularity + (1 - a) * self.last_obs.circularity,
            confidence=a * raw.confidence + (1 - a) * self.last_obs.confidence,
            timestamp=raw.timestamp,
        )
        self.last_obs = smoothed
        return smoothed


# -----------------------------
# 动作控制
# -----------------------------

class MotionController:
    def __init__(self, mcfg: MotionConfig, dry_run: bool = False):
        self.cfg = mcfg
        self.dry_run = dry_run or not HIWONDER_OK
        self.servo = load_servo_config()
        self.pan_center = int(self.servo.get("servo2", 1500))
        self.tilt_center = int(self.servo.get("servo1", 1500))
        self.cfg.tilt_min = self.tilt_center
        self.pan = self.pan_center
        self.tilt = self.tilt_center

        self._q: "queue.Queue[Tuple[str, int, bool]]" = queue.Queue(maxsize=3)
        self._busy = threading.Event()
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._action_worker, daemon=True)

        if self.dry_run:
            self.board = None
            self.ctl = None
            if not HIWONDER_OK: print(f"[WARN] 未导入SDK，进入 dry-run：{HIWONDER_IMPORT_ERROR}", file=sys.stderr)
        else:
            self.ctl = None
            if rrc is not None and Controller is not None:
                self.board = rrc.Board()
                self.ctl = Controller(self.board)
            else: self.board = Board
        self._worker.start()

    def init_pose(self):
        self.set_head(self.pan_center, self.tilt_center, 500)
        self.run_action("stand_slow", block=False)

    def shutdown(self):
        self._stop.set()
        try:
            while not self._q.empty(): self._q.get_nowait()
        except Exception: pass
        self.set_head(self.pan_center, self.tilt_center, 300)
        self.run_action("stand_slow", block=True)

    def busy(self) -> bool: return self._busy.is_set()

    def set_head(self, pan: int, tilt: int, duration_ms: int):
        self.pan = clamp(int(pan), self.pan_center - self.cfg.pan_limit, self.pan_center + self.cfg.pan_limit)
        self.tilt = clamp(int(tilt), self.cfg.tilt_min, self.cfg.tilt_max)
        if self.dry_run: return
        try:
            if self.ctl is not None:
                self.ctl.set_pwm_servo_pulse(self.cfg.tilt_servo_id, self.tilt, duration_ms)
                self.ctl.set_pwm_servo_pulse(self.cfg.pan_servo_id, self.pan, duration_ms)
            elif Board is not None:
                Board.setPWMServoPulse(self.cfg.tilt_servo_id, self.tilt, duration_ms)
                Board.setPWMServoPulse(self.cfg.pan_servo_id, self.pan, duration_ms)
        except Exception as e: print(f"[WARN] set_head failed: {e}", file=sys.stderr)

    def run_action(self, name: str, times: int = 1, with_stand: bool = False, block: bool = False):
        if block:
            self._execute_action(name, times, with_stand)
            return
        if self._busy.is_set(): return
        try: self._q.put_nowait((name, times, with_stand))
        except queue.Full: pass

    def _action_worker(self):
        while not self._stop.is_set():
            try: name, times, with_stand = self._q.get(timeout=0.05)
            except queue.Empty: continue
            self._busy.set()
            self._execute_action(name, times, with_stand)
            self._busy.clear()

    def _execute_action(self, name: str, times: int = 1, with_stand: bool = False):
        if self.dry_run:
            print(f"[ACTION] {name} x{times}")
            time.sleep(0.18)
            return
        try:
            if with_stand: AGC.runActionGroup(name, times, with_stand=True)
            else: AGC.runActionGroup(name, times)
        except TypeError:
            try: AGC.runActionGroup(name, times)
            except Exception as e: print(f"[WARN] action {name} failed: {e}", file=sys.stderr)
        except Exception as e: print(f"[WARN] action {name} failed: {e}", file=sys.stderr)


# -----------------------------
# 规划状态机
# -----------------------------

class SoccerKickAgent:
    def __init__(self, detector: BallDetector, motion: MotionController, pcfg: PlannerConfig, mcfg: MotionConfig):
        self.detector = detector
        self.motion = motion
        self.cfg = pcfg
        self.mcfg = mcfg

        self.state = RobotState.INIT
        self.last_ball_time = 0.0
        self.last_action_time = 0.0
        self.no_ball_since: Optional[float] = None
        self.search_dir = 1
        self.debug_ball_mask: Optional[np.ndarray] = None
        self.last_head_update = 0.0
        self.kick_ready_count = 0

        self.x_pid = PID.PID(P=mcfg.pan_pid_p, I=mcfg.pan_pid_i, D=mcfg.pan_pid_d) if PID else None
        self.y_pid = PID.PID(P=mcfg.tilt_pid_p, I=mcfg.tilt_pid_i, D=mcfg.tilt_pid_d) if PID else None

    def update(self, frame: np.ndarray) -> Tuple[np.ndarray, RobotState, Optional[BallObservation]]:
        obs, mask = self.detector.detect(frame)
        self.debug_ball_mask = mask
        h, w = frame.shape[:2]
        now = time.time()

        if obs is not None and obs.confidence > 0.15:
            self.last_ball_time = now
            self.no_ball_since = None
            self._track_head(obs, w, h)
            self._plan_with_ball(obs, w, h)
        else:
            if self.no_ball_since is None: self.no_ball_since = now
            self._plan_without_ball(now)

        vis = self._draw_debug(frame.copy(), mask, obs) if self.cfg.debug_draw else frame
        return vis, self.state, obs

    def _track_head(self, obs: BallObservation, w: int, h: int):
        now = time.time()
        if now - self.last_head_update < self.mcfg.head_min_interval: return
        x_error = w / 2 - obs.x
        y_error = h / 2 - obs.y
        if abs(x_error) < self.mcfg.head_x_deadband_px and abs(y_error) < self.mcfg.head_y_deadband_px: return

        dx = int(self.x_pid.output) if (self.x_pid and abs(x_error) >= self.mcfg.head_x_deadband_px and not self.x_pid.update(obs.x)) else int(0.04 * x_error) if abs(x_error) >= self.mcfg.head_x_deadband_px else 0
        dy = int(self.y_pid.output) if (self.y_pid and abs(y_error) >= self.mcfg.head_y_deadband_px and not self.y_pid.update(obs.y)) else int(0.04 * y_error) if abs(y_error) >= self.mcfg.head_y_deadband_px else 0
        if self.x_pid and abs(x_error) >= self.mcfg.head_x_deadband_px:
            self.x_pid.SetPoint = w / 2
            self.x_pid.update(obs.x)
            dx = int(self.x_pid.output)
        if self.y_pid and abs(y_error) >= self.mcfg.head_y_deadband_px:
            self.y_pid.SetPoint = h / 2
            self.y_pid.update(obs.y)
            dy = int(self.y_pid.output)

        dx = clamp(dx, -self.mcfg.head_max_step, self.mcfg.head_max_step)
        dy = clamp(dy, -self.mcfg.head_max_step, self.mcfg.head_max_step)
        if dx == 0 and dy == 0: return

        duration_ms = int(max(80, min(220, max(abs(dx), abs(dy)) * self.mcfg.servo_scale_ms)))
        self.motion.set_head(self.motion.pan + dx, self.motion.tilt + dy, duration_ms)
        self.last_head_update = now

    def _can_act(self) -> bool: return (not self.motion.busy()) and (time.time() - self.last_action_time >= self.cfg.action_cooldown)

    def _act(self, name: str, times: int = 1, with_stand: bool = False):
        if not self._can_act(): return False
        self.motion.run_action(name, times=times, with_stand=with_stand, block=False)
        self.last_action_time = time.time()
        return True

    def _plan_with_ball(self, obs: BallObservation, w: int, h: int):
        center_x = w // 2
        pan_error = self.motion.pan - self.motion.pan_center
        x_error = obs.x - center_x
        y_ratio = obs.y / float(h)

        if abs(pan_error) > self.cfg.align_pan_deadband:
            self.kick_ready_count = 0
            self.state = RobotState.ALIGN_BODY
            self._act("turn_left_small_step" if pan_error > 0 else "turn_right_small_step", with_stand=True)
            return

        if y_ratio < self.cfg.fast_y_ratio:
            self.kick_ready_count = 0
            self.state = RobotState.APPROACH_FAST
            self._act("go_forward_fast", with_stand=True)
            return

        if y_ratio < self.cfg.slow_y_ratio:
            self.kick_ready_count = 0
            self.state = RobotState.APPROACH_SLOW
            if abs(x_error) > self.cfg.align_x_deadband_px:
                self.state = RobotState.ALIGN_BODY
                self._act("turn_left_small_step" if x_error < 0 else "turn_right_small_step", with_stand=True)
            else: self._act("go_forward_fast", with_stand=True)
            return

        # 3) 近距离：强制倾向使用左脚，将球永远对准左脚击球线
        foot = "left"  
        target_x = center_x - self.cfg.kick_offset_px
        lateral_error = obs.x - target_x

        if y_ratio < self.cfg.kick_y_ratio:
            self.kick_ready_count = 0
            self.state = RobotState.APPROACH_SLOW
            if abs(lateral_error) > self.cfg.kick_x_tolerance_px * 1.5:
                self.state = RobotState.LATERAL_ALIGN
                self._lateral_move(lateral_error)
            else: self._act("go_forward_one_small_step", with_stand=True)
            return

        if abs(lateral_error) > self.cfg.kick_x_tolerance_px:
            self.kick_ready_count = 0
            self.state = RobotState.LATERAL_ALIGN
            self._lateral_move(lateral_error)
            return

        self.kick_ready_count += 1
        if self.kick_ready_count < 2:
            self.state = RobotState.APPROACH_SLOW
            return

        # ==============================================================
        # 4) 起脚射门阶段 (完全阻塞视觉，严格按照设定连招一次性执行完毕)
        # ==============================================================
        self.state = RobotState.KICK

        # 预留的开环强制微调接口。将 steps 改为 1 或更大，即可起效。
        # 这里使用 block=True，保证底层硬件走完动作前，视觉主循环处于冻结状态，彻底断绝视觉反复横跳问题。
        self.force_walk_forward(steps=2, speed_mode="fast")
        self.force_lateral_move(direction="right", steps=5) 
        self.force_turn(direction="left", steps=2) 
        #self.force_walk_forward(steps=1, speed_mode="fast")

        # 强制左脚起脚动作，并同步阻塞执行
        kick_action = "left_shot" 
        self.motion.run_action(kick_action, times=1, with_stand=True, block=True)
        
        # 动作连招执行完毕后，重置动作时间和状态机计次
        self.last_action_time = time.time()
        self.kick_ready_count = 0
        self.state = RobotState.RECOVER

    def _lateral_move(self, lateral_error: int):
        self._act("right_move" if lateral_error > 0 else "left_move", with_stand=True)

    def _plan_without_ball(self, now: float):
        self.kick_ready_count = 0
        self.state = RobotState.SEARCH
        if now - self.no_ball_since < self.cfg.lost_search_after_s: return

        step = 22 * self.search_dir
        next_pan = self.motion.pan + step
        if next_pan > self.motion.pan_center + self.mcfg.pan_limit:
            next_pan = self.motion.pan_center + self.mcfg.pan_limit
            self.search_dir = -1
        elif next_pan < self.motion.pan_center - self.mcfg.pan_limit:
            next_pan = self.motion.pan_center - self.mcfg.pan_limit
            self.search_dir = 1
        self.motion.set_head(next_pan, self.motion.tilt_center, 40)

        if now - self.no_ball_since > self.cfg.max_no_ball_before_turn_s and self._can_act():
            self._act("turn_right")
            self.no_ball_since = now

    def _draw_debug(self, frame: np.ndarray, mask: np.ndarray, obs: Optional[BallObservation]) -> np.ndarray:
        h, w = frame.shape[:2]
        cx = w // 2
        cv2.line(frame, (cx, 0), (cx, h), (0, 255, 255), 1)
        cv2.line(frame, (0, int(h * self.cfg.fast_y_ratio)), (w, int(h * self.cfg.fast_y_ratio)), (255, 255, 0), 1)
        cv2.line(frame, (0, int(h * self.cfg.slow_y_ratio)), (w, int(h * self.cfg.slow_y_ratio)), (255, 255, 0), 1)
        cv2.line(frame, (0, int(h * self.cfg.kick_y_ratio)), (w, int(h * self.cfg.kick_y_ratio)), (0, 255, 0), 2)
        left_target = cx - self.cfg.kick_offset_px
        right_target = cx + self.cfg.kick_offset_px
        cv2.line(frame, (left_target, 0), (left_target, h), (255, 0, 255), 1)
        cv2.line(frame, (right_target, 0), (right_target, h), (255, 0, 255), 1)

        if obs is not None:
            cv2.circle(frame, (obs.x, obs.y), obs.radius, (0, 255, 255), 2)
            cv2.circle(frame, (obs.x, obs.y), 3, (0, 0, 255), -1)
            cv2.putText(frame, f"ball x={obs.x} y={obs.y} r={obs.radius} conf={obs.confidence:.2f}",
                        (10, h - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        cv2.putText(frame, f"state={self.state.name} pan={self.motion.pan} tilt={self.motion.tilt}",
                    (10, h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(frame, f"LAB {self.detector.lower_lab}->{self.detector.upper_lab}",
                    (10, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        mh, mw = mask.shape[:2]
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        thumb = cv2.resize(mask_bgr, (w // 4, h // 4))
        frame[0:h // 4, w - w // 4:w] = thumb
        return frame

    # =========================================================
    # 阻塞式手动/开环微调接口 (加入 steps <= 0 保护)
    # 调用时使用 block=True，彻底屏蔽视觉抢占
    # =========================================================

    def force_walk_forward(self, steps: int = 1, speed_mode: str = "fast"):
        if steps <= 0: return
        action_map = {
            "normal": "go_forward",
            "fast": "go_forward_fast",
            "small": "go_forward_one_small_step"
        }
        action_name = action_map.get(speed_mode, "go_forward_one_small_step")
        print(f"[MANUAL OVERRIDE] 直走微调: {action_name} x {steps}")
        self.motion.run_action(action_name, times=steps, with_stand=True, block=True)

    def force_lateral_move(self, direction: str, steps: int = 1):
        if steps <= 0: return
        action_name = "left_move" if direction == "left" else "right_move"
        print(f"[MANUAL OVERRIDE] 平移微调: {action_name} x {steps}")
        self.motion.run_action(action_name, times=steps, with_stand=True, block=True)

    def force_turn(self, direction: str, steps: int = 1):
        if steps <= 0: return
        action_name = "turn_left_small_step" if direction == "left" else "turn_right_small_step"
        print(f"[MANUAL OVERRIDE] 旋转微调: {action_name} x {steps}")
        self.motion.run_action(action_name, times=steps, with_stand=True, block=True)


# -----------------------------
# 摄像头
# -----------------------------

class CameraReader:
    def __init__(self, source: str = "auto"):
        self.source = source
        self.cam = None

    def open(self):
        if self.source == "auto":
            open_once = False
            try:
                if yaml_handle is not None:
                    open_once = bool(yaml_handle.get_yaml_data("/boot/camera_setting.yaml").get("open_once", False))
            except Exception: pass

            if open_once: self.cam = cv2.VideoCapture("http://127.0.0.1:8080/?action=stream?dummy=param.mjpg")
            elif Camera is not None:
                self.cam = Camera.Camera()
                self.cam.camera_open()
            else: self.cam = cv2.VideoCapture(0)
        elif self.source.isdigit(): self.cam = cv2.VideoCapture(int(self.source))
        else: self.cam = cv2.VideoCapture(self.source)

    def read(self):
        if self.cam is None: return False, None
        return self.cam.read()

    def close(self):
        if self.cam is None: return
        try:
            if hasattr(self.cam, "camera_close"): self.cam.camera_close()
            else: self.cam.release()
        except Exception: pass


# -----------------------------
# 主程序
# -----------------------------

def build_arg_parser():
    p = argparse.ArgumentParser(description="WonderPi/TonyPi soccer ball detection and kick competition agent")
    p.add_argument("--color", default="green", help="识别颜色名，对应 lab_config.yaml")
    p.add_argument("--lower-lab", default=None, help="手动 LAB 下限")
    p.add_argument("--upper-lab", default=None, help="手动 LAB 上限")
    p.add_argument("--camera", default="auto", help="auto / 0 / 视频流URL")
    p.add_argument("--debug", type=int, default=1, help="1 显示调试窗口，0 关闭")
    p.add_argument("--debug-masks", type=int, default=1, help="1 在 debug 模式下单独显示小球 mask")
    p.add_argument("--dry-run", type=int, default=0, help="1 只打印动作，不控制机器人")
    p.add_argument("--kick-y-ratio", type=float, default=0.90, help="球中心y超过该比例认为可踢")
    p.add_argument("--kick-offset-px", type=int, default=35, help="左右脚击球线相对画面中心偏移")
    p.add_argument("--min-circularity", type=float, default=0.45, help="圆形约束，越高越严格")
    p.add_argument("--min-area", type=int, default=60, help="处理图像上的最小轮廓面积")
    return p

def main():
    args = build_arg_parser().parse_args()
    vcfg = VisionConfig(
        color_name=args.color, min_area=args.min_area,
        min_circularity=args.min_circularity,
        lower_lab=parse_lab_triplet(args.lower_lab),
        upper_lab=parse_lab_triplet(args.upper_lab),
    )
    mcfg = MotionConfig()
    pcfg = PlannerConfig(
        debug_draw=bool(args.debug),
        kick_y_ratio=args.kick_y_ratio,
        kick_offset_px=args.kick_offset_px,
    )
    detector = BallDetector(vcfg)
    motion = MotionController(mcfg, dry_run=bool(args.dry_run))
    agent = SoccerKickAgent(detector, motion, pcfg, mcfg)
    camera = CameraReader(args.camera)

    stop_flag = {"stop": False}
    def _sigint_handler(sig, frame): stop_flag["stop"] = True
    signal.signal(signal.SIGINT, _sigint_handler)
    signal.signal(signal.SIGTERM, _sigint_handler)

    print("[INFO] soccer kick agent starting (Blocking Manual Overrides)")
    print(f"[INFO] LAB {detector.lower_lab} -> {detector.upper_lab}")
    motion.init_pose()

    # =========================================================
    # 预留开环/强制起步接口
    # 在比赛一开始，距离球较远时，可以直接盲走几步快速拉近距离，节省视觉对齐时间。
    # (默认0次，运行时不会生效。需微调时，将 init_forward_steps 改为 1 或更多)
    # =========================================================
    init_forward_steps = 5
    if init_forward_steps > 0:
        print(f"[MANUAL OVERRIDE] 比赛开始，强制起步快走: go_forward_fast x {init_forward_steps}")
        motion.run_action("go_forward_fast", times=init_forward_steps, with_stand=True, block=True)

    camera.open()


    try:
        while not stop_flag["stop"]:
            ret, frame = camera.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            vis, state, obs = agent.update(frame)

            if args.debug:
                cv2.imshow("soccer_kick_camera", vis)
                if args.debug_masks and agent.debug_ball_mask is not None:
                    cv2.imshow("soccer_kick_ball_mask", agent.debug_ball_mask)
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"): break
            else: time.sleep(0.01)

    finally:
        print("[INFO] shutting down")
        camera.close()
        motion.shutdown()
        if args.debug: cv2.destroyAllWindows()

if __name__ == "__main__":
    main()