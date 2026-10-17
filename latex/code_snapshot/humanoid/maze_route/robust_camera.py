#!/usr/bin/env python3
# encoding:utf-8
"""hiwonder.Camera 的稳健替代(drop-in replacement for hiwonder.Camera)

与原版接口相同：Camera().camera_open() / read() / frame / camera_close()。
改进：
- 按 /dev/v4l/by-id 打开 USB 摄像头并指定 V4L2，重插后编号变化也能找到，
  不再走 VideoCapture(-1) 和 obsensor 后端，不刷 OpenCV 报错。
- 设备不存在时只等待，不调用 OpenCV；被占用时打印占用进程，
  若是网页控制台(tonypi_web_control)则自动关闭其相机。
- 连续读帧失败或超时自动释放并重开，重试间隔 0.5s 递增到 3s。
- read() 只返回 0.5s 内的新帧，掉线时不再把旧画面当成新画面。
"""
import glob
import json
import logging
import os
import threading
import time
import urllib.request

import cv2

try:
    from hiwonder import yaml_handle
except ImportError:
    yaml_handle = None

BY_ID = '/dev/v4l/by-id/usb-*-video-index0'
CONSOLE = 'http://127.0.0.1:9040'
FRESH = 0.5        # read() 只认这么新的帧(seconds a frame stays valid)
STALL = 2.0        # 这么久没有新帧就重开(reopen after this long without frames)


def _log(msg, _last={}):
    # 同一消息 10 秒内只记录一次(log the same message at most every 10 s)
    now = time.monotonic()
    if now - _last.get(msg, -1e9) >= 10:
        _last[msg] = now
        if logging.getLogger().handlers:
            logging.getLogger('camera').warning(msg)   # goes into the caller's log file
        else:
            print('[camera]', msg, flush=True)


def find_device():
    found = sorted(glob.glob(BY_ID))
    if found:
        return os.path.realpath(found[0])
    return '/dev/video0' if os.path.exists('/dev/video0') else None


def holders(dev):
    out = []
    for proc in glob.glob('/proc/[0-9]*'):
        if int(proc[6:]) == os.getpid():
            continue
        try:
            for fd in os.listdir(proc + '/fd'):
                if os.readlink('%s/fd/%s' % (proc, fd)) == dev:
                    with open(proc + '/cmdline', 'rb') as f:
                        out.append((int(proc[6:]), f.read().replace(b'\0', b' ').decode(errors='replace').strip()))
                    break
        except OSError:
            continue
    return out


def release_console_camera():
    """网页控制台占用时，通过其本机接口关闭相机(ask the local console to drop the camera)."""
    try:
        with urllib.request.urlopen(CONSOLE + '/api/status', timeout=2) as r:
            token = json.load(r)['token']
        req = urllib.request.Request(
            CONSOLE + '/api/command', method='POST',
            data=json.dumps({'kind': 'camera', 'enabled': False, 'client': 'robust_camera'}).encode(),
            headers={'Content-Type': 'application/json', 'X-Control-Token': token})
        with urllib.request.urlopen(req, timeout=2):
            pass
        return True
    except Exception as exc:
        _log('关闭网页控制台相机失败: %s' % exc)
        return False


class Camera:
    def __init__(self, resolution=(640, 480)):
        self.cap = None
        self.width, self.height = resolution
        self.frame = None
        self.frame_at = 0.0
        self.opened = False
        self.flip, self.flip_param = False, 0
        if yaml_handle is not None:
            try:
                setting = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')
                self.flip, self.flip_param = setting['flip'], setting['flip_param']
            except Exception:
                pass
        self.th = threading.Thread(target=self.camera_task, daemon=True)
        self.th.start()

    def camera_open(self):
        self.opened = True

    def camera_close(self):
        self.opened = False
        time.sleep(0.2)
        self._release()

    def read(self):
        if self.frame is None or time.monotonic() - self.frame_at > FRESH:
            return False, None
        return True, self.frame

    def _release(self):
        cap, self.cap = self.cap, None
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

    def _open(self):
        dev = find_device()
        if dev is None:
            _log('摄像头未连接，等待中；检查 USB 线和供电')
            return False
        owners = holders(dev)
        if owners:
            if any('tonypi_web_control/server.py' in cmd for _, cmd in owners):
                _log('摄像头被网页控制台占用，正在关闭控制台相机')
                release_console_camera()
                time.sleep(0.5)
            else:
                _log('摄像头被占用: ' + '; '.join('PID %d %s' % o for o in owners))
            return False
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            _log('摄像头 %s 打不开，稍后重试' % dev)
            return False
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap = cap
        _log('摄像头已打开: %s' % dev)
        return True

    def camera_task(self):
        backoff = 0.5
        last_ok = time.monotonic()
        while True:
            if not self.opened:
                if self.cap is not None:
                    self._release()
                time.sleep(0.05)
                continue
            try:
                if self.cap is None:
                    if not self._open():
                        time.sleep(backoff)
                        backoff = min(backoff * 2, 3.0)
                        continue
                    last_ok = time.monotonic()
                ret, img = self.cap.read()
                if ret and img is not None:
                    if img.shape[1] != self.width or img.shape[0] != self.height:
                        img = cv2.resize(img, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
                    self.frame = cv2.flip(img, self.flip_param) if self.flip else img
                    self.frame_at = last_ok = time.monotonic()
                    backoff = 0.5
                elif time.monotonic() - last_ok > STALL:
                    _log('摄像头 %.0f 秒无画面，重新打开' % STALL)
                    self._release()
                else:
                    time.sleep(0.02)
            except Exception as exc:
                _log('读取摄像头出错: %s' % exc)
                self._release()
                time.sleep(backoff)


if __name__ == '__main__':
    cam = Camera()
    cam.camera_open()
    t0 = time.monotonic()
    n = 0
    try:
        while time.monotonic() - t0 < float(os.environ.get('SECONDS_TO_RUN', '1e9')):
            ok, img = cam.read()
            if ok:
                n += 1
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    print('fresh reads:', n)
    cam.camera_close()
