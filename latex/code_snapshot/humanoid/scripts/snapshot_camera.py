"""Bounded HTTP JPEG capture sharing an existing camera owner."""
import hashlib
import time
import urllib.request
import urllib.error


class SnapshotCapture:
    def __init__(self,config,cv2,opener=None):
        self.url=config['url']
        if not self.url.startswith(('http://','https://')):
            raise ValueError('HTTP snapshot URL required')
        self.timeout=float(config.get('timeout',.8))
        fps=float(config.get('max_fps',10))
        if not .1<=self.timeout<=2 or not 1<=fps<=30:
            raise ValueError('Snapshot timeout .1..2 seconds; max_fps 1..30')
        self.interval=1/fps
        self.cv2=cv2
        self.opener=opener or urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.closed=False
        self.last_request=0
        self.last_digest=None
        self.last_error=''

    def read(self):
        import numpy as np
        if self.closed: return False,None
        remaining=self.interval-(time.monotonic()-self.last_request)
        if remaining>0: time.sleep(remaining)
        if self.closed: return False,None
        self.last_request=time.monotonic()
        try:
            request=urllib.request.Request(self.url,headers={'Accept':'image/jpeg','Cache-Control':'no-cache'})
            with self.opener.open(request,timeout=self.timeout) as response:
                data=response.read(2*1024*1024+1)
            if not data or len(data)>2*1024*1024:
                raise ValueError('Snapshot empty or exceeds 2 MiB')
            digest=hashlib.sha256(data).digest()
            if digest==self.last_digest:
                self.last_error='Snapshot unchanged; not a fresh observation'
                return False,None
            frame=self.cv2.imdecode(np.frombuffer(data,np.uint8),self.cv2.IMREAD_COLOR)
            if frame is None: raise ValueError('Snapshot is not a decodable image')
            self.last_digest=digest
            self.last_error=''
            return True,frame
        except (urllib.error.URLError,OSError,ValueError) as exc:
            self.last_error=str(exc)
            return False,None

    def release(self):
        self.closed=True
