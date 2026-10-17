from pathlib import Path
import argparse,hashlib,json
import cv2
P=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description='Extract the cited competition frames from an externally archived video.')
parser.add_argument('video',type=Path,help='Path to the original competition MP4; the video itself is not committed.')
args=parser.parse_args();src=args.video
cap=cv2.VideoCapture(str(src))
if not cap.isOpened():raise SystemExit('Cannot open the supplied video.')
fps=cap.get(cv2.CAP_PROP_FPS);count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
meta={'source':src.name,'fps':fps,'frames':count,'duration_s':count/fps,'width':int(cap.get(3)),'height':int(cap.get(4)),'sha256':hashlib.sha256(src.read_bytes()).hexdigest()}
out=P/'figures/video';out.mkdir(exist_ok=True);items=[]
for t in [8,76,102,133,165,184,196,218,234]:
 cap.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,frame=cap.read()
 if not ok:continue
 file=out/f'competition_{t:03d}s.jpg'
 file.write_bytes(cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,96])[1].tobytes())
 items.append({'id':f'V{t:03d}','requested_time_s':t,'frame_index':int(cap.get(cv2.CAP_PROP_POS_FRAMES))-1,'file':file.relative_to(P).as_posix()})
cap.release();meta['selected_frames']=items
(P/'data/video_evidence.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),'utf-8')
print('Extracted',len(items),'frames; original video remains external.')
