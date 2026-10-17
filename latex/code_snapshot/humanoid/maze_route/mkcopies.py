"""Copies of the base actions for the maze route, in ~/TonyPi/maze/actions/.

Frames, timings and legs are the originals; only servos 6/7/8/14/15/16 are set
to the last frame of grab_final_v2 (grab_final_v2 itself is copied unchanged).
"""
import hashlib, shutil, sqlite3
from contextlib import closing
from pathlib import Path
AG = Path('/home/pi/TonyPi/ActionGroups')
OUT = Path('/home/pi/TonyPi/maze/actions')
ARMS = (6, 7, 8, 14, 15, 16)

def rows(p):
    with closing(sqlite3.connect('file:%s?mode=ro' % p, uri=True)) as db:
        return [list(r) for r in db.execute('select * from ActionGroup order by "Index"')]

OUT.mkdir(parents=True, exist_ok=True)
grab = rows(AG / 'grab_final_v2.d6a')
arms = {i: grab[-1][1 + i] for i in ARMS}
shutil.copyfile(str(AG / 'grab_final_v2.d6a'), str(OUT / 'grab_final_v2.d6a'))
for name in ('catch_ball_go', 'catch_ball_turn_left', 'catch_ball_turn_right'):
    src, dst = AG / (name + '.d6a'), OUT / (name + '.d6a')
    shutil.copyfile(str(src), str(dst))
    with closing(sqlite3.connect(str(dst))) as db:
        for i in ARMS:
            db.execute('UPDATE ActionGroup SET Servo%d = ?' % i, (arms[i],))
        db.commit()
        db.execute('VACUUM')
    a, b = rows(src), rows(dst)
    assert len(a) == len(b)
    for x, y in zip(a, b):
        assert x[0] == y[0] and x[1] == y[1], 'index/time'
        for i in range(1, 19):
            assert y[1 + i] == (arms[i] if i in ARMS else x[1 + i]), 'servo %d' % i
    print('%-22s %2d frames %4d ms  only arms changed  sha %s' % (name, len(b), sum(r[1] for r in b),
          hashlib.sha256(dst.read_bytes()).hexdigest()[:12]))
print('%-22s %2d frames %4d ms  unchanged copy' % ('grab_final_v2', len(grab), sum(r[1] for r in grab)))
print('arms (from grab_final_v2 last frame):', arms)

# put_down: official "put on the floor". Only its first frame (squatting while
# still holding) gets the grab_final_v2 hands, so the cube is held tight on
# the way down; lowering, opening and standing up are the original frames.
src, dst = AG / 'put_down.d6a', OUT / 'put_down.d6a'
shutil.copyfile(str(src), str(dst))
with closing(sqlite3.connect(str(dst))) as db:
    for i in ARMS:
        db.execute('UPDATE ActionGroup SET Servo%d = ? WHERE "Index" = (SELECT MIN("Index") FROM ActionGroup)' % i, (arms[i],))
    db.commit()
    db.execute('VACUUM')
a, b = rows(src), rows(dst)
assert len(a) == len(b) and all(x[:2] == y[:2] for x, y in zip(a, b))
assert all(b[0][1 + i] == arms[i] for i in ARMS) and all(b[0][1 + i] == a[0][1 + i] for i in range(1, 19) if i not in ARMS)
assert [r[2:] for r in a[1:]] == [r[2:] for r in b[1:]]
print('%-22s %2d frames %4d ms  first frame hands = grab_final_v2' % ('put_down', len(b), sum(r[1] for r in b)))
