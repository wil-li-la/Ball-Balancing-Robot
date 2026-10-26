# Analysis of a run recorded by the dashboard (scripts/runs/*.jsonl): ball on plate, servo steps, PID term sizes,
# and a fit of the ball acceleration against the slope u, acc = M u(t - delay): M diagonal positive and near 14715 px/s^2
# means the whole chain is aligned and scaled as the model says.   python3 runfit.py runs/run_....jsonl
# analysis of a run recorded by the dashboard (stream.py messages, one per frame)
import sys, json, numpy as np
msgs = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
t = np.array([m["t"] for m in msgs]); n = len(msgs)
seen = np.array([m["ball"] is not None for m in msgs])
print(f"{n} frames, {t[-1]-t[0]:.1f} s, ball seen {seen.mean()*100:.0f}%")
# segments with the ball
with_ball = [m for m in msgs if m["ball"] is not None and m["u"] is not None]
tb = np.array([m["t"] for m in with_ball]); p = np.array([[m["ball"][0], m["ball"][1]] for m in with_ball]) - np.array([160, 120])
u = np.array([m["u"] for m in with_ball]); q = np.array([m["q_actual"] or m["q"] for m in with_ball]); r = np.array([m["ball"][2] for m in with_ball])
seg = np.r_[0, np.cumsum(np.diff(tb) > 0.08)]
print(f"ball radius px p5/50/95: {np.percentile(r,[5,50,95]).round(1)}  (expected 50)")
print(f"error rms [mm]: {np.sqrt(np.mean(p**2, axis=0)).round(1)/2.5}  |u| p50/p95: {np.percentile(np.hypot(u[:,0],u[:,1]),[50,95]).round(3)}")
dq = np.abs(np.diff(q, axis=0)).max(axis=1)[np.diff(tb) < 0.08]
print(f"servo step |dq| deg per frame p50/p90/p99/max: {np.percentile(dq,[50,90,99,100]).round(2)}  share > 5 deg: {(dq>5).mean():.3f}")
# PID terms: KP*e vs KD*de
KP, KD = 36e-5, 20e-5
e = -p; de = np.diff(e, axis=0) / np.diff(tb)[:, None]; ok = np.diff(tb) < 0.08
print(f"std KP*e {(KP*e).std(axis=0).round(4)}  std KD*de {(KD*de[ok]).std(axis=0).round(4)}")
# acceleration = M u(t - delay)
def acc(x, tt, w=7):
    h = w // 2; a = np.full(len(x), np.nan)
    for i in range(h, len(x) - h):
        T = tt[i-h:i+h+1] - tt[i]; A = np.c_[np.ones_like(T), T, T**2/2]
        a[i] = np.linalg.lstsq(A, x[i-h:i+h+1], rcond=None)[0][2]
    return a
best = None
for delay in range(0, 7):
    A, Y = [], []
    for s in range(seg.max() + 1):
        idx = np.where(seg == s)[0]
        if len(idx) < 15: continue
        a = np.c_[acc(p[idx,0], tb[idx]), acc(p[idx,1], tb[idx])]
        for j in range(delay, len(idx)):
            if not np.isnan(a[j]).any(): A.append(u[idx[j-delay]]); Y.append(a[j])
    A, Y = np.array(A), np.array(Y)
    if len(Y) < 20: continue
    M = np.linalg.lstsq(A, Y, rcond=None)[0].T
    r2 = 1 - ((Y - A @ M.T)**2).sum() / ((Y - Y.mean(0))**2).sum()
    print(f"delay {delay} ({delay*33:3d} ms) n={len(Y):4d} R2={r2:5.2f}  M=[[{M[0,0]:+7.0f} {M[0,1]:+7.0f}] [{M[1,0]:+7.0f} {M[1,1]:+7.0f}]]")
    if best is None or r2 > best[0]: best = (r2, delay, M)
r2, delay, M = best
ang = np.degrees(np.arctan2(M[1,0] - M[0,1], M[0,0] + M[1,1]))
print(f"best delay {delay} frames: diag {M[0,0]:.0f} / {M[1,1]:.0f} (theory 14715), rotation estimate {ang:+.1f} deg, R2 {r2:.2f}")
