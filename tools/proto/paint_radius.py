"""Step 0: radius of painted centreline fillets at taxi network junctions.

python3 -I paint_radius.py <apt.dat> 1,51
Each painted chain (120 rows) of the given line types is split into pieces
between chain nodes; consecutive curved (bezier) pieces turning the same way
are merged; a curve turning 30..135 deg (U-turn loops and line caps are
excluded) is matched to a network node (<= 60 m) whose two incident taxiway
edges line up with the curve's entry and exit tangents (<= 20 deg). Radius = arc length / turn. Grouped by the width
classes of those two edges.
"""
import math
import sys
from collections import defaultdict


def bez_pts(p0, c0, c1, p1, n=24):
    out = []
    for i in range(0, n + 1):
        t = i / n
        u = 1 - t
        out.append((u**3 * p0[0] + 3 * u * u * t * c0[0] + 3 * u * t * t * c1[0] + t**3 * p1[0],
                    u**3 * p0[1] + 3 * u * u * t * c0[1] + 3 * u * t * t * c1[1] + t**3 * p1[1]))
    return out


def hdg(a, b):
    return math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360


def rel(h1, h2):
    d = (h2 - h1 + 540) % 360 - 180
    return d


def main():
    path = sys.argv[1]
    types = set(int(x) for x in sys.argv[2].split(","))
    nodes = {}
    edges = []
    chains = []
    feat = None
    chain = []
    with open(path, encoding="latin-1") as f:
        for raw in f:
            s = raw.split()
            if not s:
                continue
            c = s[0]
            if c in ("110", "120", "130"):
                if chain:
                    chains.append(chain)
                chain = []
                feat = c
                continue
            if c in ("111", "112", "113", "114", "115", "116"):
                if feat != "120":
                    continue
                p = (float(s[1]), float(s[2]))
                if c in ("112", "114", "116"):
                    cp = (float(s[3]), float(s[4]))
                    rest = s[5:]
                else:
                    cp = None
                    rest = s[3:]
                t = int(rest[0]) if rest else None
                chain.append((p, cp, t))
                if c in ("113", "114"):
                    chain.append(chain[0][:2] + (None,))
                if c in ("113", "114", "115", "116"):
                    chains.append(chain)
                    chain = []
                continue
            if feat == "120" and chain:
                chains.append(chain)
                chain = []
            feat = None
            if c == "1201":
                nodes[int(s[4])] = (float(s[1]), float(s[2]))
            elif c == "1202":
                kind = s[4] if len(s) > 4 else ""
                edges.append((int(s[1]), int(s[2]), kind))
    if chain:
        chains.append(chain)
    lat0 = sum(v[0] for v in nodes.values()) / len(nodes)
    lon0 = sum(v[1] for v in nodes.values()) / len(nodes)
    k = math.pi / 180 * 6371008.8
    cl = math.cos(math.radians(lat0))

    def P(ll):
        return ((ll[1] - lon0) * cl * k, (ll[0] - lat0) * k)

    # curves
    curves = []
    for ch in chains:
        cur_t = None
        pieces = []  # (type, pts, curved)
        for i in range(len(ch) - 1):
            p0, c0, t0 = ch[i]
            p1, c1, _ = ch[i + 1]
            if t0 is not None:
                cur_t = t0
            if cur_t not in types:
                pieces.append(None)
                continue
            q0, q1 = P(p0), P(p1)
            if c0 is None and c1 is None:
                pieces.append(([q0, q1], False))
                continue
            a = P(c0) if c0 else q0
            b = (2 * q1[0] - P(c1)[0], 2 * q1[1] - P(c1)[1]) if c1 else q1
            pieces.append((bez_pts(q0, a, b, q1), True))
        # merge consecutive curved pieces with consistent turn sign
        run = []

        def close_run():
            if not run:
                return
            pts = [run[0][0]]
            for pp in run:
                pts += pp[1:]
            L = sum(math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1]) for j in range(len(pts) - 1))
            hs = [hdg(pts[j], pts[j + 1]) for j in range(len(pts) - 1)
                  if math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1]) > 1e-6]
            if len(hs) < 2:
                run.clear()
                return
            turn = sum(rel(hs[j], hs[j + 1]) for j in range(len(hs) - 1))
            if 30 <= abs(turn) <= 135:
                curves.append((pts, L, turn, hs[0], hs[-1]))
            run.clear()

        sign = 0
        for pc in pieces:
            if pc is None or not pc[1]:
                close_run()
                sign = 0
                continue
            pts = pc[0]
            hs = [hdg(pts[j], pts[j + 1]) for j in range(len(pts) - 1)
                  if math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1]) > 1e-6]
            if len(hs) < 2:
                continue
            t = sum(rel(hs[j], hs[j + 1]) for j in range(len(hs) - 1))
            sg = 1 if t > 0.5 else (-1 if t < -0.5 else 0)
            if sg == 0:
                close_run()
                sign = 0
                continue
            if run and sg != sign:
                close_run()
            sign = sg
            run.append(pts)
        close_run()

    # network
    deg = defaultdict(int)
    inc = defaultdict(list)
    for a, b, kind in edges:
        if a not in nodes or b not in nodes:
            continue
        deg[a] += 1
        deg[b] += 1
    for a, b, kind in edges:
        if a not in nodes or b not in nodes or kind.startswith("runway"):
            continue
        if deg[a] == 1 or deg[b] == 1:
            continue
        cls = kind[-1] if kind.startswith("taxiway_") else "-"
        pa, pb = P(nodes[a]), P(nodes[b])
        inc[a].append((hdg(pa, pb), cls))
        inc[b].append((hdg(pb, pa), cls))
    npos = {n: P(nodes[n]) for n in inc}
    groups = defaultdict(list)
    unmatched = 0
    for pts, L, turn, h0, h1 in curves:
        mid = pts[len(pts) // 2]
        best = None
        for n, q in npos.items():
            d = math.hypot(q[0] - mid[0], q[1] - mid[1])
            if d > 60:
                continue
            # curve enters along an edge arriving at n (heading h0 = reverse of
            # the outgoing edge direction) and leaves along an outgoing edge
            ein = [c for h, c in inc[n] if abs(rel((h + 180) % 360, h0)) <= 20]
            eout = [c for h, c in inc[n] if abs(rel(h, h1)) <= 20]
            if ein and eout and (best is None or d < best[0]):
                best = (d, ein[0], eout[0])
        if best is None:
            unmatched += 1
            continue
        key = "".join(sorted(best[1] + best[2]))
        groups[key].append(L / math.radians(abs(turn)))
        groups["min=" + min(best[1], best[2])].append(L / math.radians(abs(turn)))
    print(f"{path.split('/')[-1]} 線型 {sorted(types)}：彎道（轉 30–135°）{len(curves)} 個，"
          f"對上路口 {len(curves) - unmatched} 個，對不上 {unmatched} 個")
    for key in sorted(groups):
        v = sorted(groups[key])
        q = lambda f: v[min(len(v) - 1, int(f * len(v)))]
        print(f"  {key:6s} n={len(v):4d}  p10={q(.1):6.1f}  p25={q(.25):6.1f}  p50={q(.5):6.1f}  "
              f"p75={q(.75):6.1f}  p90={q(.9):6.1f}  min={v[0]:5.1f}  max={v[-1]:6.1f} m")


main()
