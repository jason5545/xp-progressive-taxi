"""滑行網路（1201/1202）跟地面漆的中心線（120 line，線型代碼）差多少。

用法：python3 -I cl_vs_net.py <apt.dat> [線型代碼,...]
對每條非跑道的滑行網路邊，取離兩端 > 25 m 的中段每 5 m 一點，量到最近一段漆線的距離。
"""
import math
import sys
from collections import defaultdict


def bez(p0, c0, c1, p1, n=12):
    out = []
    for i in range(1, n + 1):
        t = i / n
        u = 1 - t
        out.append((u**3 * p0[0] + 3 * u * u * t * c0[0] + 3 * u * t * t * c1[0] + t**3 * p1[0],
                    u**3 * p0[1] + 3 * u * u * t * c0[1] + 3 * u * t * t * c1[1] + t**3 * p1[1]))
    return out


def parse(path):
    nodes = {}
    edges = []
    lines = defaultdict(list)  # type -> list of segments ((x,y),(x,y)) in lat/lon
    feat = None
    chain = []

    def flush():
        # chain: list of (pt, ctrl or None, type or None, close?)
        if not chain:
            return
        cur = None
        pts = list(chain)
        if pts[-1][3]:
            pts.append(pts[0])
        for i in range(len(pts) - 1):
            p0, c0, t0, _ = pts[i]
            p1, c1, _, _ = pts[i + 1]
            if t0 is not None:
                cur = t0
            if cur is None:
                continue
            a = c0 if c0 else p0
            b = (2 * p1[0] - c1[0], 2 * p1[1] - c1[1]) if c1 else p1
            if c0 is None and c1 is None:
                poly = [p1]
            else:
                poly = bez(p0, a, b, p1)
            prev = p0
            for q in poly:
                lines[cur].append((prev, q))
                prev = q
        chain.clear()

    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            s = raw.split()
            if not s:
                continue
            code = s[0]
            if code in ("110", "120", "130"):
                flush()
                feat = code
                continue
            if code in ("111", "112", "113", "114", "115", "116"):
                if feat != "120":
                    continue
                p = (float(s[1]), float(s[2]))
                if code in ("112", "114", "116"):
                    c = (float(s[3]), float(s[4]))
                    rest = s[5:]
                else:
                    c = None
                    rest = s[3:]
                t = int(rest[0]) if rest else None
                close = code in ("113", "114")
                chain.append((p, c, t, close))
                if code in ("113", "114", "115", "116"):
                    flush()
                continue
            if code == "1201":
                nodes[int(s[4])] = (float(s[1]), float(s[2]))
            elif code == "1202":
                kind = s[4] if len(s) > 4 else ""
                name = s[5] if len(s) > 5 else ""
                edges.append((int(s[1]), int(s[2]), kind, name))
            else:
                if feat == "120":
                    flush()
                feat = None
    flush()
    return nodes, edges, lines


def main():
    path = sys.argv[1]
    types = [int(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2 else None
    nodes, edges, lines = parse(path)
    if not nodes:
        print("no network")
        return
    lat0 = sum(v[0] for v in nodes.values()) / len(nodes)
    lon0 = sum(v[1] for v in nodes.values()) / len(nodes)
    kx = 111320 * math.cos(math.radians(lat0))
    ky = 110540

    def P(ll):
        return ((ll[1] - lon0) * kx, (ll[0] - lat0) * ky)

    segs = defaultdict(list)
    for t, ss in lines.items():
        for a, b in ss:
            segs[t].append((P(a), P(b)))
    if types is None:
        print("types:", sorted((t, len(v)) for t, v in segs.items()))
        return
    S = [s for t in types for s in segs.get(t, [])]
    # grid
    G = 20.0
    grid = defaultdict(list)
    for s in S:
        (x0, y0), (x1, y1) = s
        for gx in range(int(math.floor(min(x0, x1) / G)), int(math.floor(max(x0, x1) / G)) + 1):
            for gy in range(int(math.floor(min(y0, y1) / G)), int(math.floor(max(y0, y1) / G)) + 1):
                grid[(gx, gy)].append(s)

    def dist(p):
        best = 1e9
        gx, gy = int(math.floor(p[0] / G)), int(math.floor(p[1] / G))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for (a, b) in grid.get((gx + dx, gy + dy), ()):
                    vx, vy = b[0] - a[0], b[1] - a[1]
                    L = vx * vx + vy * vy
                    t = 0 if L == 0 else max(0, min(1, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L))
                    d = math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)
                    best = min(best, d)
        return best

    ds = []
    per_edge = []
    n_tw = 0
    for a, b, kind, name in edges:
        if kind.startswith("runway"):
            continue
        n_tw += 1
        pa, pb = P(nodes[a]), P(nodes[b])
        L = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
        if L < 60:
            continue
        e = []
        s = 25.0
        while s <= L - 25:
            t = s / L
            q = (pa[0] + t * (pb[0] - pa[0]), pa[1] + t * (pb[1] - pa[1]))
            e.append(dist(q))
            s += 5
        if e:
            ds += e
            e.sort()
            per_edge.append((e[len(e) // 2], name, round(L)))
    ds.sort()
    if not ds:
        print("no samples")
        return
    q = lambda f: ds[min(len(ds) - 1, int(f * len(ds)))]
    far = sum(1 for d in ds if d > 20) / len(ds)
    print(f"{path.split('/')[-1]} types={types} 線段={len(S)} 滑行道邊={n_tw} 取樣={len(ds)} "
          f"p50={q(.5):.2f} p90={q(.9):.2f} p95={q(.95):.2f} max={'%.1f' % ds[-1] if ds[-1] < 1e8 else 'none'} "
          f"≤1m={sum(1 for d in ds if d <= 1) / len(ds):.0%} ≤3m={sum(1 for d in ds if d <= 3) / len(ds):.0%} "
          f">20m(沒漆線)={far:.0%}")
    per_edge.sort(reverse=True)
    print("  中位數最大的邊:", [(round(m, 1) if m < 1e8 else None, n, L) for m, n, L in per_edge[:6]])


main()
