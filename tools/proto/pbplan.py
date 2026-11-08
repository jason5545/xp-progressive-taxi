#!/usr/bin/env python3
"""
第 0 步原型：自動後推終點選擇 + 滑行路線（離線，不依賴 X-Plane）。

- compute_segs()：忠實移植 src/driving.c 的 compute_segs_impl / oblique，
  幾何函式照 libacfutils geom.c / helpers.c 的定義。
- apt.dat：1300/1301 停機位、100 跑道、1201 節點、1202 邊、1204 active zone。
- 路線：以「有向邊」為狀態的反向 Dijkstra（可以表達轉彎角限制與 oneway）。

用法：
  python3 -I pbplan.py --apt RCTP.dat --icao RCTP --out out/ [--fleet class|b738] [--stand-ref nose|main]
"""
import argparse
import bisect
import collections
import heapq
import json
import math
import os
import re
import sys

# ---------------------------------------------------------------------------
# libacfutils geometry (faithful)
# ---------------------------------------------------------------------------
NAN = float("nan")
NULLV = (NAN, NAN)
ROUND_ERROR = 1e-10


def isnull(v):
    return math.isnan(v[0])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def scmul(a, s):
    return (a[0] * s, a[1] * s)


def vabs(a):
    return math.hypot(a[0], a[1])


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def neg(a):
    return (-a[0], -a[1])


def mean(a, b):
    return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)


def set_abs(a, l):
    o = vabs(a)
    if o != 0.0:
        return scmul(a, l / o)
    return (0.0, 0.0)


def vnorm(v, right):
    return (v[1], -v[0]) if right else (-v[1], v[0])


def vrot(v, a):
    s, c = math.sin(math.radians(-a)), math.cos(math.radians(-a))
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c)


def hdg2dir(h):
    r = math.radians(h)
    return (math.sin(r), math.cos(r))


def _asin(x):
    if x > 1.0 or x < -1.0:
        return NAN
    return math.asin(x)


def dir2hdg(d):
    x, y = d
    a = vabs(d)
    if x >= 0 and y >= 0:
        res = math.degrees(_asin(x / a))
    elif x < 0 and y >= 0:
        res = 360 + math.degrees(_asin(x / a))
    else:
        res = 180 - math.degrees(_asin(x / a))
    return res


def rel_hdg(h1, h2):
    if h1 > h2:
        if h1 > h2 + 180:
            return 360 - h1 + h2
        return -(h1 - h2)
    if h2 > h1 + 180:
        return -(360 - h2 + h1)
    return h2 - h1


def normalize_hdg(h):
    h = math.fmod(h, 360.0)
    if h < 0:
        h += 360.0
    if h <= 0.0 or h > 360.0:
        h = min(max(h, 0.0), 360.0)
    return h


def isect(a, oa, b, ob, confined):
    if oa == ob:
        return oa
    p1, p2, p3, p4 = oa, add(oa, a), ob, add(ob, b)
    det = (p1[0] - p2[0]) * (p3[1] - p4[1]) - (p1[1] - p2[1]) * (p3[0] - p4[0])
    if det == 0.0:
        return NULLV
    ca = p1[0] * p2[1] - p1[1] * p2[0]
    cb = p3[0] * p4[1] - p3[1] * p4[0]
    rx = (ca * (p3[0] - p4[0]) - cb * (p1[0] - p2[0])) / det
    ry = (ca * (p3[1] - p4[1]) - cb * (p1[1] - p2[1])) / det
    if confined:
        E = ROUND_ERROR
        if (rx < min(p1[0], p2[0]) - E or rx > max(p1[0], p2[0]) + E or
                rx < min(p3[0], p4[0]) - E or rx > max(p3[0], p4[0]) + E or
                ry < min(p1[1], p2[1]) - E or ry > max(p1[1], p2[1]) + E or
                ry < min(p3[1], p4[1]) - E or ry > max(p3[1], p4[1]) + E):
            return NULLV
    return (rx, ry)


# ---------------------------------------------------------------------------
# compute_segs (src/driving.c)
# ---------------------------------------------------------------------------
SEG_TURN_MULT = 0.9
MIN_TURN_RADIUS = 1.5
MIN_SEG_LEN = 0.1
OBLIQUE_RADIUS_FACT = 1.02
STRAIGHT_SEG_ANGLE_LIM = 1


class Veh:
    def __init__(self, wheelbase, max_steer):
        self.wheelbase = wheelbase
        self.max_steer = max_steer

    def min_radius(self):
        return max(math.tan(math.radians(90 - self.max_steer * SEG_TURN_MULT)) *
                   self.wheelbase, MIN_TURN_RADIUS)


def _seg(kind, sp, sh, ep, eh, backward, length=0.0, r=0.0, right=False):
    return {"type": kind, "sp": sp, "sh": sh, "ep": ep, "eh": eh,
            "backward": backward, "len": length, "r": r, "right": right}


def _construct_oblique(veh, sp, sh, mid, mh, ep, eh, segs):
    n1 = _impl(veh, sp, sh, mid, mh, segs, False)
    if n1 == -1:
        return -1
    n2 = _impl(veh, mid, mh, ep, eh, segs, False)
    if n2 == -1:
        for _ in range(n1):
            segs.pop()
        return -1
    return n1 + n2


def _oblique(veh, sp, sh, ep, eh, segs, backward, radius):
    d1, d2 = hdg2dir(sh), hdg2dir(eh)
    sv = scmul(d1, -1e10 if backward else 1e10)
    ev = scmul(d2, 1e10 if backward else -1e10)
    for i in (0, 1):
        for j in (0, 1):
            c1 = add(sp, scmul(vnorm(d1, i), radius))
            c2 = add(ep, scmul(vnorm(d2, j), radius))
            s = mean(c1, c2)
            s2c1, s2c2 = sub(c1, s), sub(c2, s)
            if (not isnull(isect(s2c1, s, sv, sp, True)) or
                    not isnull(isect(s2c2, s, ev, ep, True))):
                continue
            a = math.degrees(_asin(radius / vabs(s2c1)))
            if math.isnan(a):
                continue
            for sa in (a, -a):
                if (not isnull(isect(set_abs(vrot(s2c1, sa), 1e10), s, sv, sp, True)) and
                        not isnull(isect(set_abs(vrot(s2c2, sa), 1e10), s, ev, ep, True))):
                    n = _construct_oblique(veh, sp, sh, s,
                                           dir2hdg(vrot(s2c1 if backward else s2c2, sa)),
                                           ep, eh, segs)
                    if n != -1:
                        return n
    return -1


def _impl(veh, sp, sh, ep, eh, segs, recurse):
    if dist(sp, ep) < MIN_SEG_LEN:
        return 0 if abs(sh - eh) < STRAIGHT_SEG_ANGLE_LIM else -1
    s2e = sub(ep, sp)
    rhdg = rel_hdg(sh, dir2hdg(s2e))
    backward = abs(rhdg) > 90
    min_radius = veh.min_radius()

    if abs(sh - eh) < STRAIGHT_SEG_ANGLE_LIM and (
            abs(rhdg) < STRAIGHT_SEG_ANGLE_LIM or abs(rhdg) > 180 - STRAIGHT_SEG_ANGLE_LIM):
        dir_v = hdg2dir(sh + (180 if backward else 0))
        ln = dot(dir_v, s2e)
        ep2 = add(set_abs(dir_v, ln), sp)
        segs.append(_seg("S", sp, sh, ep2, eh, backward, length=ln))
        return 1

    s1_v = scmul(hdg2dir(sh), 1e10)
    if backward:
        s1_v = neg(s1_v)
    s2_v = scmul(hdg2dir(eh), 1e10)
    if not backward:
        s2_v = neg(s2_v)
    te = isect(s1_v, sp, s2_v, ep, True)
    if isnull(te):
        if recurse:
            return _oblique(veh, sp, sh, ep, eh, segs, backward,
                            min_radius * OBLIQUE_RADIUS_FACT)
        return -1
    l1, l2 = dist(te, sp), dist(te, ep)
    x = min(l1, l2)
    l1 -= x
    l2 -= x
    a = 180 - abs(rel_hdg(sh, eh))
    r = x * math.tan(math.radians(a / 2))
    if r < min_radius:
        if recurse:
            return _oblique(veh, sp, sh, ep, eh, segs, backward,
                            min_radius * OBLIQUE_RADIUS_FACT)
        return -1
    if l1 == 0:
        s2start = add(ep, set_abs(s2_v, l2))
        s2 = _seg("S", s2start, eh, ep, eh, backward, length=l2)
        s1 = _seg("T", sp, sh, s2start, eh, backward, r=r, right=(rhdg >= 0))
    else:
        s1end = add(sp, set_abs(s1_v, l1))
        s1 = _seg("S", sp, sh, s1end, sh, backward, length=l1)
        s2 = _seg("T", s1end, sh, ep, eh, backward, r=r, right=(rhdg >= 0))
    segs.append(s1)
    segs.append(s2)
    return 2


def compute_segs(veh, sp, sh, ep, eh):
    segs = []
    n = _impl(veh, sp, sh, ep, eh, segs, True)
    return n, segs


def seg_len(s):
    if s["type"] == "S":
        return abs(s["len"])
    return s["r"] * abs(math.radians(rel_hdg(s["sh"], s["eh"])))


def seg_points(s, step=2.0):
    """Sample a segment polyline by integrating the motion direction."""
    sign = -1.0 if s["backward"] else 1.0
    if s["type"] == "S":
        L = dist(s["sp"], s["ep"])
        n = max(1, int(math.ceil(L / step)))
        return [add(s["sp"], scmul(sub(s["ep"], s["sp"]), k / n)) for k in range(n + 1)]
    d = rel_hdg(s["sh"], s["eh"])
    L = seg_len(s)
    n = max(8, int(math.ceil(abs(d) / 4.0)))
    p = s["sp"]
    pts = [p]
    for k in range(n):
        h = s["sh"] + d * (k + 0.5) / n
        p = add(p, scmul(hdg2dir(h), sign * L / n))
        pts.append(p)
    return pts


# ---------------------------------------------------------------------------
# aircraft profiles (read from the installed .acf files)
# ---------------------------------------------------------------------------
FT = 0.3048
XP = os.path.expanduser("~/X-Plane 12/Aircraft")
ACF = {
    "A": ("Cessna 172 SP", "Laminar Research/Cessna 172 SP/Cessna_172SP.acf", 11.0),
    "B": ("King Air C90B", "Laminar Research/Beechcraft King Air C90B/C90B.acf", 15.3),
    "C": ("Boeing 737-800", "Laminar Research/Boeing 737-800/b738.acf", 35.8),
    "D": ("Airbus A330-300 (D 級代理)", "Laminar Research/Airbus A330-300/A330.acf", 60.3),
    "E": ("Boeing 777-200ER", "FlightFactor777_200ER_F/777-200ER_xp12.acf", 60.9),
    "F": ("Boeing 777-200ER (F 級代理)", "FlightFactor777_200ER_F/777-200ER_xp12.acf", 60.9),
}


def read_acf(path):
    g = collections.defaultdict(dict)
    rx = re.compile(r"^P _gear/(\d+)/_(gear_type|gear_x|gear_z|steerdeg_lospeed|steerdeg_hispeed) (\S+)")
    with open(path, encoding="latin-1", errors="ignore") as f:
        for line in f:
            if not line.startswith("P _gear/"):
                continue
            m = rx.match(line)
            if m:
                g[int(m.group(1))][m.group(2)] = float(m.group(3))
    gears = [v for v in g.values() if v.get("gear_type", 0) != 0]
    center = [v for v in gears if abs(v["gear_x"]) < 0.5]
    nose = min(center, key=lambda v: v["gear_z"])
    mains = [v for v in gears if abs(v["gear_x"]) >= 0.5]
    main_z = sum(v["gear_z"] for v in mains) / len(mains)
    wb = (main_z - nose["gear_z"]) * FT
    steer = max(nose.get("steerdeg_lospeed", 0), nose.get("steerdeg_hispeed", 0))
    # bp.c: MIN(max(steerdeg1, steerdeg2), max_steer_angle()=75 on XP11+ dry)
    max_steer = min(steer, 75.0)
    if max_steer < 35:
        max_steer = (75.0 + 35.0) / 2
    return wb, max_steer, steer


_acf_cache = {}


def aircraft(cls, fleet):
    if fleet == "b738":
        cls_key = "C"
    else:
        cls_key = cls
    if cls_key not in _acf_cache:
        name, rel, span = ACF[cls_key]
        wb, ms, raw = read_acf(os.path.join(XP, rel))
        _acf_cache[cls_key] = {"name": name, "wheelbase": wb, "max_steer": ms,
                               "raw_steer": raw, "span": span, "cls": cls_key}
    ac = dict(_acf_cache[cls_key])
    # 路線用的寬度等級：代理機型實際翼展的等級，跟停機位等級取小的；b738 固定 C
    span = ac["span"]
    own = "A" if span < 15 else "B" if span < 24 else "C" if span < 36 else "D" if span < 52 else "E" if span < 65 else "F"
    ac["route_cls"] = min(cls, own, key=CLS_ORDER.index) if fleet == "class" else "C"
    return ac


# ---------------------------------------------------------------------------
# apt.dat parsing
# ---------------------------------------------------------------------------
class Airport:
    pass


def parse_apt(path):
    ap = Airport()
    ap.runways, ap.stands, ap.nodes, ap.edges = [], [], {}, []
    ap.name = ""
    last_edge, last_stand = None, None
    with open(path, encoding="latin-1") as f:
        for line in f:
            p = line.split()
            if not p:
                continue
            c = p[0]
            if c in ("1", "16", "17") and len(p) >= 5:
                ap.icao = p[4]
                ap.name = " ".join(p[5:])
            elif c == "100":
                ap.runways.append({
                    "width": float(p[1]),
                    "ends": [(p[8], float(p[9]), float(p[10]), float(p[11])),
                             (p[17], float(p[18]), float(p[19]), float(p[20]))]})
            elif c == "1300":
                last_stand = {"lat": float(p[1]), "lon": float(p[2]),
                              "hdg": normalize_hdg(float(p[3])), "type": p[4],
                              "actypes": p[5] if len(p) > 5 else "",
                              "name": " ".join(p[6:]), "cls": None}
                ap.stands.append(last_stand)
            elif c == "1301" and last_stand is not None:
                if len(p) > 1 and len(p[1]) == 1:
                    last_stand["cls"] = p[1]
            elif c == "1201":
                ap.nodes[int(p[4])] = {"lat": float(p[1]), "lon": float(p[2]),
                                       "usage": p[3], "name": " ".join(p[5:])}
            elif c == "1202":
                typ = p[4] if len(p) > 4 else "taxiway"
                cls = None
                kind = "runway" if typ == "runway" else "taxiway"
                m = re.match(r"taxiway_([A-F])$", typ)
                if m:
                    cls = m.group(1)
                last_edge = {"a": int(p[1]), "b": int(p[2]), "oneway": p[3] == "oneway",
                             "kind": kind, "cls": cls,
                             "name": " ".join(p[5:]) if len(p) > 5 else "",
                             "zones": []}
                ap.edges.append(last_edge)
            elif c == "1204" and last_edge is not None:
                if len(p) >= 3:
                    last_edge["zones"].append((p[1], p[2].split(",")))
            elif c == "1206":
                last_edge = None
    return ap


def project(ap):
    lats = [e[1] for r in ap.runways for e in r["ends"]]
    lons = [e[2] for r in ap.runways for e in r["ends"]]
    lat0, lon0 = sum(lats) / len(lats), sum(lons) / len(lons)
    k = math.pi / 180 * 6371008.8
    cl = math.cos(math.radians(lat0))

    def pr(lat, lon):
        return ((lon - lon0) * cl * k, (lat - lat0) * k)
    ap.origin = (lat0, lon0)
    for n in ap.nodes.values():
        n["p"] = pr(n["lat"], n["lon"])
    for s in ap.stands:
        s["p"] = pr(s["lat"], s["lon"])
    for r in ap.runways:
        r["p"] = [pr(e[1], e[2]) for e in r["ends"]]
        u = sub(r["p"][1], r["p"][0])
        L = vabs(u)
        r["len"] = L
        r["u"] = scmul(u, 1 / L)


# ---------------------------------------------------------------------------
# graph
# ---------------------------------------------------------------------------
CLS_ORDER = "ABCDEF"


class Graph:
    def __init__(self, ap):
        self.ap = ap
        E = []
        for i, e in enumerate(ap.edges):
            if e["a"] not in ap.nodes or e["b"] not in ap.nodes or e["a"] == e["b"]:
                continue
            A, B = ap.nodes[e["a"]]["p"], ap.nodes[e["b"]]["p"]
            L = dist(A, B)
            if L < 0.05:
                continue
            e["A"], e["B"], e["L"], e["id"] = A, B, L, len(E)
            e["hdg"] = dir2hdg(sub(B, A))
            E.append(e)
        self.edges = E
        # directed edges
        self.de = []           # (eid, frm, to, hdg, L)
        self.de_index = {}     # (eid, +1/-1) -> de id
        self.out = collections.defaultdict(list)
        self.inc = collections.defaultdict(list)
        for e in E:
            dirs = [(+1, e["a"], e["b"], e["hdg"])]
            if not e["oneway"]:
                dirs.append((-1, e["b"], e["a"], normalize_hdg(e["hdg"] + 180)))
            for sgn, f, t, h in dirs:
                did = len(self.de)
                self.de.append((e["id"], f, t, h, e["L"]))
                self.de_index[(e["id"], sgn)] = did
                self.out[f].append(did)
                self.inc[t].append(did)
        self.deg = collections.Counter()
        self.node_edges = collections.defaultdict(list)
        for e in E:
            self.deg[e["a"]] += 1
            self.deg[e["b"]] += 1
            self.node_edges[e["a"]].append(e["id"])
            self.node_edges[e["b"]].append(e["id"])
        # grid
        self.cell = 50.0
        self.grid = collections.defaultdict(list)
        for e in E:
            x0, x1 = sorted((e["A"][0], e["B"][0]))
            y0, y1 = sorted((e["A"][1], e["B"][1]))
            for gx in range(int(math.floor(x0 / self.cell)), int(math.floor(x1 / self.cell)) + 1):
                for gy in range(int(math.floor(y0 / self.cell)), int(math.floor(y1 / self.cell)) + 1):
                    self.grid[(gx, gy)].append(e["id"])
        # 停機範圍：從停機位點往機尾方向一個機身長的線段
        self.foot = []
        for s in ap.stands:
            c = s["cls"] or "C"
            back = scmul(hdg2dir(s["hdg"]), -FUSE_LEN.get(c, 38.0) * 0.95)
            self.foot.append((s, s["p"], add(s["p"], back)))
        # lead-in stubs: dead-end node within 20 m of a stand
        self.leadin = set()
        for e in E:
            if e["kind"] != "taxiway":
                continue
            for nid in (e["a"], e["b"]):
                if self.deg[nid] == 1:
                    np_ = self.ap.nodes[nid]["p"]
                    if any(dist(np_, s["p"]) < 20 for s in ap.stands):
                        self.leadin.add(e["id"])

    def near(self, p, R):
        c = self.cell
        res = set()
        for gx in range(int(math.floor((p[0] - R) / c)), int(math.floor((p[0] + R) / c)) + 1):
            for gy in range(int(math.floor((p[1] - R) / c)), int(math.floor((p[1] + R) / c)) + 1):
                res.update(self.grid.get((gx, gy), ()))
        return [self.edges[i] for i in res if seg_dist(p, self.edges[i]["A"], self.edges[i]["B"])[0] <= R]

    def nearest_taxi(self, p, R=60.0, exclude_leadin=False):
        best = (1e18, None, None)
        for e in self.near(p, R):
            if e["kind"] != "taxiway":
                continue
            if exclude_leadin and e["id"] in self.leadin:
                continue
            d, t = seg_dist(p, e["A"], e["B"])
            if d < best[0]:
                best = (d, e, t)
        return best


def seg_dist(p, A, B):
    ab = sub(B, A)
    L2 = dot(ab, ab)
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, dot(sub(p, A), ab) / L2))
    q = add(A, scmul(ab, t))
    return dist(p, q), t


def seg_isect(p1, p2, p3, p4):
    d1 = sub(p2, p1)
    d2 = sub(p4, p3)
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if den == 0:
        return None
    w = sub(p3, p1)
    t = (w[0] * d2[1] - w[1] * d2[0]) / den
    u = (w[0] * d1[1] - w[1] * d1[0]) / den
    if 0 <= t <= 1 and 0 <= u <= 1:
        return add(p1, scmul(d1, t))
    return None


def cls_ok(edge_cls, ac_cls):
    if edge_cls is None or ac_cls is None:
        return True
    return CLS_ORDER.index(edge_cls) >= CLS_ORDER.index(ac_cls)


def rwy_tokens(name):
    return set(t for t in re.split(r"[/\s,\-]+", name.strip()) if t)


def recip(rw):
    m = re.match(r"(\d+)([LRCW]?)$", rw)
    if not m:
        return rw
    n = int(m.group(1))
    r = (n + 18 - 1) % 36 + 1 if n <= 36 else n
    s = {"L": "R", "R": "L", "C": "C"}.get(m.group(2), m.group(2))
    return "%02d%s" % (r, s)


# ---------------------------------------------------------------------------
# routing
# ---------------------------------------------------------------------------
MAX_TURN = 120.0       # 禁止大於此角度的轉彎（接近掉頭）
SHARP_TURN = 90.0
SHARP_PEN = 100.0
CROSS_PEN = 300.0
BACKTRACK_MULT = 1.5


class RunwayEnd:
    pass


def runway_ends(ap, g):
    res = []
    for r in ap.runways:
        for i in (0, 1):
            re_ = RunwayEnd()
            re_.name = r["ends"][i][0]
            re_.thr = r["p"][i]
            re_.u = r["u"] if i == 0 else neg(r["u"])
            re_.len = r["len"]
            re_.width = r["width"]
            re_.pair = {r["ends"][0][0], r["ends"][1][0]}
            re_.hdg = dir2hdg(re_.u)
            rw_edges = [e for e in g.edges if e["kind"] == "runway" and rwy_tokens(e["name"]) & re_.pair]
            re_.rw_edges = set(e["id"] for e in rw_edges)
            nodes = set()
            for e in rw_edges:
                nodes.add(e["a"])
                nodes.add(e["b"])
            re_.target = None
            re_.target_along = None
            re_.end_along = None
            re_.entries = {}
            if nodes:
                along = lambda n: dot(sub(ap.nodes[n]["p"], re_.thr), re_.u)
                # 能「開進」跑道的入口：有一條滑行道有向邊以這個跑道節點為終點（尊重 oneway）
                entries = [n for n in nodes if any(g.edges[g.de[d][0]]["kind"] == "taxiway" for d in g.inc[n])]
                end_node = min(nodes, key=along)
                re_.end_along = along(end_node)
                entries = [n for n in entries if along(n) <= re_.len / 2]
                if entries:
                    best = min(entries, key=along)
                    re_.target = best
                    re_.target_along = along(best)
                    re_.entries = {n: along(n) - re_.target_along for n in entries}
            res.append(re_)
    return res


def edge_zone_rwys(e):
    s = set()
    for kind, rws in e["zones"]:
        s.update(rws)
    return s


def route_table(g, rend, ac_cls):
    """togo[de] = 從 de 的終點（以 de 的方向抵達）走到目標節點的代價。"""
    INF = float("inf")
    togo = [INF] * len(g.de)
    nxt = [None] * len(g.de)
    if rend.target is None:
        return togo, nxt
    pq = []
    for n, lost in rend.entries.items():
        for did in g.inc[n]:
            e = g.edges[g.de[did][0]]
            if e["kind"] != "taxiway" or not cls_ok(e["cls"], ac_cls):
                continue
            c0 = W_ALONG * lost
            if c0 < togo[did]:
                togo[did] = c0
                heapq.heappush(pq, (c0, did))

    def cost_of(did):
        eid, f, t, h, L = g.de[did]
        e = g.edges[eid]
        c = L
        if e["kind"] == "runway":
            if eid in rend.rw_edges:
                c = L * BACKTRACK_MULT
            else:
                c = L + CROSS_PEN
        else:
            zr = edge_zone_rwys(e)
            if zr and not (zr & rend.pair):
                c = L + CROSS_PEN / 3  # 穿越其他跑道的保護區（含 crossing 前後）
        return c

    while pq:
        c, d2 = heapq.heappop(pq)
        if c > togo[d2]:
            continue
        eid2, f2, t2, h2, L2 = g.de[d2]
        c2 = cost_of(d2)
        for d1 in g.inc[f2]:
            eid1, f1, t1, h1, L1 = g.de[d1]
            if eid1 == eid2:
                continue  # 原地掉頭
            e1 = g.edges[eid1]
            if not cls_ok(e1["cls"], ac_cls):
                continue
            if eid1 in rend.rw_edges:
                continue  # 不在目標跑道上滑行（回滑由對正時決定）
            ang = abs(rel_hdg(h1, h2))
            if ang > MAX_TURN:
                continue
            cand = c + c2 + (SHARP_PEN if ang > SHARP_TURN else 0.0)
            if cand < togo[d1]:
                togo[d1] = cand
                nxt[d1] = d2
                heapq.heappush(pq, (cand, d1))
    return togo, nxt


# ---------------------------------------------------------------------------
# pushback candidates
# ---------------------------------------------------------------------------
R_SEARCH = 300.0
STEP = 2.0
W_PB = 3.0
W_TIGHT = 3.0
OBL_PEN = 50.0
ZONE_PEN = 200.0
MAX_ROT = 150.0
PB_MAX = 350.0
NOSE_TOL = 1.5          # 鼻輪離滑行道中線的容許值
TIE_FRONT = 80.0        # tie_down 前方 80 m 內有滑行道才算直接滑出
W_ALONG = 2.0           # 每損失 1 m 跑道長度，算 2 m 滑行代價
STAND_PEN = 2000.0      # 後推主輪路徑穿過其他停機位的停機範圍
FUSE_LEN = {"A": 9.0, "B": 14.0, "C": 38.0, "D": 55.0, "E": 64.0, "F": 73.0}
FOOT_HALF = 6.0
TWX_PEN = 5000.0        # 後推路徑橫越其他滑行道中線


def in_runway(g_ap, p, margin=0.0):
    for r in g_ap.runways:
        A = r["p"][0]
        rel = sub(p, A)
        al = dot(rel, r["u"])
        lat = abs(rel[0] * r["u"][1] - rel[1] * r["u"][0])
        if -margin <= al <= r["len"] + margin and lat <= r["width"] / 2 + margin:
            return True
    return False


def stand_candidates(ap, g, stand, ac, stand_ref):
    H = stand["hdg"]
    fwd = hdg2dir(H)
    wb = ac["wheelbase"]
    Pn = stand["p"]
    Pm = sub(Pn, scmul(fwd, wb)) if stand_ref == "nose" else Pn
    veh = Veh(wb, ac["max_steer"])
    rmin = veh.min_radius()
    r_pref = max(wb, rmin * 1.15)
    diag = collections.Counter()
    foots = []
    for (s2, fa, fb) in g.foot:
        if s2 is stand or dist(s2["p"], Pn) < 10.0:
            continue
        if seg_dist(Pm, fa, fb)[0] < FOOT_HALF or seg_dist(Pn, fa, fb)[0] < FOOT_HALF:
            continue  # 重疊的 MARS 機位
        if dist(fa, Pm) > R_SEARCH + 100:
            continue
        foots.append((s2, fa, fb))
    near_edges = [e for e in g.near(Pm, R_SEARCH + 50) if e["kind"] == "taxiway" and e["id"] not in g.leadin]
    # 第一輪：便宜的幾何條件，每個 (邊, 方向) 收集候選
    pool = collections.defaultdict(list)
    for e in g.near(Pm, R_SEARCH):
        if e["kind"] != "taxiway":
            continue
        if e["id"] in g.leadin:
            diag["leadin"] += 1
            continue
        if not cls_ok(e["cls"], ac["route_cls"]):
            diag["narrow"] += 1
            continue
        A, B, L = e["A"], e["B"], e["L"]
        n = max(1, int(L / STEP))
        zone = bool(e["zones"])
        for k in range(n + 1):
            t = k / n
            Ept = add(A, scmul(sub(B, A), t))
            rel = sub(Ept, Pm)
            if dot(rel, fwd) > -2.0:
                diag["not_behind"] += 1
                continue
            if vabs(rel) > R_SEARCH:
                continue
            for sgn in (+1, -1):
                if e["oneway"] and sgn == -1:
                    continue
                eh = e["hdg"] if sgn == 1 else normalize_hdg(e["hdg"] + 180)
                nseg, segs = compute_segs(veh, Pm, H, Ept, eh)
                diag["tried"] += 1
                if nseg <= 0:
                    diag["segs_fail"] += 1
                    continue
                if not all(s["backward"] for s in segs):
                    diag["has_forward"] += 1
                    continue
                pb_len = sum(seg_len(s) for s in segs)
                if pb_len > PB_MAX:
                    diag["too_long"] += 1
                    continue
                rot = sum(abs(rel_hdg(s["sh"], s["eh"])) for s in segs if s["type"] == "T")
                if rot > MAX_ROT:
                    diag["rot"] += 1
                    continue
                endp = segs[-1]["ep"]
                if dist(endp, Ept) > 0.5:
                    diag["off_line"] += 1
                    continue
                nosep = add(endp, scmul(hdg2dir(eh), wb))
                dn, en_, _ = g.nearest_taxi(nosep, R=30.0)
                if dn > NOSE_TOL:
                    diag["nose_off"] += 1
                    continue
                turns = [s["r"] for s in segs if s["type"] == "T"]
                r_used = min(turns) if turns else float("inf")
                to_node = (1 - t) * L if sgn == 1 else t * L
                base = (W_PB * pb_len + W_TIGHT * max(0.0, r_pref - r_used) +
                        (OBL_PEN if nseg > 2 else 0.0) + (ZONE_PEN if zone else 0.0))
                pool[(e["id"], sgn)].append((base + to_node, {
                    "eid": e["id"], "sgn": sgn, "t": t, "E": Ept, "eh": eh, "segs": segs,
                    "pb_len": pb_len, "rot": rot, "r_used": r_used, "nseg": nseg,
                    "to_node": to_node, "base": base, "zone": zone}))
    # 第二輪：分支定界，只對可能勝出的候選做昂貴檢查（罰分只會加不會減）
    best = {}
    for key, lst in pool.items():
        lst.sort(key=lambda x: x[0])
        cur = None
        for sc0, c in lst:
            if cur is not None and sc0 >= cur["score"]:
                break
            pts = []
            for s in c["segs"]:
                sp = seg_points(s)
                pts.extend(sp if not pts else sp[1:])
            if any(in_runway(ap, q) for q in pts[::2]):
                diag["runway"] += 1
                continue
            bx0 = min(q[0] for q in pts) - FOOT_HALF
            bx1 = max(q[0] for q in pts) + FOOT_HALF
            by0 = min(q[1] for q in pts) - FOOT_HALF
            by1 = max(q[1] for q in pts) + FOOT_HALF
            conflicts = []
            for (s2, fa, fb) in foots:
                if (max(fa[0], fb[0]) < bx0 or min(fa[0], fb[0]) > bx1 or
                        max(fa[1], fb[1]) < by0 or min(fa[1], fb[1]) > by1):
                    continue
                for q in pts[::2]:
                    if seg_dist(q, fa, fb)[0] < FOOT_HALF:
                        conflicts.append(s2["name"])
                        break
            # 後推路徑橫越其他滑行道中線（終點附近 8 m 內的交點不算）
            xings = []
            endp = c["segs"][-1]["ep"]
            for e2 in near_edges:
                if e2["id"] == c["eid"]:
                    continue
                if (max(e2["A"][0], e2["B"][0]) < bx0 or min(e2["A"][0], e2["B"][0]) > bx1 or
                        max(e2["A"][1], e2["B"][1]) < by0 or min(e2["A"][1], e2["B"][1]) > by1):
                    continue
                for i in range(len(pts) - 1):
                    xp = seg_isect(pts[i], pts[i + 1], e2["A"], e2["B"])
                    if xp is not None and dist(xp, endp) > 8.0:
                        xings.append(e2["name"] or "(無名)")
                        break
            score = sc0 + STAND_PEN * len(conflicts) + TWX_PEN * len(xings)
            if cur is None or score < cur["score"]:
                cur = dict(c)
                cur.update({"pts": pts, "conflicts": conflicts, "xings": xings, "score": score,
                            "pb_score": c["base"] + STAND_PEN * len(conflicts) + TWX_PEN * len(xings)})
        if cur is not None:
            best[key] = cur
    return Pm, best, diag, rmin, r_pref


# ---------------------------------------------------------------------------
# evaluation
# ---------------------------------------------------------------------------
def classify_stand(g, stand, ac, stand_ref, R=300.0):
    """前後兩側各找最近的滑行道點：前方比後方近 → 機頭朝滑行道，不需要後推。"""
    fwd = hdg2dir(stand["hdg"])
    P0 = stand["p"]
    d_front = d_back = float("inf")
    for e in g.near(P0, R):
        if e["kind"] != "taxiway" or e["id"] in g.leadin:
            continue
        n = max(1, int(e["L"] / 2.0))
        for k in range(n + 1):
            q = add(e["A"], scmul(sub(e["B"], e["A"]), k / n))
            rel = sub(q, P0)
            d = vabs(rel)
            if d < 1e-6 or d > R:
                continue
            c = dot(rel, fwd) / d
            if c > 0.5:            # 前方 ±60°
                d_front = min(d_front, d)
            elif c < -0.5:         # 後方 ±60°
                d_back = min(d_back, d)
    if d_front == float("inf") and d_back == float("inf"):
        return "no_taxiway_300m", d_back, d_front
    geom = "faces_taxiway" if d_front < d_back else "pushback"
    t = stand["type"]
    if t == "gate":
        kind = "pushback" if d_back < float("inf") else "faces_taxiway"
    elif t == "tie_down":
        kind = "faces_taxiway" if (d_front <= TIE_FRONT and d_front <= d_back) or d_back == float("inf") else "pushback"
    else:
        kind = geom
    stand["_geom_kind"] = geom
    return kind, d_back, d_front


def walk_route(g, did, nxt, rend):
    seq = [did]
    seen = set([did])
    while nxt[seq[-1]] is not None:
        d = nxt[seq[-1]]
        if d in seen:
            break
        seen.add(d)
        seq.append(d)
    return seq


def describe_route(g, ap, start_did, seq, rend, ac_wb):
    names = []
    turns = []
    crossings = 0
    backtrack = 0.0
    hs_node = None
    prev = None
    in_cross = False
    for i, d in enumerate(seq):
        eid, f, t, h, L = g.de[d]
        e = g.edges[eid]
        if prev is not None:
            turns.append(abs(rel_hdg(g.de[prev][3], h)))
        prev = d
        nm = e["name"] if e["kind"] == "taxiway" else ("RWY " + e["name"])
        if nm and (not names or names[-1] != nm):
            names.append(nm)
        zr = edge_zone_rwys(e)
        is_target = (e["kind"] == "runway" and eid in rend.rw_edges) or (zr & rend.pair and e["kind"] == "taxiway" and any(k == "departure" or k == "ils" for k, rws in e["zones"] if set(rws) & rend.pair))
        if hs_node is None and is_target and i > 0:
            hs_node = f
        if e["kind"] == "runway" and eid in rend.rw_edges:
            backtrack += L
        other = (e["kind"] == "runway" and eid not in rend.rw_edges) or (zr and not (zr & rend.pair))
        if other and not in_cross:
            crossings += 1
        in_cross = bool(other)
    if hs_node is None and seq:
        hs_node = g.de[seq[-1]][1]
        hs_from_zone = False
    else:
        hs_from_zone = True
    entry = g.de[seq[-1]][2] if seq else None
    return names, turns, crossings, backtrack, hs_node, entry, hs_from_zone


def evaluate_airport(ap, fleet, stand_ref, log=print):
    g = Graph(ap)
    rends = runway_ends(ap, g)
    tables = {}
    results = []
    stats = collections.Counter()
    for si, st in enumerate(ap.stands):
        acts = st["actypes"]
        if acts and set(acts.split("|")) <= {"helos"}:
            results.append({"stand": si, "status": "helo"})
            stats["helo"] += 1
            continue
        cls = st["cls"]
        if cls is None:
            cls = "E" if "heavy" in acts else ("C" if "jets" in acts else ("B" if "turboprops" in acts else "A"))
        if fleet == "b738" and CLS_ORDER.index(cls) < CLS_ORDER.index("C"):
            results.append({"stand": si, "status": "too_small_for_fleet"})
            stats["too_small_for_fleet"] += 1
            continue
        ac = aircraft(cls, fleet)
        kind, dback, dfront = classify_stand(g, st, ac, stand_ref)
        rec = {"stand": si, "cls": cls, "ac": ac["name"], "kind": kind, "dnear": dback, "dfront": dfront}
        Pm, cands, diag, rmin, r_pref = stand_candidates(ap, g, st, ac, stand_ref)
        rec.update({"Pm": Pm, "diag": dict(diag), "rmin": rmin, "r_pref": r_pref,
                    "ncand": len(cands)})
        per = {}
        for rend in rends:
            key = (rend.name, ac["route_cls"])
            if key not in tables:
                tables[key] = route_table(g, rend, ac["route_cls"])
            togo, nxt = tables[key]
            best = None
            for (eid, sgn), c in cands.items():
                did = g.de_index.get((eid, sgn))
                if did is None:
                    continue
                if togo[did] == float("inf"):
                    continue
                total = c["score"] + togo[did]
                if best is None or total < best[0]:
                    best = (total, c, did)
            if best is None:
                if rend.target is None:
                    reason = "no_runway_edges"
                elif not cands:
                    reason = "no_candidate"
                else:
                    reason = "no_route"
                per[rend.name] = {"ok": False, "reason": reason}
                continue
            total, c, did = best
            seq = walk_route(g, did, nxt, rend)
            names, turns, crossings, backtrack, hs_node, entry, hs_zone = describe_route(g, ap, did, seq, rend, ac["wheelbase"])
            e = g.edges[c["eid"]]
            endp, endh = c["segs"][-1]["ep"], c["segs"][-1]["eh"]
            # 終點離中線距離、與滑行道邊夾角（主輪與鼻輪）
            dmain, emain, _ = g.nearest_taxi(endp, R=60.0)
            ang = None
            for ee in g.near(endp, 1.0):
                if ee["kind"] != "taxiway":
                    continue
                a_ = abs(rel_hdg(endh, ee["hdg"]))
                a_ = min(a_, 180 - a_)
                ang = a_ if ang is None else min(ang, a_)
            nosep = add(endp, scmul(hdg2dir(endh), ac["wheelbase"]))
            dnose, _, _ = g.nearest_taxi(nosep, R=60.0)
            taxi_dist = c["to_node"] + sum(g.de[d][4] for d in seq[1:]) if seq else c["to_node"]
            # 路線 polyline：E → 第一條邊終點 → ...
            route_pts = [endp] + [g.ap.nodes[g.de[d][2]]["p"] for d in seq]
            per[rend.name] = {
                "ok": True, "edge": e["name"], "eid": c["eid"], "end": endp, "end_hdg": endh,
                "pb_len": c["pb_len"], "rot": c["rot"], "r_used": c["r_used"], "nseg": c["nseg"],
                "steer": math.degrees(math.atan(ac["wheelbase"] / c["r_used"])) if c["r_used"] != float("inf") else 0.0,
                "off_main": dmain, "off_nose": dnose, "ang_edge": ang,
                "first_turn": turns[0] if turns else 0.0, "max_turn": max(turns) if turns else 0.0,
                "n_sharp": sum(1 for a in turns if a > SHARP_TURN),
                "crossings": crossings, "backtrack": backtrack, "taxi": taxi_dist,
                "names": names, "hs": g.ap.nodes[hs_node]["p"] if hs_node is not None else None,
                "zone_end": c["zone"], "pts": c["pts"], "route": route_pts, "sgn": c["sgn"],
                "entry_lost": rend.entries.get(entry, 0.0) + (rend.target_along or 0.0), "hs_zone": hs_zone,
                "conflicts": c["conflicts"], "xings": c["xings"],
            }
        rec["per"] = per
        results.append(rec)
    return g, rends, results


# ---------------------------------------------------------------------------
# statistics
# ---------------------------------------------------------------------------
def pct(xs, q):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    k = (len(xs) - 1) * q
    f = int(math.floor(k))
    c = min(f + 1, len(xs) - 1)
    return xs[f] + (xs[c] - xs[f]) * (k - f)


def dstr(xs, fmt="%.1f"):
    if not xs:
        return "n/a"
    return ("中位 " + fmt + " / p95 " + fmt + " / 最大 " + fmt) % (pct(xs, .5), pct(xs, .95), max(xs))


def tier(v):
    """標準：直退＋一個彎、轉 ≤120°、≤200 m、不壓停機位、不橫越滑行道。"""
    if v["conflicts"] or v["xings"] or v["pb_len"] > 250:
        return "suspect"
    if v["nseg"] <= 2 and v["rot"] <= 120 and v["pb_len"] <= 200:
        return "standard"
    return "unusual"


def summarize(ap, rends, results):
    S = {}
    kinds = collections.Counter(r.get("kind", r["status"] if "status" in r else "?") for r in results)
    S["stands_total"] = len(ap.stands)
    S["kinds"] = dict(kinds)
    push = [r for r in results if r.get("kind") == "pushback"]
    S["pushback_stands"] = len(push)
    pairs = [(r, rn, v) for r in push for rn, v in r["per"].items()]
    ok = [(r, rn, v) for r, rn, v in pairs if v["ok"]]
    S["pairs"] = len(pairs)
    S["pairs_ok"] = len(ok)
    S["stands_all_ok"] = sum(1 for r in push if all(v["ok"] for v in r["per"].values()))
    S["stands_none_ok"] = sum(1 for r in push if not any(v["ok"] for v in r["per"].values()))
    S["fail_reasons"] = dict(collections.Counter(v["reason"] for r, rn, v in pairs if not v["ok"]))
    S["off_main"] = dstr([v["off_main"] for _, _, v in ok], "%.2f")
    S["off_nose"] = dstr([v["off_nose"] for _, _, v in ok], "%.2f")
    S["ang_edge"] = dstr([v["ang_edge"] for _, _, v in ok if v["ang_edge"] is not None], "%.2f")
    S["pb_len"] = dstr([v["pb_len"] for _, _, v in ok])
    S["steer"] = dstr([v["steer"] for _, _, v in ok])
    S["rot"] = dstr([v["rot"] for _, _, v in ok])
    S["taxi"] = dstr([v["taxi"] for _, _, v in ok], "%.0f")
    S["first_turn_gt90"] = sum(1 for _, _, v in ok if v["first_turn"] > 90)
    S["first_turn_gt100"] = sum(1 for _, _, v in ok if v["first_turn"] > 100)
    S["any_turn_gt100"] = sum(1 for _, _, v in ok if v["max_turn"] > 100)
    S["type_geom_conflict"] = sum(1 for r in results if "kind" in r and ap.stands[r["stand"]].get("_geom_kind") not in (None, r["kind"]))
    S["any_sharp"] = sum(1 for _, _, v in ok if v["n_sharp"] > 0)
    S["oblique"] = sum(1 for _, _, v in ok if v["nseg"] > 2)
    S["zone_end"] = sum(1 for _, _, v in ok if v["zone_end"])
    S["crossing_pairs"] = sum(1 for _, _, v in ok if v["crossings"] > 0)
    S["backtrack_pairs"] = sum(1 for _, _, v in ok if v["backtrack"] > 1)
    S["no_hs"] = sum(1 for _, _, v in ok if v["hs"] is None)
    S["off_nose_gt3"] = sum(1 for _, _, v in ok if v["off_nose"] > 3)
    # 同一停機位，不同跑道端選了不同的推出方向
    dep = 0
    for r in push:
        dirs = set((v["eid"], v["sgn"]) for v in r["per"].values() if v["ok"])
        if len(dirs) > 1:
            dep += 1
    S["stands_dir_depends_on_rwy"] = dep
    S["dnear_pushback"] = dstr([r["dnear"] for r in push])
    S["runway_targets"] = {re_.name: {"entry_along": (round(re_.target_along, 1) if re_.target_along is not None else None),
                                      "end_along": (round(re_.end_along, 1) if re_.end_along is not None else None)} for re_ in rends}
    S["backtrack_gt150"] = sum(1 for _, _, v in ok if v["backtrack"] > 150)
    S["backtrack"] = dstr([v["backtrack"] for _, _, v in ok if v["backtrack"] > 1], "%.0f")
    S["dfront_faces"] = dstr([r["dfront"] for r in results if r.get("kind") == "faces_taxiway"])
    S["kind_by_type"] = dict(collections.Counter("%s/%s" % (ap.stands[r["stand"]]["type"], r.get("kind")) for r in results if "kind" in r))
    S["rot_gt120"] = sum(1 for _, _, v in ok if v["rot"] > 120)
    S["tier"] = dict(collections.Counter(tier(v) for _, _, v in ok))
    S["stand_tier"] = dict(collections.Counter(
        min((tier(v) for v in r["per"].values() if v["ok"]), key=["standard", "unusual", "suspect"].index, default="none")
        for r in push))
    S["stand_conflict_pairs"] = sum(1 for _, _, v in ok if v["conflicts"])
    S["tw_cross_pairs"] = sum(1 for _, _, v in ok if v["xings"])
    S["tw_cross_stands"] = sum(1 for r in push if any(v["ok"] and v["xings"] for v in r["per"].values()))
    S["stand_conflict_stands"] = sum(1 for r in push if any(v["ok"] and v["conflicts"] for v in r["per"].values()))
    S["pb_gt200"] = sum(1 for _, _, v in ok if v["pb_len"] > 200)
    S["pb_gt250"] = sum(1 for _, _, v in ok if v["pb_len"] > 250)
    S["entry_lost"] = dstr([v["entry_lost"] for _, _, v in ok], "%.0f")
    S["entry_lost_gt150"] = sum(1 for _, _, v in ok if v["entry_lost"] > 150)
    S["hs_no_zone"] = sum(1 for _, _, v in ok if not v["hs_zone"])
    S["fail_by_rwy"] = dict(collections.Counter("%s:%s" % (rn, v["reason"]) for r, rn, v in pairs if not v["ok"]))
    return S


# ---------------------------------------------------------------------------
# HTML
# ---------------------------------------------------------------------------
def r1(p):
    return [round(p[0], 1), round(p[1], 1)]


def export_html(ap, g, rends, results, S, path, title):
    data = {
        "title": title,
        "runways": [[r1(add(r["p"][0], scmul(vnorm(r["u"], True), r["width"] / 2))),
                     r1(add(r["p"][1], scmul(vnorm(r["u"], True), r["width"] / 2))),
                     r1(add(r["p"][1], scmul(vnorm(r["u"], False), r["width"] / 2))),
                     r1(add(r["p"][0], scmul(vnorm(r["u"], False), r["width"] / 2)))] for r in ap.runways],
        "rlabels": [[r["ends"][i][0], r1(r["p"][i])] for r in ap.runways for i in (0, 1)],
        "edges": [[r1(e["A"]), r1(e["B"]), e["kind"][0], e["cls"] or "", e["name"],
                   1 if e["oneway"] else 0, 1 if e["id"] in g.leadin else 0, 1 if e["zones"] else 0] for e in g.edges],
        "rends": [re_.name for re_ in rends],
        "stands": [],
        "stats": S,
    }
    for r in results:
        st = ap.stands[r["stand"]]
        item = {"n": st["name"], "p": r1(st["p"]), "h": round(st["hdg"], 1), "t": st["type"],
                "c": r.get("cls"), "k": r.get("kind", r.get("status")), "ac": r.get("ac"),
                "per": {}}
        if "per" in r:
            item["diag"] = r["diag"]
            item["rmin"] = round(r["rmin"], 1)
            for rn, v in r["per"].items():
                if not v["ok"]:
                    item["per"][rn] = {"ok": 0, "why": v["reason"]}
                    continue
                item["per"][rn] = {
                    "ok": 1, "e": v["edge"], "end": r1(v["end"]), "eh": round(v["end_hdg"]),
                    "pb": round(v["pb_len"]), "st": round(v["steer"]), "rot": round(v["rot"]),
                    "taxi": round(v["taxi"]), "ft": round(v["first_turn"]), "x": v["crossings"],
                    "bt": round(v["backtrack"]), "nm": v["names"], "hs": r1(v["hs"]) if v["hs"] else None,
                    "on": round(v["off_nose"], 1), "pts": [r1(q) for q in v["pts"][::2]] + [r1(v["pts"][-1])],
                    "rt": [r1(q) for q in v["route"]], "z": 1 if v["zone_end"] else 0,
                    "cf": v["conflicts"], "lost": round(v["entry_lost"]), "xw": v["xings"], "tier": tier(v),
                }
        data["stands"].append(item)
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "viewer.html"), encoding="utf-8") as f:
        tpl = f.read()
    html = tpl.replace("/*__DATA__*/null", json.dumps(data, ensure_ascii=False, separators=(",", ":")))
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)


def main():
    ap_ = argparse.ArgumentParser()
    ap_.add_argument("--apt", required=True)
    ap_.add_argument("--label", required=True)
    ap_.add_argument("--out", required=True)
    ap_.add_argument("--fleet", default="class", choices=["class", "b738"])
    ap_.add_argument("--stand-ref", default="nose", choices=["nose", "main"])
    ap_.add_argument("--no-html", action="store_true")
    a = ap_.parse_args()
    ap = parse_apt(a.apt)
    project(ap)
    g, rends, results = evaluate_airport(ap, a.fleet, a.stand_ref)
    S = summarize(ap, rends, results)
    S["label"] = a.label
    S["fleet"] = a.fleet
    S["stand_ref"] = a.stand_ref
    S["graph"] = {"nodes": len(ap.nodes), "taxi_edges": sum(1 for e in g.edges if e["kind"] == "taxiway"),
                  "runway_edges": sum(1 for e in g.edges if e["kind"] == "runway"),
                  "leadin_edges": len(g.leadin)}
    os.makedirs(a.out, exist_ok=True)
    tag = "%s_%s_%s" % (a.label, a.fleet, a.stand_ref)
    with open(os.path.join(a.out, tag + ".json"), "w", encoding="utf-8") as f:
        json.dump(S, f, ensure_ascii=False, indent=1)
    if not a.no_html:
        export_html(ap, g, rends, results, S, os.path.join(a.out, tag + ".html"), "%s (%s, %s)" % (a.label, a.fleet, a.stand_ref))
    print(json.dumps(S, ensure_ascii=False))


if __name__ == "__main__":
    main()
