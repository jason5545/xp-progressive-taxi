#!/usr/bin/env python3
"""把 pbplan 的結果畫成靜態 SVG（給自己目視檢查，也可給人看）。"""
import sys, math
from xml.sax.saxutils import escape
sys.path.insert(0, __import__("os").path.dirname(__file__))
import pbplan as P


def render(ap, g, rends, res, rname, path, bbox=None, title="", stands_filter=None, show_routes=None):
    xs = [p for e in g.edges for p in (e["A"][0], e["B"][0])] + [s["p"][0] for s in ap.stands]
    ys = [p for e in g.edges for p in (e["A"][1], e["B"][1])] + [s["p"][1] for s in ap.stands]
    if bbox:
        x0, y0, x1, y1 = bbox
    else:
        x0, x1, y0, y1 = min(xs) - 50, max(xs) + 50, min(ys) - 50, max(ys) + 50
    W = 1800
    sc = W / (x1 - x0)
    H = int((y1 - y0) * sc)
    T = lambda p: "%.1f,%.1f" % ((p[0] - x0) * sc, (y1 - p[1]) * sc)
    o = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" style="background:#fbfaf7">' % (W, H + 40, W, H + 40)]
    o.append('<text x="10" y="%d" font-size="22" font-family="Helvetica">%s</text>' % (H + 30, escape(title)))
    for r in ap.runways:
        n = P.vnorm(r["u"], True)
        pts = [P.add(r["p"][0], P.scmul(n, r["width"] / 2)), P.add(r["p"][1], P.scmul(n, r["width"] / 2)),
               P.add(r["p"][1], P.scmul(n, -r["width"] / 2)), P.add(r["p"][0], P.scmul(n, -r["width"] / 2))]
        o.append('<polygon points="%s" fill="#555" opacity="0.5"/>' % " ".join(T(p) for p in pts))
        for i in (0, 1):
            q = T(r["p"][i]).split(",")
            o.append('<text x="%s" y="%s" font-size="20" fill="#000">%s</text>' % (q[0], q[1], r["ends"][i][0]))
    for e in g.edges:
        col = "#333" if e["kind"] == "runway" else ("#bbb" if e["id"] in g.leadin else "#888")
        dash = ' stroke-dasharray="6,4"' if e["oneway"] else ""
        o.append('<line x1="%s" y1="%s" x2="%s" y2="%s" stroke="%s" stroke-width="%.1f"%s/>' % (
            *T(e["A"]).split(","), *T(e["B"]).split(","), col, max(1.0, 2.0), dash))
        if e["name"] and e["L"] * sc > 40 and e["kind"] == "taxiway":
            m = T(P.mean(e["A"], e["B"])).split(",")
            o.append('<text x="%s" y="%s" font-size="11" fill="#2a5">%s</text>' % (m[0], m[1], escape(e["name"])))
    for r in res:
        st = ap.stands[r["stand"]]
        if stands_filter and st["name"] not in stands_filter:
            continue
        kind = r.get("kind", r.get("status"))
        v = r.get("per", {}).get(rname)
        col = {"pushback": "#1f8a4c" if (v and v["ok"]) else "#c62828", "faces_taxiway": "#2f6fc0"}.get(kind, "#999")
        if v and v["ok"]:
            o.append('<polyline points="%s" fill="none" stroke="#e07a00" stroke-width="2"/>' % " ".join(T(p) for p in v["pts"]))
            h = v["end_hdg"]
            tip = P.add(v["end"], P.scmul(P.hdg2dir(h), 8))
            o.append('<line x1="%s" y1="%s" x2="%s" y2="%s" stroke="#7b3fb5" stroke-width="3"/>' % (*T(v["end"]).split(","), *T(tip).split(",")))
            if show_routes and st["name"] in show_routes:
                o.append('<polyline points="%s" fill="none" stroke="#00a3b5" stroke-width="3" opacity="0.8"/>' % " ".join(T(p) for p in v["route"]))
        tip = P.add(st["p"], P.scmul(P.hdg2dir(st["hdg"]), 6))
        o.append('<circle cx="%s" cy="%s" r="3.5" fill="%s"/>' % (*T(st["p"]).split(","), col))
        o.append('<line x1="%s" y1="%s" x2="%s" y2="%s" stroke="%s" stroke-width="2"/>' % (*T(st["p"]).split(","), *T(tip).split(","), col))
        q = T(st["p"]).split(",")
        o.append('<text x="%s" y="%s" font-size="10" fill="%s">%s</text>' % (float(q[0]) + 4, q[1], col, escape(st["name"])))
    o.append("</svg>")
    open(path, "w").write("\n".join(o))
