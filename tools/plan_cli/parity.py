#!/usr/bin/env python3
"""
C 規劃核心（tools/plan_cli/plan_cli）跟 Python 原型（tools/proto/pbplan.py）
逐筆對拍，並檢查第 2 段驗收門檻 1–4：

  1. 構造不變量（100% 可行組）：主輪離中線 <0.1 m、終點朝向與邊夾角 <0.5°、
     鼻輪 ≤1.5 m、路線沒有 >120° 的轉彎、後推 >300 m 的一律標成不自動推
  2. 覆蓋率不退步：至少一條跑道可行的停機位數 ≥ 原型
  3. 同一停機位×跑道端選到同一條邊同一方向 ≥99%
  4. 每座機場「標準」比例 ≥ 原型減 2 個百分點

用法：
  parity.py --fleet fleet.txt --cli tools/plan_cli/plan_cli \\
            --work DIR AIRPORT=APT_FILE [AIRPORT=APT_FILE ...]
每座機場會跑 pbplan.py --pairs-out 與 plan_cli all，結果放在 DIR。
結束碼 0＝全部門檻通過。
"""
import argparse
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROTO = os.path.join(HERE, "..", "proto", "pbplan.py")


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit("命令失敗：%s" % " ".join(cmd))
    return r


def key(rec):
    return (rec["si"], rec.get("rwy"))


def compare(label, py, c):
    P = {key(r): r for r in py}
    C = {key(r): r for r in c}
    out = {"label": label}
    out["missing_in_c"] = sorted(set(P) - set(C))[:5]
    out["missing_in_py"] = sorted(set(C) - set(P))[:5]
    pairs = [k for k in P if k in C and k[1] is not None]
    kind_mis = sorted({k[0] for k in pairs if P[k]["kind"] != C[k]["kind"]})
    ok_mis, reason_mis, same, diff_choice = [], [], 0, []
    maxd = {"xy": 0.0, "hdg": 0.0, "pb": 0.0, "taxi": 0.0, "lost": 0.0}
    names_mis = tier_mis = cf_mis = 0
    both_ok = 0
    for k in pairs:
        p, q = P[k], C[k]
        if p["ok"] != q["ok"]:
            ok_mis.append((p["stand"], k[1], p.get("reason"), q.get("reason")))
            continue
        if not p["ok"]:
            if p["reason"] != q["reason"]:
                reason_mis.append((p["stand"], k[1], p["reason"], q["reason"]))
            else:
                same += 1
            continue
        both_ok += 1
        if (p["a"], p["b"], p["sgn"]) == (q["a"], q["b"], q["sgn"]):
            same += 1
            maxd["xy"] = max(maxd["xy"], math.hypot(p["x"] - q["x"], p["y"] - q["y"]))
            dh = abs(p["hdg"] - q["hdg"]) % 360
            maxd["hdg"] = max(maxd["hdg"], min(dh, 360 - dh))
            maxd["pb"] = max(maxd["pb"], abs(p["pb"] - q["pb"]))
            maxd["taxi"] = max(maxd["taxi"], abs(p["taxi"] - q["taxi"]))
            maxd["lost"] = max(maxd["lost"], abs(p["lost"] - q["lost"]))
            names_mis += (p["names"] != q["names"])
            tier_mis += (p["tier"] != q["tier"])
            cf_mis += ((p["conf"], p["xing"]) != (q["conf"], q["xing"]))
        else:
            diff_choice.append((p["stand"], k[1], (p["a"], p["b"], p["sgn"]), (q["a"], q["b"], q["sgn"]),
                                round(p["pb"], 2), round(q["pb"], 2)))
    out.update({"pairs": len(pairs), "same": same, "kind_mismatch_stands": kind_mis[:5],
                "n_kind_mismatch": len(kind_mis), "ok_mismatch": ok_mis[:5], "n_ok_mismatch": len(ok_mis),
                "reason_mismatch": reason_mis[:5], "n_reason_mismatch": len(reason_mis),
                "diff_choice": diff_choice[:5], "n_diff_choice": len(diff_choice), "both_ok": both_ok,
                "maxdiff_same_choice": {k: float("%.3g" % v) for k, v in maxd.items()},
                "names_mismatch": names_mis, "tier_mismatch": tier_mis, "conf_xing_mismatch": cf_mis})
    out["parity_pct"] = 100.0 * same / max(1, len(pairs))
    return out


def pushback_view(recs):
    """只看需要後推的停機位（跟原型的統計一致）。"""
    push = [r for r in recs if r.get("kind") == "pushback" and "rwy" in r]
    stands = {}
    for r in push:
        stands.setdefault(r["si"], []).append(r)
    ok = [r for r in push if r["ok"]]
    cover = sum(1 for v in stands.values() if any(r["ok"] for r in v))
    std = sum(1 for r in ok if r["tier"] == "standard")
    return push, ok, cover, (100.0 * std / len(ok) if ok else 0.0)


def invariants(c_recs):
    bad = {"off_main": [], "ang_edge": [], "off_nose": [], "turn_gt120": [], "pb_gt300_allowed": []}
    for r in c_recs:
        if not r.get("ok"):
            continue
        tag = (r["stand"], r["rwy"])
        if r["om"] >= 0.1:
            bad["off_main"].append(tag)
        if r["ang"] is not None and r["ang"] >= 0.5:
            bad["ang_edge"].append(tag)
        if r["on"] > 1.5:
            bad["off_nose"].append(tag)
        if r["mt"] > 120:
            bad["turn_gt120"].append(tag)
        if r["pb"] > 300 and r["allowed"]:
            bad["pb_gt300_allowed"].append(tag)
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fleet", required=True)
    ap.add_argument("--cli", required=True)
    ap.add_argument("--work", required=True)
    ap.add_argument("--reuse-py", action="store_true", help="已有 Python 結果就不重跑")
    ap.add_argument("airports", nargs="+")
    a = ap.parse_args()
    os.makedirs(a.work, exist_ok=True)
    all_ok = True
    rows = []
    for spec in a.airports:
        label, path = spec.split("=", 1)
        pyj = os.path.join(a.work, label + ".py.json")
        cj = os.path.join(a.work, label + ".c.json")
        if not (a.reuse_py and os.path.exists(pyj)):
            run([sys.executable, "-I", PROTO, "--apt", path, "--label", label, "--out", a.work,
                 "--no-html", "--pairs-out", pyj])
        r = run([a.cli, "all", "--apt", path, "--fleet", a.fleet, "--out", cj, "--timing"])
        timing = r.stderr.strip()
        py = json.load(open(pyj))
        c = json.load(open(cj))
        cmp_ = compare(label, py, c)
        _, py_ok, py_cover, py_std = pushback_view(py)
        _, c_ok, c_cover, c_std = pushback_view(c)
        inv = invariants(c)
        t1 = all(len(v) == 0 for v in inv.values())
        t2 = c_cover >= py_cover
        t3 = cmp_["parity_pct"] >= 99.0
        t4 = c_std >= py_std - 2.0
        all_ok &= (t1 and t2 and t3 and t4)
        rows.append((label, cmp_, py_cover, c_cover, py_std, c_std, inv, (t1, t2, t3, t4), timing))

    for label, cmp_, py_cover, c_cover, py_std, c_std, inv, ts, timing in rows:
        print("=" * 72)
        print("%s  門檻1 %s  門檻2 %s  門檻3 %s  門檻4 %s" % ((label,) + tuple("通過" if t else "不通過" for t in ts)))
        print("  對拍：%d/%d 組一致（%.2f%%），雙方都可行 %d 組" % (cmp_["same"], cmp_["pairs"], cmp_["parity_pct"], cmp_["both_ok"]))
        print("  選擇不同 %d、可行性不同 %d、原因不同 %d、分類不同的停機位 %d" % (
            cmp_["n_diff_choice"], cmp_["n_ok_mismatch"], cmp_["n_reason_mismatch"], cmp_["n_kind_mismatch"]))
        print("  同選擇時最大差：%s；路線名稱不同 %d、分級不同 %d、壓停機位／橫越數不同 %d" % (
            cmp_["maxdiff_same_choice"], cmp_["names_mismatch"], cmp_["tier_mismatch"], cmp_["conf_xing_mismatch"]))
        for name in ("diff_choice", "ok_mismatch", "reason_mismatch", "missing_in_c", "missing_in_py"):
            if cmp_[name]:
                print("  %s 例：%s" % (name, cmp_[name]))
        print("  覆蓋（至少一條跑道可行的停機位）：原型 %d → C %d" % (py_cover, c_cover))
        print("  標準比例：原型 %.1f%% → C %.1f%%" % (py_std, c_std))
        print("  不變量違反：%s" % {k: len(v) for k, v in inv.items()})
        for k, v in inv.items():
            if v:
                print("    %s 例：%s" % (k, v[:5]))
        for line in timing.splitlines():
            print("  " + line)
    print("=" * 72)
    print("全部門檻通過" if all_ok else "有門檻沒過")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
