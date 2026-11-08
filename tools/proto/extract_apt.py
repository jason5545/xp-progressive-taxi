import sys, os
src, outdir, *icaos = sys.argv[1:]
want = set(icaos)
cur = None; buf = []; found = {}
def flush():
    if cur and cur in want:
        found[cur] = found.get(cur, 0) + 1
        with open(os.path.join(outdir, f"{cur}.dat"), "w", encoding="latin-1") as f:
            f.write("".join(buf))
with open(src, encoding="latin-1") as f:
    for line in f:
        p = line.split(None, 5)
        if p and p[0] in ("1", "16", "17", "99"):
            flush(); buf = []; cur = None
            if p[0] != "99" and len(p) >= 5:
                cur = p[4]
        if cur in want:
            buf.append(line)
flush()
print(found)
