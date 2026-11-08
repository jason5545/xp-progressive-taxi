#!/bin/bash
#
# 建離線規劃 CLI（tools/plan_cli/plan_cli）。跟插件編同一份
# src/segs.c 與 src/planner/*.c，連結 tools/build-mac.sh 建好的
# libacfutils.a。-ffp-contract=off 讓浮點結果跟 Python 原型一致。
#
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
L="$REPO/src/libacfutils"
LIB="$L/qmake/mac64/libacfutils.a"
[[ -f "$LIB" ]] || { echo "找不到 ${LIB}，先跑 tools/build-mac.sh" >&2; exit 1; }

clang -std=c99 -Wall -Wextra -Werror -O2 -ffp-contract=off \
    -D_GNU_SOURCE -DAPL=1 -DIBM=0 -DLIN=0 \
    -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DXPLM302=1 \
    -I "$REPO/src" -I "$L/src" -I "$L/SDK/CHeaders/XPLM" \
    -o "$REPO/tools/plan_cli/plan_cli" \
    "$REPO/tools/plan_cli/plan_cli.c" "$REPO/src/segs.c" \
    "$REPO/src/planner/ptx_apt.c" "$REPO/src/planner/ptx_plan.c" \
    "$LIB" 2> >(grep -v '^ld: warning' >&2)
echo "建好 tools/plan_cli/plan_cli"
