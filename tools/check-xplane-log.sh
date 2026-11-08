#!/bin/bash
#
# 讀 X-Plane 的 Log.txt，判斷 ProgressiveTaxi 有沒有載入成功。
# 用法：tools/check-xplane-log.sh [Log.txt 路徑]
# 結束碼：0 載入成功且沒有插件錯誤；1 沒載入或有錯誤；2 找不到 Log.txt
#
set -uo pipefail

LOG="${1:-$HOME/X-Plane 12/Log.txt}"
SIG="jason5545.ProgressiveTaxi"

[[ -f "$LOG" ]] || { echo "找不到 ${LOG}"; exit 2; }

echo "Log：${LOG}"
echo "修改時間：$(stat -f '%Sm' -t '%Y-%m-%d %H:%M:%S' "$LOG")"
head -3 "$LOG" | sed 's/^/  /'
echo

loaded="$(grep -a "^Loaded: .*ProgressiveTaxi\.xpl (${SIG})" "$LOG" || true)"
banner="$(grep -a "This is ProgressiveTaxi-" "$LOG" || true)"
other_bp="$(grep -a "^Loaded: .*(skiselkov\.BetterPushback)" "$LOG" || true)"
# 插件自己的 log 行格式是「<時間> ProgressiveTaxi[檔案:行]: 訊息」（libacfutils
# log.c 的 PREFIX_FMT）；另外抓 X-Plane 對這個插件的抱怨
errors="$(grep -a -n -i -E "ProgressiveTaxi\[[^]]*\]: .*(error|fail|assert|panic|could not|couldn't|unable)|${SIG}.*(error|fail)" "$LOG" || true)"

ok=1
if [[ -n "$loaded" ]]; then
	echo "載入：OK"
	echo "  ${loaded}"
else
	echo "載入：沒有找到 ProgressiveTaxi 的 Loaded 行"
	ok=0
fi
if [[ -n "$banner" ]]; then
	echo "啟動訊息：OK"
	echo "  ${banner}"
else
	echo "啟動訊息：沒有找到（XPluginStart 可能沒跑完）"
	ok=0
fi
if [[ -n "$other_bp" ]]; then
	echo "注意：原版 BetterPushback 也有載入"
	echo "  ${other_bp}"
fi
if [[ -n "$errors" ]]; then
	echo "錯誤行："
	echo "$errors" | sed 's/^/  /'
	ok=0
else
	echo "錯誤行：沒有"
fi
echo
echo "插件相關的所有行："
grep -a -n "ProgressiveTaxi\|${SIG}" "$LOG" | sed 's/^/  /' | head -40

(( ok == 1 )) && { echo; echo "結論：載入成功"; exit 0; }
echo; echo "結論：有問題，見上方"
exit 1
