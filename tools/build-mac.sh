#!/bin/bash
#
# 在 macOS（Apple Silicon、新版 Xcode SDK、CMake 4）建置 ProgressiveTaxi。
#
# 原版 build_release／libacfutils build_deps 是 2023 年的工具鏈寫的，在新環境
# 會踩到下面 5 個坑。這支腳本只用環境變數、暫存的編譯器包裝和一次手動的 curl
# 建置繞過去，不改 libacfutils submodule：
#
#   1. CMake 4 移除了 cmake_minimum_required < 3.5 的相容
#      → CMAKE_POLICY_VERSION_MINIMUM=3.5
#   2. zlib 1.3 的 zutil.h 在新 SDK 下把 fdopen 定義成 NULL，撞到 _stdio.h
#      → 只對 zlib 加 -Dfdopen=fdopen
#   3. libpng 1.6.40 遇到 TARGET_OS_MAC 會去 include 不存在的 <fp.h>
#      → 只在編譯 libpng 自己的 .c 時先 -include math.h
#        （全域加會讓 configure 的 pow 檢查失敗）
#   4. build_deps 的回傳值不可信：子殼裡 make 失敗不會往上傳
#      → 最後逐一檢查每個依賴產物，缺一個就非零結束
#   5. curl 8.4 的 CMake 會抓到 Homebrew 的 libpsl，把 /opt/homebrew/include
#      放進標頭路徑，吃到 Homebrew 的 OpenSSL 4 標頭
#      → 失敗時用 -DCURL_USE_LIBPSL=OFF、隔離 pkg-config 重建 curl
#
# 另外設定 MACOSX_DEPLOYMENT_TARGET（預設 12.0），插件不會被標成只能在
# 建置機的 macOS 版本執行。
#
# 用法：tools/build-mac.sh
# 產物：ProgressiveTaxi/（發行資料夾，可整個複製到 X-Plane 的
#       Resources/plugins/ 底下）
#
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
LAU="$REPO/src/libacfutils"
export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-12.0}"
export CMAKE_POLICY_VERSION_MINIMUM=3.5
NCPUS=$(( $(sysctl -n hw.ncpu) + 1 ))

say() { printf '\n==> %s\n' "$*"; }
die() { printf '\n!!! %s\n' "$*" >&2; exit 1; }

[[ "$(uname)" == "Darwin" ]] || die "這支腳本只給 macOS 用"

say "檢查工具"
missing=()
for pair in qmake:qtbase pkg-config:pkgconf gsed:gnu-sed cmake:cmake \
    autoconf:autoconf; do
	tool="${pair%%:*}"; formula="${pair##*:}"
	if command -v "$tool" >/dev/null 2>&1; then
		printf '  %-11s %s\n' "$tool" "$(command -v "$tool")"
	else
		missing+=("$formula")
	fi
done
if (( ${#missing[@]} )); then
	die "缺少工具，請先執行：brew install ${missing[*]}"
fi

if [[ ! -f "$LAU/build_deps" ]]; then
	say "初始化 libacfutils submodule"
	git -C "$REPO" submodule update --init src/libacfutils
fi

WRAP="$(mktemp -d "${TMPDIR:-/tmp}/ptaxi-wrap.XXXXXX")"
trap 'rm -rf "$WRAP"' EXIT

cat > "$WRAP/clang-zlib" <<'EOS'
#!/bin/sh
exec /usr/bin/clang -Dfdopen=fdopen "$@"
EOS
cat > "$WRAP/clang-png" <<'EOS'
#!/bin/sh
for a in "$@"; do
	case "$a" in
	*/libpng-*/*.c) exec /usr/bin/clang -include math.h "$@" ;;
	esac
done
exec /usr/bin/clang "$@"
EOS
chmod +x "$WRAP/clang-zlib" "$WRAP/clang-png"

# 坑 2、3：zlib 和 libpng 先用包裝建好，build_deps 看到產物就會跳過
if [[ ! -f "$LAU/zlib/zlib-mac-64/lib/libz.a" ]]; then
	say "建 zlib（-Dfdopen=fdopen）"
	( cd "$LAU/zlib" && CC="$WRAP/clang-zlib" ./build_zlib_deps )
fi
if [[ ! -f "$LAU/libpng/libpng-mac-64/lib/libpng16.a" ]]; then
	say "建 libpng（只對 libpng 原始檔 -include math.h）"
	( cd "$LAU/libpng" && CC="$WRAP/clang-png" ./build_libpng_deps )
fi

say "建其餘依賴（build_deps，回傳值不採信）"
( cd "$LAU" && ./build_deps ) || echo "  build_deps 回傳非零，繼續往下逐一檢查產物"

# 坑 5：curl
build_curl() {
	local B="$LAU/curl"
	say "重建 curl（關掉 libpsl、隔離 Homebrew 的 pkg-config）"
	( cd "$B" && rm -rf libcurl-mac-64 && mkdir libcurl-mac-64 &&
	  cd libcurl-mac-64 &&
	  PKG_CONFIG_LIBDIR=/var/empty cmake \
	    -DCMAKE_INSTALL_PREFIX="$(pwd)" \
	    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
	    -DBUILD_SHARED_LIBS=OFF -DBUILD_CURL_EXE=OFF \
	    -DBUILD_STATIC_LIBS=ON -DHTTP_ONLY=ON -DENABLE_UNIX_SOCKETS=OFF \
	    -DCURL_DISABLE_COOKIES=ON -DCURL_DISABLE_NTLM=ON \
	    -DCURL_WINDOWS_SSPI=OFF -DUSE_LIBIDN2=OFF -DUSE_WIN32_IDN=OFF \
	    -DCURL_USE_LIBSSH2=OFF -DUSE_NGHTTP2=OFF -DCURL_ZSTD=OFF \
	    -DCURL_USE_LIBPSL=OFF -DCURL_BROTLI=OFF \
	    -DOPENSSL_ROOT_DIR="$LAU/ssl/openssl-mac-64" \
	    -DZLIB_INCLUDE_DIR="$LAU/zlib/zlib-mac-64/include" \
	    -DZLIB_LIBRARY="$LAU/zlib/zlib-mac-64/lib/libz.a" \
	    "-DCMAKE_OSX_ARCHITECTURES=x86_64;arm64" \
	    "../$CURL_DIR" &&
	  make -j "$NCPUS" && make install )
}
if [[ ! -f "$LAU/curl/libcurl-mac-64/lib/libcurl.a" ]]; then
	# 版本跟著 libacfutils 走（a4e2070 是 8.4.0 .tar.xz，olivierbutler 的是 8.13.0 .tar.bz2）
	CURL_TAR="$(cd "$LAU/curl" && ls curl-*.tar.* | head -1)"
	CURL_DIR="${CURL_TAR%.tar.*}"
	[[ -d "$LAU/curl/$CURL_DIR" ]] || ( cd "$LAU/curl" && tar -xf "$CURL_TAR" )
	build_curl
fi

# 坑 4：逐一檢查產物，而且都要是 x86_64 + arm64
say "檢查依賴產物"
DEPS=(
	zlib/zlib-mac-64/lib/libz.a
	libiconv/libiconv-mac-64/lib/libiconv.a
	libpng/libpng-mac-64/lib/libpng16.a
	libxml2/libxml2-mac-64/lib/libxml2.a
	freetype/freetype-mac-64/lib/libfreetype.a
	cairo/cairo-mac-64/lib/libcairo.a
	cairo/pixman-mac-64/lib/libpixman-1.a
	opus/opus-mac-64/lib/libopus.a
	opus/opusfile-mac-64/lib/libopusfile.a
	opus/libogg-mac-64/install/lib/libogg.a
	ssl/openssl-mac-64/lib/libssl.a
	ssl/openssl-mac-64/lib/libcrypto.a
	curl/libcurl-mac-64/lib/libcurl.a
	pcre2/pcre2-mac-64/lib/libpcre2-8.a
	glew/glew-1.13.0-mac-64/install/lib/libGLEWmx.a
	openal-soft/openal-soft-1.21.0-mac-64/build/install/lib/libopenal.a
	lzma/qmake/mac-64/liblzma.a
)
bad=0
for d in "${DEPS[@]}"; do
	f="$LAU/$d"
	if [[ ! -f "$f" ]]; then
		printf '  缺   %s\n' "$d"; bad=1; continue
	fi
	archs="$(lipo -archs "$f" 2>/dev/null || echo '?')"
	if [[ "$archs" != *x86_64* || "$archs" != *arm64* ]]; then
		printf '  架構不對 %s (%s)\n' "$d" "$archs"; bad=1; continue
	fi
	printf '  OK   %-58s %s\n' "$d" "$archs"
done
for h in cglm/cglm-0.7.9/include/cglm/cglm.h; do
	if [[ -f "$LAU/$h" ]]; then printf '  OK   %s\n' "$h"
	else printf '  缺   %s\n' "$h"; bad=1; fi
done
(( bad == 0 )) || die "依賴不齊，見上方清單"

say "建 libacfutils.a（qmake）"
(
	cd "$LAU/qmake"
	rm -rf mac64
	mkdir -p mac64
	make distclean >/dev/null 2>&1 || true
	qmake -set DLL 0
	qmake -set NOERRORS 1
	qmake -spec macx-clang
	make -j "$NCPUS"
	mv libacfutils.a mac64/
)
LIBACF="$LAU/qmake/mac64/libacfutils.a"
[[ -f "$LIBACF" ]] || die "libacfutils.a 沒產出"
printf '  OK   %s  %s\n' "${LIBACF#$REPO/}" "$(lipo -archs "$LIBACF")"

say "建插件（build_release，部署目標 ${MACOSX_DEPLOYMENT_TARGET}）"
( cd "$REPO" && ./build_release )

say "檢查插件產物"
XPL="$REPO/ProgressiveTaxi/mac_x64/ProgressiveTaxi.xpl"
[[ -f "$XPL" ]] || die "沒有 $XPL"
archs="$(lipo -archs "$XPL")"
[[ "$archs" == *x86_64* && "$archs" == *arm64* ]] || die "插件架構不對：${archs}"
minos="$(otool -l -arch arm64 "$XPL" | awk '/LC_BUILD_VERSION/{f=1} f&&/minos/{print $2; exit}')"
[[ "$minos" == "$MACOSX_DEPLOYMENT_TARGET" ]] || die "插件 minos=${minos}，預期 ${MACOSX_DEPLOYMENT_TARGET}"
# 先存下 nm 的輸出：直接 nm | grep -q 的話，grep 一找到就結束，nm 吃到
# SIGPIPE，pipefail 會把整條判成失敗（插件變大之後才出現）
exports="$(nm -gU -arch arm64 "$XPL")"
for sym in _XPluginStart _XPluginStop _XPluginEnable _XPluginDisable _XPluginReceiveMessage; do
	grep -q " T $sym\$" <<<"$exports" || die "插件沒有匯出 $sym"
done
extra="$(find "$REPO/ProgressiveTaxi" -name '*.xpl' ! -path "$XPL")"
[[ -z "$extra" ]] || die "發行資料夾裡有多餘的 xpl：${extra}"
printf '  OK   %s\n  架構 %s，minos %s，匯出 5 個 XPlugin 入口\n' \
    "${XPL#$REPO/}" "$archs" "$minos"
printf '  發行資料夾 %s（%s）\n' "ProgressiveTaxi/" "$(du -sh "$REPO/ProgressiveTaxi" | cut -f1)"

say "完成"
