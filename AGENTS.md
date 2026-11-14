# xp-progressive-taxi

X-Plane 12 插件：按一個鍵選跑道，自動把飛機推到正確的滑行道上、機頭朝向那條跑道；推完接著給 progressive taxi 指示。

這個 repo 是 BetterPushback 的 fork。2026/11/14 起基底換成 [olivierbutler/BetterPusbackMod](https://github.com/olivierbutler/BetterPusbackMod)（BPM，BetterPushback 現在有人維護的版本；用的是 `fast-ground-handling` 分支 b5a5865，也就是 x-plane.org 10/28 發布的 1.16），用 `git merge --allow-unrelated-histories` 合進來（BPM 跟 skiselkov 沒有共同歷史），我們之前的歷史都還在。之前的基底是 [skiselkov/BetterPushbackC](https://github.com/skiselkov/BetterPushbackC)（2024/4 封存）加 [Sowintuu/BetterPushback](https://github.com/Sowintuu/BetterPushback) 的 3 個修正。拖車、推行、聲音、Ground Operations 視窗、相容性程式碼沿用 BPM；新增的是自動規劃（選終點、滑行路線）和 progressive taxi。

要再跟 BPM 上游：`git fetch bpm`（remote 已設 `tagOpt=--no-tags`，不要把 BPM 的 v1.x tag 抓進來，會讓 `git describe` 版本號亂掉），再 `git merge bpm/<分支>`。上游的檔案以 BPM 為準，我們的改動要重新套：改名（下面「並存」那張表）、`src/xplane.c` 的 `ptx_xp_*` 掛勾和 `bp/*` 別名、`src/bp.c` 的推行保護和主輪版 `nearing_end()`、`src/driving.c` 拿掉 `compute_segs`（在 `src/segs.c`）。BPM 的程式是 4 空白縮排、我們的 `ptx_*`、`planner/` 是 tab，各自維持。

授權：CDDL 1.0（`COPYING`）。改到的原檔保留 CDDL 標頭和原作者版權宣告；新檔可以用自己的授權，`COPYING` 不動。

## 分支與 commit

- 直接在 `master` commit，不開分支。
- push 要 Jason 放行。

## 建置（macOS）

```sh
tools/build-mac.sh
```

需要的工具（缺的話腳本會停下來，告訴你要 `brew install` 什麼）：qtbase（qmake）、pkgconf、gnu-sed、cmake、autoconf。

腳本做的事：

1. 用暫存的編譯器包裝建 zlib、libpng（新 SDK 下原版會編譯失敗）。
2. 跑 libacfutils 的 `build_deps`，帶 `CMAKE_POLICY_VERSION_MINIMUM=3.5`（CMake 4 不再支援舊 CMakeLists）。
3. curl 沒產出時，關掉 libpsl、隔離 Homebrew 的 pkg-config 重建（不然會吃到 Homebrew 的 OpenSSL 4 標頭）。curl 版本跟著 libacfutils 的 tarball 走（目前 8.13.0）。
4. **不信任 `build_deps` 的回傳值**：子殼裡 make 失敗不會往上傳，所以逐一檢查每個依賴產物存在、而且是 x86_64＋arm64，缺一個就非零結束。
5. 用 qmake 建 `libacfutils.a`，再跑 `build_release` 建插件。
6. 檢查插件：架構、`minos` 等於部署目標、5 個 `XPlugin*` 入口、發行資料夾裡沒有別的 `.xpl`。

`build_release`／`build_xpl.sh` 是 BPM 的版本改的：只建 macOS（BPM 的 `-f` 會用 Docker 交叉編譯 Windows／Linux，我們不用），發行資料夾是 `ProgressiveTaxi/`。BPM 的 C 檔用 `-Wall -Wextra -Werror` 編譯，連註解裡寫 `ptaxi/*` 這種 `/*` 都會被擋。

部署目標預設 `MACOSX_DEPLOYMENT_TARGET=12.0`，要改就在前面設環境變數。

產物（都在 `.gitignore` 裡，不 commit）：

- `ProgressiveTaxi/`：發行資料夾，整個複製到 X-Plane 就能用。
- `mac_x64/ProgressiveTaxi.xpl`：插件本體（universal）。
- `src/libacfutils/` 底下各依賴的建置目錄、`src/libacfutils/qmake/mac64/libacfutils.a`。換了 libacfutils 版本之後，舊的建置產物要先清掉（`git -C src/libacfutils clean -xdf`），不然腳本看到產物存在就不會重建。

版本字串是 `git describe --tags --always --dirty`，會出現在 Log.txt，例如 `ProgressiveTaxi-v0.52-10-g1234abc`。

第一次從零建置約 8 分鐘，之後只重建插件約 10 秒。

## 安裝到 X-Plane

安裝位置只有一個：`~/X-Plane 12/Resources/plugins/ProgressiveTaxi/`（X-Plane 標準插件位置）。不要放到別的地方，也不要跟原版 BetterPushback 共用資料夾。

```sh
pgrep -fl "X-Plane" && echo "X-Plane 還開著，先關掉" && exit 1
rsync -a --delete ProgressiveTaxi/ "$HOME/X-Plane 12/Resources/plugins/ProgressiveTaxi/"
```

- 一定要先確認 X-Plane 沒在執行。
- `--delete` 只影響插件自己的資料夾；使用者設定存在 `Output/preferences/ProgressiveTaxi.cfg`，不在插件資料夾裡。
- 不要自己開 X-Plane（會佔用 Jason 的畫面），要實測就列清單請 Jason 開。

解除安裝：刪掉 `~/X-Plane 12/Resources/plugins/ProgressiveTaxi/`。插件另外會在 X-Plane 裡留下這些可重生的檔案：`Output/preferences/ProgressiveTaxi.cfg`、`Output/caches/ProgressiveTaxi_routes.dat`、`Output/caches/ProgressiveTaxiAirports.cache/`。

## 測試

### 插件有沒有載入

Jason 開過 X-Plane 之後：

```sh
tools/check-xplane-log.sh
```

讀 `~/X-Plane 12/Log.txt`，找 `Loaded: …/ProgressiveTaxi/mac_x64/ProgressiveTaxi.xpl (jason5545.ProgressiveTaxi).` 和插件自己印的 `This is ProgressiveTaxi-…`，並列出插件相關的錯誤行。

### 自動後推怎麼用（插件內）

指令（可以綁鍵盤或搖桿按鈕）：

| 指令 | 作用 |
|---|---|
| `ProgressiveTaxi/auto_pushback` | 開跑道選單；選單開著時再按一次＝推到反白的跑道 |
| `ProgressiveTaxi/auto_pushback_next` / `_prev` | 反白下一條／上一條跑道 |
| `ProgressiveTaxi/auto_pushback_best` | 不開選單，直接推到建議跑道 |
| `ProgressiveTaxi/auto_pushback_close` | 關選單 |
| `ProgressiveTaxi/auto_pushback_xp_tug` | 改用 X-Plane 自己的拖車（只會左轉 90°、右轉 90°、直退），方向依反白的跑道自動選 |
| `ProgressiveTaxi/taxi_route` | 不後推，從目前位置規劃滑行路線到反白的跑道（選單沒開就用建議跑道），發布成 `ptaxi/route/*` |
| `ProgressiveTaxi/taxi_route_clear` | 收回已發布的滑行路線 |

Plugins › Progressive Taxi › Auto pushback 也有同樣的項目，選單算好後會列出每條跑道（英文）。

流程：最近的機場（`XPLMFindNavAid`）→ 依 `scenery_packs.ini` 找到 X-Plane 實際用的 apt.dat（背景執行緒；Global Airports 第一次會建索引，存在 `Output/caches/ProgressiveTaxi_aptidx/`）→ 從飛機主輪位置規劃每條跑道 → 選單（中文需要系統有 PingFang TC 或 Heiti TC，沒有就英文）→ 點一條跑道 → 在 X-Plane 本地座標重算 BetterPushback 路段 → 原本的拖車流程。E、F 級機預設「全長入口」。

Log.txt 會記下載入了哪份 apt.dat、每條跑道的規劃結果，前綴 `ProgressiveTaxi`。

### 滑行路線（給 Safety Pilot）

介面規格在 `design/taxi-route-interface.md`（dataref 名稱、型別、路徑形狀、生命週期都以它為準，Safety Pilot 照它寫）。自動後推完、拖車離開後，插件把那條跑道的滑行路線發布成 `ptaxi/route/*`；`taxi_route` 指令是不後推的版本。每次發布和收回都記在 Log.txt（`taxi route published #…`、`taxi route #… withdrawn: …`）。

### C 規劃核心、CLI、對拍

`src/planner/`（不依賴 X-Plane SDK）、`src/ptx_menu.c`、`src/ptx_ui_render.c` 插件和 CLI 共用。

```sh
tools/plan_cli/build.sh            # 先跑過 tools/build-mac.sh（要 libacfutils.a）
# 一個停機位的跑道選單（文字），加 --png 輸出插件選單的樣子
tools/plan_cli/plan_cli menu --apt /tmp/apt/RCTP.dat --stand B5 \
    --wheelbase 15.39 --max-steer 75 --class C --wind 060 12 --png /tmp/menu.png
# X-Plane 對某機場會用哪份 apt.dat
tools/plan_cli/plan_cli scenery --xpdir "$HOME/X-Plane 12" --cache /tmp/idx RCTP LFMN
# 插件發布的滑行路線（JSON，鍵名＝dataref 名稱），--png 畫網路、路徑、圓弧、等待線
tools/plan_cli/plan_cli route --apt /tmp/apt/RCTP.dat --stand 516 --rwy 05R \
    --wheelbase 10.494 --max-steer 60 --class C --out /tmp/route.json --png /tmp/route.png
# 加 --from-pose：不後推，從停機位直接滑出（taxi_route 指令的規劃）
# 一個機場所有停機位 × 跑道端的滑行路線驗收（結束碼 0＝全過），ATR 72 與 777 兩組參數
tools/plan_cli/plan_cli routes --apt /tmp/apt/RCTP.dat --wheelbase 10.494 --max-steer 60 --class C
tools/plan_cli/plan_cli routes --apt /tmp/apt/RCTP.dat --wheelbase 25.348 --max-steer 70 --class E
# 進度追蹤不跳段（人工 U 形彎；5 個測試機場都沒有回頭這麼近的路線）
tools/plan_cli/plan_cli selftest
# 跟 Python 原型逐筆對拍＋驗收門檻 1–4（結束碼 0＝全過）
python3 -I tools/proto/pbplan.py --apt /tmp/apt/RCSS.dat --label x --out /tmp/x --no-html --fleet-out /tmp/fleet.txt
python3 -I tools/plan_cli/parity.py --fleet /tmp/fleet.txt --cli tools/plan_cli/plan_cli \
    --work /tmp/parity RCTP=/tmp/apt/RCTP.dat KSFO=/tmp/apt/KSFO.dat
```

改 `src/planner/` 的演算法時，Python 原型要一起改，對拍必須維持全過。`ptx_path.c` 和 `ptx_plan_from_pose()` 沒有 Python 原型，不在對拍範圍，改了跑 `plan_cli routes`。兩邊刻意用同樣的浮點寫法與走訪順序（`segs.c`、`planner/*.c` 以 `-ffp-contract=off` 編譯）。

### BPM 的回歸測試

```sh
sh tests/run_all_tests.sh
```

BPM 自帶的單元／合約測試，路徑已改成用 `src/libacfutils`。在這台 Mac 上有兩組會失敗，跟我們的改動無關：`run_ground_ops_text_fit_tests.sh`（新版 clang 對 imgui 報 `-Wnontrivial-memcall` 等警告，`-Werror` 擋下）、`run_translation_catalog_tests.sh`（寫死 `-DLIN=1`，跟 macOS SDK 的 `strlcpy` 巨集撞名）。`run_all_tests.sh` 是 `set -e`，碰到第一組就停，要看其他組就逐個跑 `tests/run_*.sh`、`tests/*.py`。

`tests/run_nosewheel_status_tests.py` 把 `src/bp.c` 裡控制寫入點（`dr_set*`、`brakes_set`、`turn_nosewheel(`、`bp.step =`）的數量和雜湊鎖死。改到這些地方要一起更新合約的數字和雜湊，並在註解寫為什麼（目前是 `push_guard()` 多的 3 個寫入和 1 個比較）。

### 離線規劃原型（第 0 步）

`tools/proto/` 是 Python 參考實作，C 規劃核心拿它當對拍基準：

```sh
# 從 apt.dat 抽出機場（Global Airports 383 MB，約 2 秒）
python3 -I tools/proto/extract_apt.py "$HOME/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat" /tmp/apt RCTP KSFO
# 每個停機位 × 每個跑道端選後推終點，輸出統計 JSON 與互動 HTML
python3 -I tools/proto/pbplan.py --apt /tmp/apt/RCTP.dat --label RCTP --out tools/proto/out
```

自訂場景（例如 Aerosoft LFMN、LPFR）的 apt.dat 在 `~/X-Plane 12/Custom Scenery/<套件>/Earth nav data/apt.dat`，優先序照 `Custom Scenery/scenery_packs.ini`。

`tools/proto/out/` 不 commit。

## 與原版 BetterPushback 並存：改了什麼

| 項目 | 原版 | 這個 fork |
|---|---|---|
| 插件資料夾 | `plugins/BetterPushback/` | `plugins/ProgressiveTaxi/` |
| 插件檔 | `mac_x64/BetterPushback.xpl` | `mac_x64/ProgressiveTaxi.xpl` |
| signature | `skiselkov.BetterPushback` | `jason5545.ProgressiveTaxi` |
| 插件名稱 | `BetterPushback-<版本>` | `ProgressiveTaxi-<版本>` |
| 指令 | `BetterPushback/*` | `ProgressiveTaxi/*`（全部，包括 BPM 新增的 pause_resume、manual_push_*、ground_ops_*、abort_push、call_emergency_tow、preference、reload、acknowledge_clear） |
| dataref | `bp/*` | `ptaxi/*`（全部，包括 BPM 新增的 planner_open、parking_brake_set、parking_brake_override、ground_crew_audio_volume）；另外用原名發布 `bp/started`、`bp/connected` 別名，見下面 |
| 物件動畫 | `bp/anim/*` | `ptaxi/anim/*`（`objects/tugs/*/*.obj`、`objects/wing_walker/wing_walker.obj` 同步改） |
| 設定檔 | `Output/preferences/BetterPushback.cfg` | `Output/preferences/ProgressiveTaxi.cfg` |
| 路線快取 | `Output/caches/BetterPushback_routes.dat` | `Output/caches/ProgressiveTaxi_routes.dat` |
| 機場快取 | `Output/caches/BetterPushbackAirports.cache` | `Output/caches/ProgressiveTaxiAirports.cache` |
| 停機位路線快取（BPM） | `Output/caches/BetterPushback_Gate_Routes/` | `Output/caches/ProgressiveTaxi_Gate_Routes/` |
| 遙測（BPM，預設不編） | `Output/BetterPushback/telemetry/` | `Output/ProgressiveTaxi/telemetry/` |
| WED 轉出的場景路線 | `Custom Scenery/*/BetterPushback_routes.dat` | `Custom Scenery/*/ProgressiveTaxi_routes.dat` |
| Plugins 選單 | Better Pushback | Progressive Taxi |
| Log.txt 前綴 | `BetterPushback` | `ProgressiveTaxi` |

刻意沒改的：

- FlightFactor A320 的 value object 名稱（`BetterPushbackStart` 等）：那架飛機是用名字找的，改了就失效。原版有裝時讓原版註冊，這個 fork 跳過。
- `BetterPushback_doors.cfg`（機型艙門定義，隨插件附帶，也讀 `Output/preferences/` 的同名檔）：跟原版共用格式和使用者自訂檔。

畫面上的名稱（視窗標題、Reload、選單）改成 Progressive Taxi／ProgressiveTaxi，`data/po/` 的翻譯檔 msgid 一起改，翻譯不會掉。

`bp/started`、`bp/connected` 別名：原版 BetterPushback 加 `bp/connected` 是給飛機外掛「拖車接上時停止自己寫鼻輪」用的（skiselkov/BetterPushbackC#268）。我們在 `XPluginEnable` 用原名註冊這兩個（跟 `ptaxi/*` 同一組變數），原版有載入就不註冊。11/14 實測：FlightFactor 777 v2 光有這兩個還是不讓出鼻輪。FF 777 後推要改用方向舵輸入（`override_joystick_heading`＋`yoke_heading_ratio`，約 76°／1.0，飽和約 ±68°），實測數據和還沒驗證的條件在 `design/ff777-nosewheel.md`。

## 程式結構（BetterPushback 既有）

- `src/xplane.c`：插件入口、選單、指令。
- `src/bp.c`：後推狀態機、拖車連接、推行控制。
- `src/driving.c`：路段計算（`compute_segs`）與跟車控制。`use_rear_pos` 開著，所以路段終點是**主輪（後軸）**位置。
- `src/bp_cam.c`：原版的手動規劃畫面。
- `src/wed2route.c`：從 `Custom Scenery/*/earth.wed.xml` 轉出作者定義的後推路線。
- `src/segs.c`：`compute_segs()`，從 driving.c 原封搬出來，不依賴 X-Plane SDK。
- `src/libacfutils/`：submodule，[olivierbutler/libacfutils](https://github.com/olivierbutler/libacfutils) bc0b479（skiselkov a4e2070 之後再 128 個 commit，XP SDK 4.3.0；BPM 指定用這個 fork）。BPM 原本要求 libacfutils 放在 repo 外面的同層資料夾，我們維持 submodule。`airportdb` 只解析 1300 停機位，不解析 1201/1202 滑行網路。
- BPM 新增的：Ground Operations 視窗（`ground_ops_*`、imgui）、手動推、緊急拖、wing walker、Fast Ground Handling、停機位路線快取（`gate_route_*`）、鼻輪動畫狀態（`nw_*`，給飛機做鼻輪動畫用）。說明在 `README.md` 後半、`USER_GUIDE.md`。

## 程式結構（這個 fork 新增）

- `src/planner/ptx_apt.c`：apt.dat 讀取（100、1300/1301、1201、1202、1204）。
- `src/planner/ptx_scenery.c`：依 scenery_packs.ini 找 apt.dat、大檔建索引。
- `src/planner/ptx_plan.c`：選後推終點、路由到跑道入口、停機位分類。`tools/proto/pbplan.py` 的 C 版。另有不後推的 `ptx_plan_from_pose()`、等待線 `ptx_route_holds()`。
- `src/planner/ptx_path.c`：規劃結果 → 發布用的滑行路徑（路口圓弧、跑道延伸、flags、等待線、滑行道名稱區段、進度追蹤）。插件和 `plan_cli route` 共用。
- `src/ptx_menu.c`：規劃結果 → 選單文字。
- `src/ptx_ui_render.c`：選單版面與 cairo 繪製、字型。
- `src/ptx_ui.c`：X-Plane 視窗。
- `src/ptx_xp.c`：指令、背景載入、規劃、交給 BetterPushback 或 X-Plane 拖車、Plugins 子選單、`taxi_route` 指令。
- `src/ptx_route.c`：`ptaxi/route/*` dataref、發布／收回、每幀進度。
- `src/steer_late.c`：後推時跟飛機外掛搶鼻輪角度（FF 777 v2 每幀把 `tire_steer_cmd` 寫回 0）。bp.c 每次寫鼻輪指令就呼叫 `steer_late_note()`：馬上一起寫實際角 `tire_steer_act`；後推開始時另建一個 BeforeFlightModel 的 flight loop，同一幀再寫一次指令角和實際角。每趟後推結束記一行 `steer_late: … steering frames …`，統計值在哪個時間點被別人改掉。`push_guard()` 照舊讀實際角，鼻輪還是不跟就停推。11/14 寫好，還沒實飛驗證。同天在停機位用 Web API 直接寫 `tire_steer_cmd`／`tire_steer_act`，讀回全是 0（FF 連實際角都蓋），所以大概沒用；FF 777 改用方向舵輸入帶鼻輪，見 `design/ff777-nosewheel.md`。
- `tools/plan_cli/plan_route.c`：`plan_cli route`／`routes`。
- `tools/proto/cl_vs_net.py`、`tools/proto/paint_radius.py`：量漆線離滑行網路多遠、路口漆線彎道半徑（定圓弧偏好半徑用）。
