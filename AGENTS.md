# xp-progressive-taxi

X-Plane 12 插件：按一個鍵選跑道，自動把飛機推到正確的滑行道上、機頭朝向那條跑道；推完接著給 progressive taxi 指示。

這個 repo 是 [skiselkov/BetterPushbackC](https://github.com/skiselkov/BetterPushbackC)（2024/4 封存）的 fork，合進了 [Sowintuu/BetterPushback](https://github.com/Sowintuu/BetterPushback) 2026/10/24 的 3 個修正。拖車、推行、聲音、相容性程式碼沿用 BetterPushback；新增的是自動規劃（選終點、滑行路線）和 progressive taxi。

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
3. curl 沒產出時，關掉 libpsl、隔離 Homebrew 的 pkg-config 重建（不然會吃到 Homebrew 的 OpenSSL 4 標頭）。
4. **不信任 `build_deps` 的回傳值**：子殼裡 make 失敗不會往上傳，所以逐一檢查每個依賴產物存在、而且是 x86_64＋arm64，缺一個就非零結束。
5. 用 qmake 建 `libacfutils.a`，再跑 `build_release` 建插件。
6. 檢查插件：架構、`minos` 等於部署目標、5 個 `XPlugin*` 入口、發行資料夾裡沒有別的 `.xpl`。

部署目標預設 `MACOSX_DEPLOYMENT_TARGET=12.0`，要改就在前面設環境變數。

產物（都在 `.gitignore` 裡，不 commit）：

- `ProgressiveTaxi/`：發行資料夾，整個複製到 X-Plane 就能用。
- `mac_x64/ProgressiveTaxi.xpl`：插件本體（universal）。
- `src/libacfutils/` 底下各依賴的建置目錄、`src/libacfutils/qmake/mac64/libacfutils.a`。

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

Plugins › Progressive Taxi › Auto pushback 也有同樣的項目，選單算好後會列出每條跑道（英文）。

流程：最近的機場（`XPLMFindNavAid`）→ 依 `scenery_packs.ini` 找到 X-Plane 實際用的 apt.dat（背景執行緒；Global Airports 第一次會建索引，存在 `Output/caches/ProgressiveTaxi_aptidx/`）→ 從飛機主輪位置規劃每條跑道 → 選單（中文需要系統有 PingFang TC 或 Heiti TC，沒有就英文）→ 點一條跑道 → 在 X-Plane 本地座標重算 BetterPushback 路段 → 原本的拖車流程。E、F 級機預設「全長入口」。

Log.txt 會記下載入了哪份 apt.dat、每條跑道的規劃結果，前綴 `ProgressiveTaxi`。

### C 規劃核心、CLI、對拍

`src/planner/`（不依賴 X-Plane SDK）、`src/ptx_menu.c`、`src/ptx_ui_render.c` 插件和 CLI 共用。

```sh
tools/plan_cli/build.sh            # 先跑過 tools/build-mac.sh（要 libacfutils.a）
# 一個停機位的跑道選單（文字），加 --png 輸出插件選單的樣子
tools/plan_cli/plan_cli menu --apt /tmp/apt/RCTP.dat --stand B5 \
    --wheelbase 15.39 --max-steer 75 --class C --wind 060 12 --png /tmp/menu.png
# X-Plane 對某機場會用哪份 apt.dat
tools/plan_cli/plan_cli scenery --xpdir "$HOME/X-Plane 12" --cache /tmp/idx RCTP LFMN
# 跟 Python 原型逐筆對拍＋驗收門檻 1–4（結束碼 0＝全過）
python3 -I tools/proto/pbplan.py --apt /tmp/apt/RCSS.dat --label x --out /tmp/x --no-html --fleet-out /tmp/fleet.txt
python3 -I tools/plan_cli/parity.py --fleet /tmp/fleet.txt --cli tools/plan_cli/plan_cli \
    --work /tmp/parity RCTP=/tmp/apt/RCTP.dat KSFO=/tmp/apt/KSFO.dat
```

改 `src/planner/` 的演算法時，Python 原型要一起改，對拍必須維持全過。兩邊刻意用同樣的浮點寫法與走訪順序（`segs.c`、`planner/*.c` 以 `-ffp-contract=off` 編譯）。

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
| 指令 | `BetterPushback/*` | `ProgressiveTaxi/*`（start、stop、start_planner、stop_planner、connect_first、cab_camera、recreate_scenery_routes、disconnect、reconnect、reload_tug） |
| dataref | `bp/*` | `ptaxi/*`（started、connected、slave_mode、op_complete、plan_complete、tug_name、anim/*） |
| 拖車物件動畫 | `bp/anim/*` | `ptaxi/anim/*`（`objects/tugs/*/*.obj` 同步改） |
| 設定檔 | `Output/preferences/BetterPushback.cfg` | `Output/preferences/ProgressiveTaxi.cfg` |
| 路線快取 | `Output/caches/BetterPushback_routes.dat` | `Output/caches/ProgressiveTaxi_routes.dat` |
| 機場快取 | `Output/caches/BetterPushbackAirports.cache` | `Output/caches/ProgressiveTaxiAirports.cache` |
| WED 轉出的場景路線 | `Custom Scenery/*/BetterPushback_routes.dat` | `Custom Scenery/*/ProgressiveTaxi_routes.dat` |
| Plugins 選單 | Better Pushback | Progressive Taxi |
| Log.txt 前綴 | `BetterPushback` | `ProgressiveTaxi` |

刻意沒改的：

- FlightFactor A320 的 value object 名稱（`BetterPushbackStart` 等）：那架飛機是用名字找的，改了就失效。原版有裝時讓原版註冊，這個 fork 跳過。
- 有翻譯的使用者字串（例如「BetterPushback Preferences」、「aircraft is incompatible with BetterPushback」）：改英文原文會讓 6 種語言的翻譯對不上。之後改 UI 時再一起處理。

相容性取捨：外部程式如果讀 `bp/started`、`bp/connected` 來判斷有沒有在後推，對這個 fork 會讀不到，要改讀 `ptaxi/*`。

## 程式結構（BetterPushback 既有）

- `src/xplane.c`：插件入口、選單、指令。
- `src/bp.c`：後推狀態機、拖車連接、推行控制。
- `src/driving.c`：路段計算（`compute_segs`）與跟車控制。`use_rear_pos` 開著，所以路段終點是**主輪（後軸）**位置。
- `src/bp_cam.c`：原版的手動規劃畫面。
- `src/wed2route.c`：從 `Custom Scenery/*/earth.wed.xml` 轉出作者定義的後推路線。
- `src/segs.c`：`compute_segs()`，從 driving.c 原封搬出來，不依賴 X-Plane SDK。
- `src/libacfutils/`：submodule（v0.37-75-ga4e2070，上游已封存）。`airportdb` 只解析 1300 停機位，不解析 1201/1202 滑行網路。

## 程式結構（這個 fork 新增）

- `src/planner/ptx_apt.c`：apt.dat 讀取（100、1300/1301、1201、1202、1204）。
- `src/planner/ptx_scenery.c`：依 scenery_packs.ini 找 apt.dat、大檔建索引。
- `src/planner/ptx_plan.c`：選後推終點、路由到跑道入口、停機位分類。`tools/proto/pbplan.py` 的 C 版。
- `src/ptx_menu.c`：規劃結果 → 選單文字。
- `src/ptx_ui_render.c`：選單版面與 cairo 繪製、字型。
- `src/ptx_ui.c`：X-Plane 視窗。
- `src/ptx_xp.c`：指令、背景載入、規劃、交給 BetterPushback 或 X-Plane 拖車、Plugins 子選單。
