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
- `src/libacfutils/`：submodule（v0.37-75-ga4e2070，上游已封存）。`airportdb` 只解析 1300 停機位，不解析 1201/1202 滑行網路。
