# 滑行路線介面：ProgressiveTaxi → Safety Pilot

2026/11/13 定。ProgressiveTaxi 算滑行路線、發布成 dataref；Safety Pilot（`~/GitHub/xplane-safety-pilot`）讀它，滑行時替 Jason 轉鼻輪守中心線。

## 範圍（Jason 11/13 定）

- **只管方向**：Safety Pilot 跟起飛對中線（ACF_TOCL）一樣只轉鼻輪。油門、煞車、在等待線停住都是 Jason 的。
- **出場先做**：後推完（或停機位直接滑出）→ 跑道等待線 → 進跑道對正 → 起飛滑跑交給起飛對中線。進場（脫離跑道 → 停機位）之後再做。

## 分工

| | ProgressiveTaxi（這個 repo） | Safety Pilot |
|---|---|---|
| 角色 | 導航：走哪條路、中心線在哪、下一個轉彎、等待線在哪 | 操縱：把機頭帶在中心線上 |
| 做的事 | 規劃路線、在路口做圓弧（fillet）、延伸到跑道上對正、發布 dataref、自己算進度 | 判斷何時接手／交還、控制律、鼻輪輸出、報讀 |
| 不做的事 | 不碰任何操縱 dataref | 不自己規劃路線、不讀滑行網路 |

兩個外掛只透過下面的 dataref 溝通，不互相連結程式碼（ProgressiveTaxi 是 CDDL）。ProgressiveTaxi 沒裝或沒有路線時，Safety Pilot 的滑行功能就不啟動。

## 為什麼跟著滑行網路走，不跟地上漆的黃線

11/13 量過（`tools/proto/cl_vs_net.py <apt.dat> 1,51,7,57`：每條非跑道的網路邊，取離兩端 > 25 m 的中段每 5 m 一點，量到最近一段漆線的距離）：

| 機場（X-Plane 實際用的 apt.dat） | 網路邊中段到漆線的距離 | 沒有漆線（> 20 m）的取樣 |
|---|---|---|
| RCSS（Global Airports） | p50 0.61 m、p90 2.67 m | 0% |
| RCTP（Global Airports） | p50 2.16 m | 20% |
| RCKH（Global Airports） | — | 100% |
| KSFO（Global Airports） | — | 92% |
| LFMN（Aerosoft 自訂場景） | — | 92% |

多數機場地上的線是場景另外畫的，外掛讀不到。滑行網路（1201／1202）每個機場都有，所以第一版以網路為準，路口用圓弧接起來。有漆線的機場改貼漆線，之後再做。

## Dataref（ProgressiveTaxi 擁有，別人只讀）

前綴 `ptaxi/route/`。外掛啟用時就註冊，沒有路線時 `active` = 0。全部唯讀。

陣列的註冊長度固定：`x`、`y`、`s`、`flags` 是 4096 個元素，`hs_s` 是 16 個；有效的只有前 `n`（`hs_n`）個，後面是上一條路線的殘值，不要讀。字串是固定長度、以 NUL 結尾的位元組陣列（`hs_names` 256、`rwy` 8、`cur_name`／`next_name` 72），讀的時候讀到第一個 NUL 為止。

| 名稱 | 型別 | 意思 |
|---|---|---|
| `version` | int | 介面版本，這份是 1。Safety Pilot 看到不認得的版本就不啟動。 |
| `id` | int | 每發布一條新路線（含重新規劃）加 1。讀的一方看到 `id` 變了才重讀陣列。 |
| `active` | int | 0 沒有路線；1 路線有效、可以跟。後推進行中一律 0。 |
| `ref_lat`、`ref_lon` | double | 座標原點（度）。註冊成 double，也能當 float 讀，但 float 只有約 0.2 m 精度，一定要用 `XPLMGetDatad`。 |
| `n` | int | 路徑點數，≤ 4096。 |
| `x`、`y` | float[n] | 路徑點，公尺，x 向東、y 向北（投影見下）。相鄰點距離 ≤ 2.0 m。 |
| `s` | float[n] | 從第一點起算的累積距離（m），遞增。 |
| `flags` | int[n] | 每點的旗標，見下。 |
| `hs_n` | int | 等待線個數，≤ 16。 |
| `hs_s` | float[16] | 每條等待線在路徑上的位置（`s`，m），遞增。 |
| `hs_names` | data（字串） | 等待線對應的跑道，分號分隔，順序同 `hs_s`，例：`05L/23R;05R/23L`。一條等待線同時管幾條跑道時（1204 區域列了好幾條、或兩條等待線落在同一點），那一段裡用逗號分隔，例：`01L/19R,10/28`。跑道名稱照 apt.dat 100 列的寫法。 |
| `rwy` | data（字串） | 起飛跑道，例：`05L`。 |
| `rwy_hdg` | float | 起飛跑道真航向（度）。 |
| `prog_s` | float | ProgressiveTaxi 自己算的機頭進度（m），給畫面和報讀用。Safety Pilot 控制用自己的投影，不依賴它。每幀只在上一次的 `prog_s` 往回 20 m、往前 150 m 的範圍裡找，取「離路徑距離 + 0.1 ×（離上一次 `prog_s` 多遠）」最小的點，路徑繞回同一區時不會跳段（`plan_cli selftest`：U 形彎回程離機頭比去程近 1 m 也不跳）；挑到的點離機頭超過 30 m 時才全路徑重找最近點。 |
| `next_turn_s` | float | 下一個轉彎（圓弧起點）的 `s`：起點 ≥ `prog_s` 的第一個轉彎，已經在彎裡就報再下一個；沒有就 -1。 |
| `next_turn_deg` | float | 下一個轉彎的轉角，右轉為正。同方向、間隔不到 10 m 的幾個圓弧合併成一個轉彎，轉角相加。 |
| `cur_name`、`next_name` | data（字串） | 目前所在、下一條不同名稱的滑行道（沒名字就空字串）。跑道段寫成 `RWY 05L/23R`（穿越）或 `RWY 05L`（起飛跑道上的延伸段）；直線接出段（見下）沒有名字。 |

### 投影

跟規劃核心 `ptx_project()` 同一個公式（`src/planner/ptx_plan.c`）：

```
k = π / 180 × 6371008.8
x = (lon − ref_lon) × cos(ref_lat) × k
y = (lat − ref_lat) × k
```

`ref_lat`／`ref_lon` 就是規劃用的原點（跑道端點的平均）。讀的一方拿 `sim/flightmodel/position/latitude`／`longitude`（double）照這個公式投影，誤差在機場範圍內是公分級。

### flags

| bit | 名稱 | 意思 |
|---|---|---|
| 0 | `ARC` | 這點在路口圓弧上 |
| 1 | `RWY_DEP` | 在起飛跑道上（含 backtrack、對正後的延伸段） |
| 2 | `RWY_CROSS` | 在要穿越的其他跑道上 |
| 3 | `MANUAL` | 幾何不可靠，Safety Pilot 到這裡之前要交還：轉角 > 150°（跑道頭迴轉、U 形彎）、圓弧半徑小於這架飛機的最小迴轉半徑 |

- `ARC`：圓弧上的點，而且那個轉彎（合併後）合計 > 5°。≤ 5° 的小折角也做圓弧（不然會留下折角），但不標 `ARC`、不算轉彎。
- `RWY_DEP`：起飛跑道的等待線之後、在起飛跑道矩形裡（100 列兩端、寬度）的點，加上進跑道圓弧之後的整段延伸。
- `RWY_CROSS`：在其他跑道矩形裡的點（幾何判斷，不看滑行網路的邊型別）。
- `MANUAL`：
  - 最小迴轉半徑 = 軸距 ÷ sin(0.9 × 鼻輪最大轉角)，是鼻輪貼著線走的半徑；0.9 是 BetterPushback `compute_segs()` 的同一個餘裕。軸距、最大轉角照 BetterPushback（`bp.c`）的算法：鼻輪 = 最前面的可轉向輪，主輪 = 其他輪的平均，最大轉角 = min(max(acf_nw_steerdeg1, 2), 75°)。11/13 從 .acf 讀：DWSim ATR 72-600 軸距 10.49 m、60° → 12.97 m；FlightFactor 777-200ER 25.35 m、70° → 28.45 m。
  - 半徑小於它的圓弧，弧上每一點都標。
  - 轉角 > 150° 的角不做圓弧，留成尖角，前後各「偏好半徑」那麼長（不超過相鄰邊長）都標。目前的規劃核心每個路口最多轉 120°，進跑道最大量到 145°，5 個機場都沒出現過。

### 路徑的形狀

- 起點有三種：
  - 自動後推完：後推完的主輪位置（`ptx_result_t.end`），方向沿著那條滑行道往跑道走。
  - `ProgressiveTaxi/taxi_route`，機頭 15 m 內有方向跟航向差 ≤ 45° 的滑行道邊（已經在滑行道上）：主輪投影到那條邊上的點。
  - `ProgressiveTaxi/taxi_route`，停機位面向滑行道（附近沒有同向的邊；11/13 量 LFMN 210 個面向滑行道的停機位，最近的邊中位數 26 m、夾角 90°，同向的只有 1 個）：從目前主輪位置沿目前航向直線往前，接到機頭前方 2–150 m 第一條碰到的滑行道邊，接點做圓弧轉上去（往哪一邊走由路由決定，轉角 ≤ 120°）。這段直線接出不屬於滑行網路，沒有名字。
  - 兩種 `taxi_route` 的候選一起比路由成本，直線接出多算 20 m，同樣好時留在原來的滑行道上。
- 路口：每個折角（> 0.05°）都換成跟兩邊相切的圓弧。半徑先取偏好值（見下表），切點離角的距離（切線長 = R × tan(θ/2)）有上限：
  - 這條邊另一端也有圓弧時，最多用邊長的一半；另一端的圓弧用不到一半，剩下的可以給這一端。
  - 另一端沒有圓弧（路徑起點、跑道延伸的終點、直通的節點）時整條邊都能用。
  - 超過上限就縮小半徑。圓弧離原本折線最遠 R × (sec(θ/2) − 1)。
- 偏好半徑按路口兩條滑行道裡較窄的寬度等級（1202 的 `taxiway_X`）取；沒有等級的邊用 C 的值。11/13 量 RCSS（線型 1、51）和 RCTP（線型 51，滑行道中線燈）漆線在路口的彎道半徑（`paint_radius.py`：轉 30–135° 的彎道、對上 60 m 內兩邊方向都吻合的路口）：

  | 路口較窄的等級 | 樣本 | 量到的半徑 | 預設值 |
  |---|---|---|---|
  | A | 無 | — | 15 m（未量，內插） |
  | B | 無 | — | 20 m（未量，內插） |
  | C | RCSS 14 | p25 30.3 / p50 35.2 / p75 39.0 | 35 m |
  | D | 無 | — | 45 m（未量，內插） |
  | E | RCSS 5、RCTP 6 | RCSS p50 51.3；RCTP（E/F）p50 63.9 | 50 m |
  | F | RCTP 66 | p25 54.5 / p50 57.8 / p75 60.4 | 55 m |

  可以在 `Output/preferences/ProgressiveTaxi.cfg` 用 `taxi_route_radius_A` … `taxi_route_radius_F`、`taxi_route_radius_unknown`（公尺）覆寫。
- 終點：路由的最後一個節點（跑道上的入口節點）先移到 100 列的跑道中心線上（垂足；5 個機場離中心線最多 2.43 m），再沿起飛方向在中心線上延伸 300 m（不超過跑道剩下的長度；5 個機場剩下的最少 2233 m，從沒碰到上限），標 `RWY_DEP`。進跑道的轉彎跟其他路口一樣做圓弧，延伸段那一側整段都能當切線用。
- 等待線位置：規劃核心的等待點節點（1204 起飛區域的起點；沒有 1204 時是進跑道前的最後一個節點，5 個機場都有 1204），在路徑上取那個節點圓弧的中點。穿越其他跑道時，在進入那條跑道（或它的 1204 區域）的節點各放一條。
## 生命週期

- 發布（`active` 0 → 1、`id` 加 1）：
  - 自動後推完成、拖車離開（`ptaxi/started` = 0、`ptaxi/connected` = 0），而且主輪停在路徑起點 15 m 內。`ProgressiveTaxi/stop` 中途停推也會把 `op_complete` 設成 1，分不出來，所以改看位置；離太遠就不發布，Log.txt 記一行。路線在按下選單那一刻就算好，拖車走了才發布。用 X-Plane 自己的拖車時不發布（它只會 90° 轉），推完再按 `taxi_route`。
  - 新指令 `ProgressiveTaxi/taxi_route`：從目前位置規劃到選單反白的跑道（選單沒開就用建議跑道：逆風最大，再比滑行距離加 2 倍少掉的跑道長度，跟選單同一套）。用在停機位直接滑出、已經在滑行道上、或要重新規劃。要在地上、沒有在後推；機場資料還沒載入時先載入，載完自動規劃。軸距、轉角直接讀起落架 dataref，翼展從 .acf 讀，滑行中也能用。Plugins › Progressive Taxi › Auto pushback 裡有同名的選單項目。
  - 每次發布，Log.txt 記一行摘要：跑道、點數、長度、轉彎數（MANUAL、縮小、最小半徑）、等待線和位置、經過的滑行道。
- 收回（`active` 1 → 0，`id` 不變）：
  - 新指令 `ProgressiveTaxi/taxi_route_clear`。
  - 飛機離地（`sim/flightmodel/failures/onground_any` = 0）。
  - 機頭離路徑 > 50 m 超過 10 秒。
  - 在 `RWY_DEP` 段上（以 `prog_s` 所在的點判斷）地速 > 40 kt（已經在起飛滑跑）。
  - 開始新的後推。
- Safety Pilot 自己另外檢查 `ptaxi/started` = 0、`ptaxi/connected` = 0，拖車接著時絕不接手。

## Safety Pilot 那邊要做的（給它的規格另外寫，這裡只列對介面的期待）

- 機型宣告 `ACF_TAXI` 才開。
- 接手：地上、`active` = 1、`version` 認得、AUTO ARM ON、機頭離路徑 ≤ 5 m 且航向差 ≤ 30°、往前滑（地速 > 1 kt）、不在起飛對中線或落地流程裡。
- 控制：以機頭（鼻輪）為參考點跟路徑（駕駛艙在中心線上，跟實際操作一樣，主輪在彎裡會往內切），只寫鼻輪轉向。
- 交還：踩方向舵、DISENGAGE、AUTO ARM OFF、機頭偏 > 10 m、`active` 變 0、到 `MANUAL` 段之前、起飛對中線接手。

## 驗收怎麼做

- ProgressiveTaxi：`tools/plan_cli/plan_cli route` 輸出跟插件發布的同一組陣列（JSON，鍵名就是 dataref 名稱，數值先轉成 float 再印，跟 dataref 讀到的一樣；`prog_s` 和跟它有關的值是 `prog_s` = 0 時的值；另外多 `turns`、`spans`、`meta` 三個鍵，不是 dataref）。插件和 CLI 都呼叫 `src/planner/ptx_path.c` 的 `ptx_path_build()`。Safety Pilot 的離線測試拿這些 JSON 當固定資料。`--from-pose` 是 `taxi_route` 的規劃（不後推，從停機位滑出）。
- `plan_cli routes` 對一個機場所有停機位 × 跑道端跑一遍，檢查點距、`s` 遞增、折角、圓弧偏離、延伸段、等待線、進度追蹤，結束碼 0 = 全過。
- 實飛：代理不自己開 X-Plane（這個 repo 的 AGENTS.md：列清單請 Jason 開；Safety Pilot 的 AGENTS.md：Jason 當輪說可以才開）。
