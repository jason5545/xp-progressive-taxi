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

前綴 `ptaxi/route/`。外掛啟用時就註冊，沒有路線時 `active` = 0。

| 名稱 | 型別 | 意思 |
|---|---|---|
| `version` | int | 介面版本，這份是 1。Safety Pilot 看到不認得的版本就不啟動。 |
| `id` | int | 每發布一條新路線（含重新規劃）加 1。讀的一方看到 `id` 變了才重讀陣列。 |
| `active` | int | 0 沒有路線；1 路線有效、可以跟。後推進行中一律 0。 |
| `ref_lat`、`ref_lon` | double | 座標原點（度）。 |
| `n` | int | 路徑點數，≤ 4096。 |
| `x`、`y` | float[n] | 路徑點，公尺，x 向東、y 向北（投影見下）。相鄰點距離 ≤ 2.0 m。 |
| `s` | float[n] | 從第一點起算的累積距離（m），遞增。 |
| `flags` | int[n] | 每點的旗標，見下。 |
| `hs_n` | int | 等待線個數，≤ 16。 |
| `hs_s` | float[16] | 每條等待線在路徑上的位置（`s`，m），遞增。 |
| `hs_names` | data（字串） | 等待線對應的跑道，分號分隔，順序同 `hs_s`，例：`05L/23R;05R/23L`。 |
| `rwy` | data（字串） | 起飛跑道，例：`05L`。 |
| `rwy_hdg` | float | 起飛跑道真航向（度）。 |
| `prog_s` | float | ProgressiveTaxi 自己算的機頭進度（m），給畫面和報讀用。Safety Pilot 控制用自己的投影，不依賴它。 |
| `next_turn_s` | float | 下一個轉彎（圓弧起點）的 `s`；沒有就 -1。 |
| `next_turn_deg` | float | 下一個轉彎的轉角，右轉為正。 |
| `cur_name`、`next_name` | data（字串） | 目前所在、下一條滑行道的名稱（沒名字就空字串）。 |

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

### 路徑的形狀

- 起點：後推完的主輪位置（`ptx_result_t.end`），方向沿著那條滑行道往跑道走。停機位直接滑出或從滑行道上要路線時，起點是目前主輪投影到路線第一條邊上的點。
- 路口：轉角 > 5° 的節點換成跟兩邊相切的圓弧。半徑先取這個寬度等級的偏好值，切點不能超過兩邊各自長度的一半（連續轉彎時兩個弧共用一條邊），不夠就縮小半徑。偏好值第 0 步從 RCSS／RCTP 的漆線量（見下），寫成 ini 可調。
- 終點：進到起飛跑道後，沿起飛方向在跑道中心線上再延伸 300 m（不超過跑道剩下的長度），標 `RWY_DEP`。對正之後一直到起飛對中線接手，Safety Pilot 都有線可以跟。

## 生命週期

- 發布（`active` 0 → 1、`id` 加 1）：
  - 自動後推完成、拖車離開（`ptaxi/started` = 0、`ptaxi/connected` = 0）。
  - 新指令 `ProgressiveTaxi/taxi_route`：從目前位置規劃到選單反白的跑道（選單沒開就用建議跑道）。用在停機位直接滑出、已經在滑行道上、或要重新規劃。
- 收回（`active` 1 → 0，`id` 不變）：
  - 新指令 `ProgressiveTaxi/taxi_route_clear`。
  - 飛機離地。
  - 機頭離路徑 > 50 m 超過 10 秒。
  - 在 `RWY_DEP` 段上地速 > 40 kt（已經在起飛滑跑）。
  - 開始新的後推。
- Safety Pilot 自己另外檢查 `ptaxi/started` = 0、`ptaxi/connected` = 0，拖車接著時絕不接手。

## Safety Pilot 那邊要做的（給它的規格另外寫，這裡只列對介面的期待）

- 機型宣告 `ACF_TAXI` 才開。
- 接手：地上、`active` = 1、`version` 認得、AUTO ARM ON、機頭離路徑 ≤ 5 m 且航向差 ≤ 30°、往前滑（地速 > 1 kt）、不在起飛對中線或落地流程裡。
- 控制：以機頭（鼻輪）為參考點跟路徑（駕駛艙在中心線上，跟實際操作一樣，主輪在彎裡會往內切），只寫鼻輪轉向。
- 交還：踩方向舵、DISENGAGE、AUTO ARM OFF、機頭偏 > 10 m、`active` 變 0、到 `MANUAL` 段之前、起飛對中線接手。

## 驗收怎麼做

- ProgressiveTaxi：`tools/plan_cli/plan_cli` 新增 `route` 子指令，輸出跟插件發布的同一組陣列（JSON），插件和 CLI 用同一個 C 函式產生。Safety Pilot 的離線測試拿這些 JSON 當固定資料。
- 實飛：代理不自己開 X-Plane（這個 repo 的 AGENTS.md：列清單請 Jason 開；Safety Pilot 的 AGENTS.md：Jason 當輪說可以才開）。
