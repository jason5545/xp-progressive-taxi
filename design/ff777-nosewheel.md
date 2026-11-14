# FlightFactor 777 v2：後推時誰能轉鼻輪

這份記錄 2026/11/14 的調查和實測，給之後實作「777 後推改用方向舵帶鼻輪」用。
機型：`Aircraft/FlightFactor777_200ER_F/777-F_xp12.acf`（FF 777 v2，2.6.8-beta）。

## 結論

- FF 777 自己接管鼻輪：`sim/operation/override/override_wheel_steer` 一直是 1，每幀自己寫 `tire_steer_cmd` 和 `tire_steer_act`。外掛直接寫這兩個（包括實際角）都會被蓋掉。
- 插件發布 `bp/connected` 沒用（11/14 實測：接上拖車後一直是 1，鼻輪照樣 0°）。插拖行銷、EFB 的 FWS LOCK、JAR GHD INTEGRATION 也都沒用。
- **方向舵輸入有用**：用 `override_joystick_heading` 加 `yoke_heading_ratio` 給方向舵，FF 自己把鼻輪轉到將近 ±68°，實際角跟得上。也就是 FF 現在的設定是「方向舵帶 tiller」，不是真機只有 ±7° 的踏板轉向（真機數字見下面來源）。
- 所以 777 後推的做法：插件照算要的鼻輪角度，換算成方向舵輸入寫給 FF，讓 FF 自己轉鼻輪。

## 實測（11/14 10:5x，RCSS 74 號停機位，發動機運轉、停機煞車拉著、地速 0）

工具：X-Plane Web API（localhost:8086），腳本在當時的 scratchpad，以下是原始輸出。

### A：直接寫鼻輪角度

每 0.02 s 把 `sim/flightmodel/parts/tire_steer_cmd[0]` 和 `tire_steer_act[0]` 寫成 20，連續 2 秒，每次寫完立刻讀回：

```
baseline                     cmd=   0.00 act=   0.00 fm2cmd=   0.00 fm2act=   0.00 gs=0.000 pbrk=1.0
A: 8 write/read cycles; readback cmd==20: 0, act==20: 0, fm2act==20: 0
A after 0.5 s                cmd=   0.00 act=   0.00 fm2cmd=   0.00 fm2act=   0.00 gs=0.000 pbrk=1.0
```

讀回全部是 0：FF 連實際角都每幀寫回去。Web API 的寫入時間點跟插件 flight loop 不同，在插件 flight loop 裡補寫的版本（`steer_late.c`）也還沒實飛驗證就拿掉了，改走方向舵。

### B：方向舵輸入

`sim/operation/override/override_joystick_heading` = 1，持續寫 `sim/joystick/yoke_heading_ratio`，每個值 2.5 秒：

```
B yaw=+1.0: last cmd=67.58 fm2act=67.85 ff_rudder_anim=0.000 rpedal_anim=1.000  max|cmd|=67.58 max|act|=67.85
B yaw=-1.0: last cmd=-36.42 fm2act=-38.83 ...  max|cmd|=68.49 max|act|=67.73
B yaw=+0.0: last cmd=-0.90 fm2act=-0.79 ...
```

（yaw −1.0 那段 2.5 秒還沒轉到底，從 +67.6° 一路轉過去；max 欄位看得到它到過 −68.5°。）

### C：輸入對角度、轉動速度

```
yaw=+0.10: settled cmd=   7.46 act=   7.49
yaw=+0.25: settled cmd=  18.96 act=  19.00
yaw=+0.50: settled cmd=  38.02 act=  38.08
yaw=+0.75: settled cmd=  57.42 act=  57.46
yaw=-0.50: settled cmd= -25.87 act= -27.12     ← 2.2 秒內還沒從 +57 轉到位
slew 0->+1: reach 30 deg at 0.86 s, 60 deg at 1.73 s
```

- 大致線性：約 76°／1.0 輸入，飽和在約 ±68°。正值＝右轉，跟 `tire_steer_cmd` 同號。
- 轉動速度約 35°/s。後推轉彎時插件的鼻輪指令變化比這慢，但要轉大角度時會落後約 1–2 秒。
- 每項測完都把 `override_joystick_heading`、`yoke_heading_ratio` 還原成 0，最後確認實際角回到 0°。

## 還沒驗證的

- **拖車接上、拖行銷插著、FWS LOCK 鎖上時，方向舵還帶不帶得動鼻輪**。這次只在停機位、沒接拖車時測。
- **發動機關著、只開 APU 時**。這次發動機是開著的；鼻輪轉向要中央液壓有壓力，冷機後推時可能沒壓力。
- **FF 的「方向舵帶 tiller」是哪個選項、預設開不開**。FF 更新紀錄有「Tiller by Rudder」「limited Tiller control via pedals」，選項檔裡沒找到對應名稱；如果是使用者選項，別人的設定可能只有 ±7°。
- **推行中 X-Plane 物理會不會照 FF 的鼻輪角度轉**。停機位只能看角度，看不到軌跡。

## 實作（11/14，`src/rudder_steer.c`，還沒實飛）

後推時轉向的是拖車，鼻輪只是被拖車帶著轉：`turn_nosewheel()` 先轉拖車（`tug_set_steering`），再把拖車相對機身的角度當成鼻輪角。X-Plane 裡沒有真的拖車連桿，物理只認飛機的鼻輪角，所以「讓鼻輪等於拖車帶出來的角度」就是後推轉彎本身。FF 777 不讓外掛直接寫鼻輪角，就改用方向舵把同一個角度交給 FF，FF 自己的拖車也是這樣（`pushback.state.ini` 的 `tireAngle`）。

機型判斷：`acf_author` 含 FlightFactor、`acf_ICAO` 開頭 B77（777-F 是 B77L，200ER 是 B772）。

1. 後推開始時記下 `override_joystick_heading` 原值，設成 1。
2. 每幀把要的鼻輪角度換算成 `yoke_heading_ratio = clamp(deg / 76, -1, 1)` 寫進去。
3. 後推結束、停推、插件停用時一定還原 `override_joystick_heading` 和 `yoke_heading_ratio`（不然 Jason 的方向舵會一直被插件吃掉）。
4. `push_guard()` 照留：方向舵也帶不動（例如沒液壓）就照樣停推。
5. 只在 `bp_connected` 期間接管方向舵；拖車沒接上之前方向舵還是駕駛的。
6. 每趟後推記一行 `rudder_steer: rudder given back; … frames, max nosewheel command …, nosewheel more than 10 deg off in … frames`，看鼻輪跟不跟得上。

## 來源

- BetterPushback 的 `bp/connected` 是給飛機外掛停止自己轉鼻輪用的：commit 72d7d04「Implemented bp/connected dataref for steering lockout」；skiselkov/BetterPushbackC#268 原作者的說明。
- FF 777 跟 BetterPushback 的相容性：x-plane.org 論壇 topic 316081「Pushback broken in 2.0.17 - FIXED」（2024/8，症狀相同，FF 的 Ivan_12 回下一版修好）、topic 339440（2025/11，原版 BPB 在 777 客機版正常）。
- 真機 777 鼻輪轉向：tiller ±70°、方向舵踏板 ±7°（AOPA 1999/12「The rudder pedals alone provide only 7 degrees」；FlightGear 論壇 2020/4；多份 777 訓練卡片一致）。
- 11/14 插件後推 Log：`pushback stopped: nosewheel steering ignored for 3.0 s (commanded -56.9 deg, actual 0.0 deg, gear 0)`。
