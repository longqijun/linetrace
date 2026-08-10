# PID巡线算法详细分析（8路传感器版）

**分析基于的代码版本（commit）**：`c699d96`

本文汇总 PID 模式（`TRACK_ALGO_PID`）在**8路传感器**下的实现细节、误差来源、以及外轮封顶等关键结构改动，便于调参和排障时查阅。历史上的"车身晃动"排查（5路时代）见文末第 6 节，仅作历史记录。

> **重要更新**：本项目已从 5 路（CH2~CH6）升级为 **8 路（CH1~CH8）**，误差信号也从 `is_white[]` 二值离散量改成了**模拟量加权的连续位置**。旧文档里"5 路 / 9 档离散阶梯 / 对称差速 base±output"的描述都已过时，本文按最新代码重写。

---

## 1. 硬件与传感器布局（8路）

- 8 路光电，`PINS = {32, 33, 34, 35, 36, 39, 13, 14}`，索引 `index0=CH1 … index7=CH8`（`sensor_module.cpp:6`）。
- **传感器物理反装（180° 翻转）**：`index0(CH1)` 现在在**物理最右**，`index7(CH8)` 在**物理最左**。归一化时符号取反，保证 −1 仍代表物理最左（`sensor_module.cpp:78-79`）。
- 物理左→右顺序：`CH8 CH7 CH6 CH5 | CH4 CH3 CH2 CH1`。
- **标定状态**：8 路 THRESHOLD / WHITE_REF / BLACK_REF **均已实车标定**——CH1/CH7/CH8 是新接入后用 `print on` 看原始 ADC 白/黑值、再用 `threshold` / `whiteref` / `blackref` 命令逐路测量重设并 `save` 的。**实际运行值来自设备 flash 的 `/config.json`，不是源码里的默认值。** 源码 `sensor_module.cpp:10-19` 里 CH1/CH7/CH8 仍写着 `占位未校准`，那只是无 config 文件时的兜底默认，注释已过时（待用实测值同步更新，见第 7 节）。

---

## 2. 误差来源（PID 现在用模拟量加权，不再是二值离散）

PID 分支的误差**不再**用 `sensor_position_from()`（二值离散阶梯值），而是改用 **`sensor_position_analog_from(vals)`**（`track_module.cpp:517`，2026-07-29 起）：

- 每路先算"在线权重" `w = (BLACK_REF − 原始值) / (BLACK_REF − WHITE_REF)`，压线中心 w≈1.0、完全没压到线 w≈0.0，**中间是连续比例**（`sensor_module.cpp:94-108`）。
- 再按权重加权平均各路物理索引，归一化到 **−1.0 ~ +1.0**：中心 `center_index = (8−1)/2 = 3.5`，符号反转对应物理反装。
- **正 = 线偏右**（需向右修正：左轮加速 / 右轮减速）；setpoint = 0（居中）。
- 丢线判定：权重总和 `< 0.3` 视为丢线返回 NAN（0.3 是起调参考未实车验证）。

**为什么改**：传感器实测是从约 2000 慢慢变到约 600 的**连续过程**，不是一下子跳变。旧的 `is_white[]` 二值判断把它压成"9 档离散阶梯"，Kd 对阶梯信号求导会在跳档瞬间炸出尖峰。模拟量加权更贴近真实物理量，微分才有意义。

> 注意：`sensor_position_from()`（二值版）**仍在用**——给 bang-bang 的 lost/cross 判定和调试打印用（`sensor_module.cpp:67-68`）。只有 PID 的误差改成了 analog 版。

---

## 3. PID 计算（track_module.cpp:511-555）

### 3.1 固定周期重算，与 loop() 解耦
```
if (_pid_last_ms == 0 || now - _pid_last_ms >= PID_INTERVAL_MS) {   // PID_INTERVAL_MS = 10
    dt = (now - _pid_last_ms)/1000            // 恒定 ~10ms
    _pid_integral += error*dt;  clamp(±2.0)
    raw_derivative = (error - _pid_last_error)/dt
    _pid_filtered_deriv += 0.3*(raw_derivative - _pid_filtered_deriv)   // 一阶低通
    output = Kp*error + Ki*integral + Kd*_pid_filtered_deriv
    _pid_hold_output = lroundf(output)         // 四舍五入，不截断
}
```
- **PID 只每 10ms 重算一次**，中间的 loop 沿用 `_pid_hold_output`——避免在 ~1ms 尺度上对信号频繁求导/频繁改 PWM 放大噪声。`dt` 因此恒定。
- **积分限幅 ±2.0**（抗积分饱和）。
- **微分一阶低通滤波 α=0.3**：把跳档尖峰摊到后面几个周期，避免单点打满。
- **输出用 `lroundf()` 四舍五入**，不是向零截断，小误差不会被吃成死区。

### 3.2 ⭐外轮封顶在 base（关键结构改动，见 `../问题/02_PID外轮boost加速冲弯.md`）
```
int pl = base + _pid_hold_output;
int pr = base - _pid_hold_output;
if (pl > base) pl = base;   // 外轮封顶：不加速
if (pr > base) pr = base;
pwm_l = clamp_pwm(pl);      // 内轮仍可为负(反转)
pwm_r = clamp_pwm(pr);
```
- **原来是对称差速 `base ± output`**：弯道 output 大时，外轮会被顶到 **2 倍多 base**（log28 实测飙到 226 ≈ 2.4×base），车一边猛转一边加速冲进弯 → 过冲丢线 → 遇左弯即飞。
- **现在外轮封顶在 base**：修正量**只减慢内轮**（内轮可为负=反转），外轮最高只到 base，绝不加速冲弯。
- 只改 PID 分支；bang-bang 分支（下面 else）完全不动。
- **这条是让 PID 能跑过急弯的前提**——不封顶，纯 PID 一遇急弯就冲飞（log28 → log29 从"2s 冲飞"变成"跑 80s"）。

### 3.3 默认增益 & 复位
```
PID_KP_DEFAULT = 40.0    PID_KI_DEFAULT = 0.0    PID_KD_DEFAULT = 5.0
PID_INTEGRAL_CLAMP = 2.0   PID_INTERVAL_MS = 10   PID_DERIV_FILTER_ALPHA = 0.3
```
- 均为起调参考值。`pid kp/ki/kd N` 在线调，`save` 持久化到 `/config.json`；也可用算法管理器存成命名档。
- **实测最优基线（≠ 默认）见 log31**：`kp80 / ki0 / kd20 / 外轮封顶`（LOST 18.5%，见 `../日志分析/汇总.md`）——机理是 kp80 温和稳态 + 高 kd20 进弯预判强反转 pivot + 外轮封顶保证只转不冲。
- `pid_reset()` 清空 integral/filtered_deriv/hold_output/last_error；调用时机：切换算法、`track_set(on/off)`、**进入丢线状态**（避免带着丢线期间误差重新压线冲一把，`track_module.cpp:495`）。
- `PID_INTERVAL_MS` / `PID_DERIV_FILTER_ALPHA` 目前是编译期常量，没做成运行时命令。

---

## 4. 与 bang-bang 共用的异常状态判定（8路）

lost / cross 两个判定两种算法共用，不受 `_algo` 影响，都基于二值 `is_white[]`：

- **丢线（lost）**：8 路全黑（`track_module.cpp:473-474`）。延续 `_last_dir` 方向找线，`sharp_turn = base*_sharp_ratio` 反转找线，1.5s 超时（`LOST_TIMEOUT_MS`）停车。丢线期间 `pid_reset()`。
- **十字/宽线（cross）**：要求 **`far_left`(CH7 或 CH8) 且 `far_right`(CH1 或 CH2)** 同时压线才算（`track_module.cpp:478-480`），直行穿过、不触发转向、不更新 `_last_dir`。旧写法分界切在 CH4/CH5 之间会把居中的 CH4+CH5 误判成十字（见 `../日志分析/log解析答疑.md` Q4）；收紧后整趟 CROSS 只触发几次（都是真横杠）。

只有"有白线、不丢线、不是十字"这个正常跟踪分支，才按 `_algo` 分岔到 PID 或 bang-bang。

---

## 5. loop 频率与卡顿（已修复）

- `loop()` 无 `delay()`，每圈：`bt_connected` → `cmd_poll` → `track_update`（内含一次 8 路 `analogRead`，只读一次，二值和 analog 加权都用这份 `vals[]`，`track_module.cpp:455-460`）。粗估一圈约 1ms 量级。
- **历史 bug（已修）**：早期 `print_module` 的 flash 环形 log 用 `seek()` 往大文件随机偏移写，导致 **loop 每隔约 1~2 秒被卡 50~211ms**，控制回路冻结、车盲跑（log24 用 `lograte 10` 抓到）。已改为顺序追加/去掉 `out()` 的持续 flash 写，**log25 实测卡顿完全消失**（心跳全 10~11ms、0 大间隔）。详见 `../问题/01_loop周期性卡顿(flash文件日志).md`。
- **PWM 限速（slew）**：`SLEW_RATE_DEFAULT = 800`（单位/秒），把档位/误差跳档时的硬跳变摊平；`slewrate` 命令可调，越大越接近不限速。PID 和 bang-bang 都受益。

---

## 6. 历史记录：5路时代的"车身晃动"排查（已过时，仅存档）

以下是升级到 8 路 + 模拟量加权 + 外轮封顶**之前**的分析，**结论不代表当前代码行为**，重新调参前按上文最新实现重新观察：

- **log4（5路, Kp40/Ki0/Kd5, 旧对称差速）**：比例项正常但增益偏小（打满误差修正量仅 ±40 占 base ~39%）；丢线频繁（~30%）；微分对离散阶梯求导放大噪声，出现满幅 `L:-255 R:255` 尖峰；本质是"披着 PID 外壳的弱 P 控制器"。
- **晃动排查**：`Kd=0` 纯 P 依然晃 → 推翻"Kd 是唯一原因"，怀疑 ① 转向权威不足+车速偏快的画龙过冲 ② 传感器阈值抖动高频反跳。这两点催生了后来的**固定周期 + 微分低通 + 模拟量加权**三项改动。
- 当时的 loop 计时也受第 5 节那个 flash 多秒卡顿干扰，log 数据不完全可信。

**这些问题在 8 路重写后大多已从计算层面缓解**：模拟量加权消除了离散跳档、外轮封顶消除了冲弯过冲、卡顿修复消除了盲跑。当前的调参重点已转移到 kp/kd 甜点（见 `../日志分析/汇总.md`）。

---

## 7. 风险与待办

- **源码默认值待同步**：8 路已实车标定（运行值在 flash `/config.json`），但源码 `sensor_module.cpp` 里 CH1/CH7/CH8 的 THRESHOLD/WHITE_REF/BLACK_REF 仍是 `占位未校准` 默认值——建议把实测值回填到源码默认，让"无 config 时的兜底"也正确、注释不再误导。
- 默认增益（40/0/5）未实车验证；实测较优的是 kp80/kd20（log31），但 LOST 18.5% 仍未到个位数，还需继续压。
- `PID_INTERVAL_MS` / `PID_DERIV_FILTER_ALPHA` / analog 丢线阈值 0.3 均为编译期猜测值，未做成运行时命令、未实车验证。
- Ki 全程 0，积分项尚未真正投入使用。

---

相关：`../问题/02_PID外轮boost加速冲弯.md`(外轮封顶)、`../问题/01_loop周期性卡顿(flash文件日志).md`、`../日志分析/汇总.md`(调参汇总)、`../日志分析/log28分析.md`~`log31分析.md`、`8路传感器方案.md`
