# 02 · PID 外轮被 output 顶上天，加速冲进弯 → 过冲丢线

| | |
|---|---|
| 发现时间 | 2026-08-10 |
| 来源 | log28（PID 首测：~2s 遇左弯即飞、一直打转） |
| 严重度 | 高（纯 PID 一遇急弯就飞，无法跑弯道） |
| 状态 | **方案A已改代码、编译通过，待烧录/实车验证**（2026-08-10；只动 PID 分支，BB 未碰） |

## 现象
PID（speed15, kp80, ki0, kd15, slew5000, base≈96）跑弯道：前 ~2s 直行段跟得很好（err 小、PWM 90/100），一遇**左弯**就冲出去、在左侧一直打转找线 ~9s，最终 LOST_STOP。详见 `../日志分析/log28分析.md`。

## 根因
PID 输出下发是**对称**的：
```c
pwm_l = clamp_pwm(base + _pid_hold_output);
pwm_r = clamp_pwm(base - _pid_hold_output);
```
左弯时 err 大负、output 大负 → **外轮(右) = base − output = 96 −(−130) = 226 ≈ 2.4×base**（逼近上限 255）。
- 车**一边猛左转、一边把前进速度拉到 2 倍多** → 动能巨大 → **直接冲过左弯半径** → 线扫出左边缘 → 丢线。
- `kd=15` 火上浇油：output −130 里 P 项仅 −52，其余 −78 是 err 快速变化时的**微分尖峰**，把外轮顶得更高。
- 对比 bang-bang：外轮**最多 base、急弯还 turn_ratio 降速**，**绝不加速冲弯**。PID 这个对称公式恰好相反——**越要转，外轮越快**。这就是 PID 一个弯就飞、BB 能跑几十秒的直接区别。

## 修改方案

### 核心约束：只动 PID，绝不碰 BB
所有改动**只在 `track_update()` 的 PID 分支内**（`else if (_algo == TRACK_ALGO_PID) { ... }`）。
BB 分支（`else { ... }` 那段各档判定）**一行都不改** → 从代码结构上就不可能影响 BB 算法。

### 方案 A（推荐，核心）：PID 外轮封顶在 base —— 只"减内轮"、不"加外轮"
把 PID 分支末尾两行改成：
```c
// 只让修正量"减慢内轮"（可到反转），外轮不超过 base，绝不加速冲进弯（对齐 bang-bang 的外轮≤base）
int l = base + _pid_hold_output;
int r = base - _pid_hold_output;
if (l > base) l = base;      // 外轮封顶
if (r > base) r = base;      // 外轮封顶
pwm_l = clamp_pwm(l);        // 内轮仍可为负(反转)，clamp 到 -255~255
pwm_r = clamp_pwm(r);
mode_code = 'P';
```
- **效果**：转弯时只有内轮被减速/反转，外轮最多 base（不再飙到 2 倍）→ 不再加速冲弯 → 不过冲。
- **代价**：单侧转向，转向率约为原来一半（差速 = base − 内轮，而非 2×output）。要更狠靠**提 kp 让内轮反转**（见"配合调参"）。
- 改动**极小、纯 PID 分支内**，BB 完全不受影响。

### 方案 B（可选，配合）：按 |err| 降 base（PID 版弯道降速）
若封顶后仍偏快，再叠加"弯越大整车越慢"：
```c
float ae = fabsf(error);
int base_dyn = base - (int)(_pid_corner_slow * base * ae);   // _pid_corner_slow 0~1, 新增PID专用参数
if (base_dyn < base_min) base_dyn = base_min;                 // 防失速地板(可复用minpwm思路)
int l = base_dyn + _pid_hold_output;  int r = base_dyn - _pid_hold_output;
if (l > base_dyn) l = base_dyn;  if (r > base_dyn) r = base_dyn;
pwm_l = clamp_pwm(l);  pwm_r = clamp_pwm(r);
```
- 需新增一个**PID 专用**参数 `_pid_corner_slow`（不进 BB，可放算法档案的 PID 字段/或全局），及命令 `cornerslow N`。
- 先只上方案 A 验证；不够再加方案 B。

### 配合调参（不是代码）
- 封顶外轮后，内轮在 err=1 时 = base − kp。要像 BB 发卡那样反转内轮，需 **kp > base**（base96 → kp≈120~140 内轮才到负）。
- 建议顺序：先上方案 A → speed 12、kp 80、kd 6 重试（先确认不再飞）→ 再把 kp 提到能过发卡。
- `kd` 从 15 降到 5~8，减少微分尖峰。

## 涉及文件
- `sensor_debug/track_module.cpp`：仅 `track_update()` 的 PID 分支末尾两行（方案 A）；方案 B 另加一个 PID 专用参数+命令（config/cmd）。
- **不涉及** BB 分支、cross/LOST 判定、算法管理器等。

## 验证
PID 重跑一趟，看：① 外轮 PWM 不再超过 base（log 里 pwm 不再出现 200+）；② 左弯不再冲飞、不再一直打转；③ BB 档照跑（回归确认没被影响）。

## 相关
- `../日志分析/log28分析.md`（现象与解码）
- `../日志分析/log16分析.md`（PID 直线本来很稳）
- `../设计/BANGBANG算法修改方案.md`（BB 的外轮≤base + 降速思路，正是 PID 要对齐的）
