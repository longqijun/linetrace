# 系统设计文档

记录硬件/软件设计决策、变更原因和风险评估。按主题分节，持续更新。

---

## R70急弯：外轮同步降速

**日期：** 2026-07-19

**背景：** 机械改造（传感器挪到轮轴50mm处）后实测：车速稍快，急弯直接冲出赛道；车速调到更慢，电机因静摩擦死区直接不走了——"能动"和"能过弯"这两个速度区间几乎没有重叠（详细分析见下方"软件模块设计"里track_module的问题记录）。

**根因：** 原来的急转逻辑（`SHARP_L`/`SHARP_R`）只反转内轮，外轮仍是和直行一样的`base`满速，车身带着直道的动能冲进急弯，转向力矩来不及在冲出赛道前把车头掰过来。

**改动：** 给急弯新增一个独立的"外轮速度比例"参数`_turn_outer_ratio`（track_module.cpp），触发`SHARP_L`/`SHARP_R`时外轮也降到`base * ratio`而不是维持满速，默认65%，运行时可调（不用重新烧录）：
- 新增BT/USB命令 `turnspeed N`（N=0~100，百分比），对应`track_set_turn_ratio()`
- 纳入`config_module`持久化（JSON新增`turn_ratio`字段），`save`命令一并保存，开机自动加载
- `speed`档位继续控制直道基准速度（解决"死区"问题的那个量），`turnspeed`只影响急弯瞬间的外轮（解决"过弯冲出去"的那个量），两者解耦，不用再用同一个全局速度硬扛两个矛盾的约束

**待办：**
- 实车测试`turnspeed`从65%开始调，观察R70能否通过、是否还会冲出去
- 若外轮降速仍不够，可能还需要"丢线冲出后自动倒车找线"这个后续方案（已讨论，未实现）

---

## 参数调优记录：sharpratio -30%→-40%

**日期：** 2026-07-22

**背景：** `log分析/log-2.txt`（自建赛道，46.6秒连续巡线，详见`log分析/log-2-分析报告.txt`）
分析发现两次丢线时长逼近`LOST_TIMEOUT_MS`(1500ms)上限的情况，均为`SHARP_R→LOST_R`
（右转弯后长时间找不到线）：T1107117~1108497约1380ms（余量120ms）、
T1122087~1123407约1320ms（余量180ms）。虽然两次都在超时前重新压回线上没有真正
触发`LOST_STOP`，但余量偏薄，属于隐患而非已确认的失败。

为此新增了`sharpratio`命令（对应`_sharp_ratio`，原来是硬编码`TURN_RATIO_SHARP`
常量，现在跟`turnspeed`一样运行时可调+持久化，见"软件模块设计"里track_module的
相关记录），目的是加大急弯内轮反转力度，让车头更快甩过弯，缩短丢线时长。

**改动（本轮只改了一个参数）：**

调整前（log-2.txt实测时的参数，`sharp_ratio`为代码默认值-0.3，未曾调过）：
```json
{"speed":16,"turn_ratio":0.65,"sharp_ratio":-0.3,"threshold":[1546,1580,1422,1400,1500]}
```

调整后（实车执行`sharpratio 40` + `save`）：
```json
{"speed":16,"turn_ratio":0.65,"sharp_ratio":-0.4,"threshold":[1546,1580,1422,1400,1500]}
```

只有`sharp_ratio`从-0.3变成-0.4（急弯内轮反转力度从30%加大到40%），
`speed`/`turn_ratio`/5路阈值均未变。

**效果：** 用户实车（仍是自建赛道）测试反馈"效果还可以"——这是定性判断，
目前没有配套的新log文件做量化对比，无法确认之前那两处逼近超时的LOST_R具体
缩短了多少。

**待办：**
- 如需量化验证，按跟log-2同样的方式（`print bt on` + `track on`）跑一遍并导出
  日志（存成`log分析/log-3.txt`），可以直接对比同样两个弯（右转SHARP_R→LOST_R）
  这次丢线时长是否比log-2的1320~1380ms明显缩短，以及有没有引入新问题
  （比如内轮反转力度加大后是否出现打滑、左转是否受影响）
- 仍未在公司比赛赛道实测，这轮调参的结论只在自建赛道上成立

---

## 新增PID算法，与bangbang并列可切换

**日期：** 2026-07-22

**背景：** track_module原本只有一种算法——基于5路离散白/黑状态、按优先级判定的三级
差速bang-bang控制（详见"软件模块设计"里track_module的算法确认与流程图）。用户要求
增加一个PID算法作为可选项，默认仍是bangbang，可通过参数切换，方便对比哪种更平稳。

**设计取舍：**

1. **误差从哪来**：`sensor_module.cpp`本来就有一个连续加权位置`sensor_position()`
   （-1.0~+1.0），但之前只用于`print on`的调试打印，`track_update()`控制路径完全
   没用到它。PID正好需要这个连续误差量，直接复用。
2. **避免重复采样**：`sensor_position()`内部会自己调一次`sensor_binary()`重新读
   ADC，如果PID模式直接调它，就会在`track_update()`一个loop周期里对5路传感器采样
   两次（开头`sensor_binary(is_white)`一次，PID分支`sensor_position()`又一次），
   跟log-1分析报告里发现的"打印代码多次独立采样导致log自相矛盾"是同一类问题，还会
   拖慢控制周期。为此把`sensor_position()`拆成了一个纯函数`sensor_position_from
   (is_white[])`，PID分支直接传入`track_update()`开头已经读好的那份`is_white[]`，
   只采样一次。
3. **算法切换点**：`lost`（丢线）和`cross`（十字路口/宽线）两个判定继续用原来的
   离散布尔状态、两种算法共用，不区分bangbang/pid——这两个是"异常状态处理"，没有
   必要为PID单独做一套，丢线时依然延续`_last_dir`方向反转找线（沿用bang-bang的
   `sharp_turn`），超时1.5s停车逻辑也不变。只有"五路里有白线、不丢线也不是十字
   路口"这个正常跟踪分支，才按`_algo`分岔成bangbang优先级判定 或 PID闭环两条路。
4. **PID输出直接是PWM差速修正量**，不做归一化：`output = Kp*error + Ki*积分 +
   Kd*微分`，`pwm_l = base + output`、`pwm_r = base - output`（error>0=线偏右
   →左轮加速/右轮减速），最后clamp到±255防止溢出。这样Kp的量纲直接是"每单位误差
   对应多少PWM"，跟bang-bang里`sharp_turn`/`mild_turn`的PWM直觉一致，方便调参时
   类比（比如error=1时Kp*1应该跟`sharp_turn`的量级差不多）。
5. **积分限幅+状态重置**：积分项限幅±2.0防止长时间小误差累积失控（抗积分饱和）；
   `track_set(on)`开关巡线、`track_set_algo()`切换算法、进入`lost`状态时都会清空
   PID的积分/微分历史，避免带着丢线期间或上一次运行的陈旧误差数据继续算，冲一把。

**参数（均运行时可调，纳入config_module持久化）：**
- `algo bangbang` / `algo pid`：切换算法，默认bangbang
- `pid kp N` / `pid ki N` / `pid kd N`：浮点数，默认Kp=40.0 / Ki=0.0 / Kd=5.0

**风险与待办：**
- 三个PID默认增益（Kp40/Ki0/Kd5）和积分限幅(±2.0)都只是起调参考值，**完全没有
  实车验证过**，跟当年`TURN_RATIO_SHARP`/`TURN_OUTER_RATIO_DEFAULT`第一次给默认值
  时一样，需要实车从这个起点开始试调，不能直接当作可用参数
- 还没和bangbang在同一条赛道上做过对比测试，不知道PID实际是否比bang-bang更平稳，
  这正是用户想通过新增这个算法来验证的问题，待实测
- Kd项对传感器噪声敏感（丢线边缘、阈值抖动都会在error上产生突变，微分会放大），
  如果实车测试发现车身抖动明显，优先怀疑Kd偏大

---

## 电机供电改造：升压至7V

**日期：** 2026-07-19

**改动：** 电机侧供电由单18650（3.7V）经升压模块稳压升到7V，直接给DRV8833 VM供电，目的是让电机性能变强。

**硬件参数：**

供电：单18650 → 升压模块 → 稳定7V → DRV8833 VM

电机：Tamiya 70168 双电机减速齿轮箱套件，电机型号为马夫奇FA-130
- 额定电压：1.5~3V（说明书标称工作电压3V，原装用2节AA电池）
- 空载电流：约0.15A（3V时）
- 堵转电流：约2.1A（3V时）

驱动：DRV8833
- VM工作电压范围：2.7~10.8V（TI规格书）
- 单通道额定电流：1.5A（连续）/ 2A（峰值，需良好散热铜皮才能达到）

**风险评估：**

7V约为电机额定电压的2.3倍，DRV8833本身电气上没问题（7V在2.7~10.8V安全区间内），但对电机是明显超压，有两个风险：

1. 电机寿命/发热：转速远超设计值，电刷/换向器磨损加快，绕组发热明显增加，长期连续跑容易烧线圈或电刷积碳打火。不会立刻炸，但寿命会大幅缩短。

2. 堵转电流可能超过DRV8833额定（更值得关注）：堵转电流大致正比于电压，3V时~2.1A，粗略换算到7V可能到4~5A，超过DRV8833单通道1.5A（连续）/2A（峰值）的额定值。track_module.cpp里急弯（SHARP_L/SHARP_R）会让内轮反转到-30%，这个瞬间是类堵转/急反转工况，正是电流冲击最大的时刻，也是最容易让DRV8833过流保护跳闸或过热损坏的场景。

**建议/待办：**
- 实测急转弯瞬间的电流（或卡住电机测堵转电流），确认是否接近或超过DRV8833的1.5~2A
- 如果PCB上DRV8833周围没有足够散热铜皮，2A峰值也未必扛得住，容易热保护自动断
- 折中方案：软件上把`motor_level_to_pwm`（motor_module.cpp）的PWM上限压低（比如封顶到180左右而不是255），相当于把最大有效电压限制在~5V左右，比原3V方案更有劲，又不会像满7V那样激进
- 电机摸起来烫手是过热信号，是电机寿命在倒计时的直接证据

**状态：** 已改造为7V供电，尚未实测电流，尚未加软件PWM上限保护。

---

## 软件模块设计

代码位于 `sensor_debug/`，Arduino项目，主程序 `sensor_debug.ino` + 7个功能模块（每个模块一对 `.h`/`.cpp`）。模块间以C函数接口通信，无全局状态共享（各模块内部用`static`变量封装私有状态）。

### 模块依赖关系

```mermaid
graph TD
  ino[sensor_debug.ino<br/>主程序]
  bt[bt_module<br/>蓝牙SPP通信]
  print[print_module<br/>输出通道控制]
  sensor[sensor_module<br/>5路传感器]
  motor[motor_module<br/>DRV8833电机驱动]
  config[config_module<br/>LittleFS配置持久化]
  track[track_module<br/>自动巡线]
  cmd[cmd_module<br/>命令行]

  ino --> bt
  ino --> print
  ino --> sensor
  ino --> motor
  ino --> config
  ino --> track
  ino --> cmd

  print --> bt
  config --> sensor
  config --> bt
  track --> sensor
  track --> motor
  track --> config
  track --> print
  cmd --> bt
  cmd --> motor
  cmd --> print
  cmd --> config
  cmd --> track
  cmd --> sensor
```

依赖是单向的，没有循环依赖。`sensor_module`/`motor_module`/`bt_module`是最底层，不依赖任何其他自定义模块；`print_module`只依赖`bt_module`（把BT作为一个输出通道）；`config_module`/`track_module`依赖底层模块做实际读写；`cmd_module`是最上层，几乎依赖所有模块，因为命令行要能操控整个系统。

### 各模块职责

**bt_module** — 蓝牙SPP通信封装
- 接口：`bt_begin(name)` / `bt_connected()` / `bt_send(msg)` / `bt_poll_line(buf, maxlen)`
- 内部维护一个64字节行缓冲区`_line_buf`，`bt_poll_line`做字符级读取，处理退格（0x08/0x7F）和回车换行，实现带回显的行读取，收到完整行返回true
- 是`print_module`和`cmd_module`共同依赖的底层通道

**print_module** — 数据流输出开关（区别于命令响应）
- 接口：`print_begin()` / `print_set_usb(bool)` / `print_set_bt(bool)` / `out(msg)` / `out_usb(msg)` / `out_bt(msg)`
- 内部两个bool开关`_usb`/`_bt`，默认都是false（开机不输出，需要`print on`手动开启）
- 用途：给高频/大量的调试数据流（传感器读数、track调试log）加开关，避免刷屏；命令响应走`cmd_module`自己的`reply()`，不经过这里，所以命令始终可见

**sensor_module** — 5路红外传感器
- 接口：`sensor_begin()` / `sensor_read(values[])` / `sensor_binary(is_white[])` / `sensor_position()` / `sensor_get_threshold(i)` / `sensor_set_threshold(i,v)`
- 引脚固定映射 `PINS[5] = {33,34,35,36,39}`（CH2~CH6），12位ADC分辨率
- 每路独立阈值`THRESHOLD[5]`，white=低于阈值（NPN光电晶体管active-low特性），阈值可被`config_module`加载的json覆盖，也可被`threshold`命令在线调整
- `sensor_position()`：只统计压白的传感器索引均值，归一化到-1~+1，全黑（丢线）返回NAN；因传感器物理反装180°，这里做了符号取反以保持-1仍代表物理最左

**motor_module** — DRV8833双路电机驱动
- 接口：`motor_begin()` / `motor_stop()` / `motor_brake()` / `motor_set(pwm_l, pwm_r)` / `motor_level_to_pwm(level)`
- 4根PWM引脚（IN1~IN4）+ 1根使能引脚EEP，每侧电机用两根同极性引脚实现正反转（一根给PWM另一根拉低=正转，反过来=反转）
- `motor_level_to_pwm`：1~40档线性映射到0~255 PWM占空比，是`config_module`存的"速度档位"和实际PWM之间的唯一换算点
- 不感知电压（3.7V/7V切换对这一层透明），纯粹是占空比计算

**config_module** — LittleFS + JSON 配置持久化
- 接口：`config_begin()` / `config_get_speed()` / `config_set_speed(level)` / `config_save()` / `config_print()`
- 持久化四类数据：速度档位（int，自己持有）+ 急弯外轮比例（float，通过`track_module`的get/set读写）+ 急弯内轮反转比例（float，通过`track_module`的get/set读写）+ 5路阈值（数组，通过`sensor_module`的get/set threshold读写），阈值和这两个比例自己都不存副本，只做json↔模块内变量的搬运
- 开机`config_begin()`挂载LittleFS、读`/config.json`，文件不存在或字段缺失则保留默认值（速度12，阈值用sensor_module内置默认）
- `config_save()`是唯一写入Flash的入口，`config_set_speed`/`sensor_set_threshold`只改内存，需要显式`save`命令持久化

**track_module** — 自动巡线（三级差速转向，非PID）
- 接口：`track_begin()` / `track_set(bool)` / `track_is_on()` / `track_update()` / `track_get_turn_ratio()` / `track_set_turn_ratio(float)`
- 核心状态机在`track_update()`，每次loop调用一次，非阻塞：
  - 读5路二值状态，映射为`sharp_l/mild_l/center/mild_r/sharp_r`（因传感器反装，index 0↔CH2对应物理右侧，index 4↔CH6对应物理左侧）
  - 判定优先级：丢线(lost) > 十字路口/宽线(cross) > 急转(sharp) > 缓转(mild) > 直行(center/默认)
  - 差速通过修改单侧PWM实现：缓转内轮减速50%（`TURN_RATIO_MILD`，固定），急转内轮反转到`_sharp_ratio`（默认-30%，运行时可调，见`sharpratio`命令）+ 外轮同步降速到`_turn_outer_ratio`（默认65%，运行时可调，见`turnspeed`命令），应对R70急弯——外轮全速会带着直道动能冲进弯道，同步降速减少入弯动能
  - 丢线后用`_last_dir`延续上次转向方向找线，超过`LOST_TIMEOUT_MS`(1.5s)仍找不到则停车
  - 内置调试log（`DEBUG_INTERVAL_MS`节流），走`out()`所以受`print_module`开关控制
- 依赖`config_module`取当前速度档位算基准PWM，依赖`motor_module`下发实际PWM；`_turn_outer_ratio`/`_sharp_ratio`均被`config_module`读写用于持久化（对称于`sensor_module`的阈值模式）

**cmd_module** — 命令行（USB+BT统一处理）
- 接口：`cmd_begin()` / `cmd_poll()`
- 内部各自维护一份行缓冲区（BT走`bt_poll_line`，USB自己实现了一份等价的`serial_poll_line`，两者逻辑重复但故意不合并成一个模块，因为Serial和BluetoothSerial是不同的Arduino对象类型）
- `handle_command()`是一个大的`if-else`字符串匹配分发器，命令集：print/stop/track/speed/turnspeed/save/config/threshold/go/back/spin left/spin right/help
- 响应通过`reply()`直接走`Serial.print`+`bt_send`，不经过`print_module`的开关，所以命令交互始终可见，不受"数据流开关"影响
- `go`/`back`/`spin`系列命令内部用`delay()`阻塞N秒后停车——这几个是唯一会阻塞主循环的命令，用于手动测试电机/巡线以外的动作，测试期间`cmd_poll`/`track_update`都不会执行

**sensor_debug.ino（主程序）**
- `setup()`：关闭欠压检测 → 初始化LED/按钮引脚 → 按顺序`print_begin → sensor_begin → motor_begin → config_begin → track_begin → bt_begin` → 打印一次`config_print()` → `cmd_begin()`
- `loop()`非阻塞轮询顺序：LED状态同步蓝牙连接 → `cmd_poll()`处理命令 → Boot按钮消抖切换track on/off → `track_update()` → 每200ms打印一次传感器原始数据（受`print_module`控制）
- Boot按钮（GPIO0）是唯一不经过命令行、直接切换巡线开关的物理入口

### track_module 算法确认与详细流程图

**日期：** 2026-07-21

> **2026-07-22更新：** 本节写于PID模式加入之前，当时track_module确实只有
> bang-bang一种算法。现在track_module新增了PID作为并列可选算法（默认仍是
> bangbang），详见下方"新增PID算法，与bangbang并列可切换"一节。本节下面的
> 结论、流程图、模式表**只描述bangbang算法这一条路径**，历史信息仍然准确，
> 不需要重画，但读的时候要知道现在多了一个平行的PID分支不在这张图里。

**算法类型确认（针对bangbang算法）：** 基于优先级判定的有限状态机（FSM）+ 三级差速 bang-bang 控制，**不是PID**。

- `sensor_module.cpp` 计算的连续加权位置 `sensor_position()`（-1~+1）**只用于 `print on` 调试打印**，`track_update()` 的实际转向决策完全不使用这个值，走的是5路各自独立的白/黑二值状态（`sensor_binary()`），所以控制路径上不存在PID需要的"连续误差量"。
- 转向输出是离散的几档（直行/缓转50%内轮减速/急转30%内轮反转+外轮同步降速），跳变式切档而非连续比例调节，是bang-bang控制的特征。
- 决策走固定优先级判定树：**丢线 > 十字路口/宽线 > 急转 > 缓转 > 直行**，逐条`if-else`短路判断，不是误差反馈闭环运算。

**完整流程图（对应 `track_update()`，每次 `loop()` 调用一次，非阻塞）：**

```mermaid
flowchart TD
  start([track_update 被调用]) --> onCheck{track是否开启?}
  onCheck -- 否 --> ret1([直接返回])
  onCheck -- 是 --> readSensor[sensor_binary读取5路二值状态]

  readSensor --> mapIdx["映射为 sharp_l/mild_l/center/mild_r/sharp_r<br/>（因传感器180°反装，index0↔CH2=物理右，index4↔CH6=物理左）"]

  mapIdx --> calcState["计算 lost = 五路全黑<br/>计算 cross = 左侧任一压线 AND 右侧任一压线"]

  calcState --> calcPwm["base = speed档位→PWM<br/>mild_turn = base×0.5<br/>sharp_turn = base×(-0.3)<br/>初始 pwm_l = pwm_r = base"]

  calcPwm --> lostQ{lost?}

  lostQ -- 是 --> lostSinceQ{lost_since==0?}
  lostSinceQ -- 是 --> setLostSince[记录 lost_since = now]
  lostSinceQ -- 否 --> timeoutQ
  setLostSince --> timeoutQ{now - lost_since > 1.5s?}
  timeoutQ -- 是 --> lostStop["LOST_STOP<br/>pwm_l=0 pwm_r=0（停车）"]
  timeoutQ -- 否 --> dirQ{_last_dir?}
  dirQ -- "< 0（上次左转）" --> lostL["LOST_L<br/>pwm_l=sharp_turn（延续左转找线）<br/>pwm_r=base"]
  dirQ -- "> 0（上次右转）" --> lostR["LOST_R<br/>pwm_r=sharp_turn（延续右转找线）<br/>pwm_l=base"]
  dirQ -- "== 0（无记录，从未转过）" --> lostBlind["⚠边界情况：无分支命中<br/>保持初始值 pwm_l=pwm_r=base<br/>盲目直行直到超时"]

  lostQ -- 否 --> crossQ{cross?}
  crossQ -- 是 --> crossMode["CROSS：清零lost_since<br/>不改pwm（直行穿过）<br/>不更新_last_dir"]

  crossQ -- 否 --> clearLost[清零 lost_since] --> sharpLQ{sharp_l?}
  sharpLQ -- 是 --> sharpL["SHARP_L（CH6压线，急转）<br/>pwm_l = sharp_turn（内轮反转-30%）<br/>pwm_r = base×turn_outer_ratio（外轮降速，默认65%）<br/>_last_dir = -1"]

  sharpLQ -- 否 --> mildLQ{mild_l?}
  mildLQ -- 是 --> mildL["LEFT（CH5压线，缓转）<br/>pwm_l = mild_turn（内轮减速50%）<br/>_last_dir = -1"]

  mildLQ -- 否 --> sharpRQ{sharp_r?}
  sharpRQ -- 是 --> sharpR["SHARP_R（CH2压线，急转）<br/>pwm_r = sharp_turn（内轮反转-30%）<br/>pwm_l = base×turn_outer_ratio（外轮降速）<br/>_last_dir = +1"]

  sharpRQ -- 否 --> mildRQ{mild_r?}
  mildRQ -- 是 --> mildR["RIGHT（CH3压线，缓转）<br/>pwm_r = mild_turn（内轮减速50%）<br/>_last_dir = +1"]

  mildRQ -- 否 --> straight["STRAIGHT（仅CH4压线或全白）<br/>pwm_l = pwm_r = base<br/>_last_dir 不变"]

  lostStop --> motorOut
  lostL --> motorOut
  lostR --> motorOut
  lostBlind --> motorOut
  crossMode --> motorOut
  sharpL --> motorOut
  mildL --> motorOut
  sharpR --> motorOut
  mildR --> motorOut
  straight --> motorOut

  motorOut[motor_set 下发 pwm_l/pwm_r] --> dbgQ{距上次调试log ≥30ms?}
  dbgQ -- 是 --> dbgLog["打印: 时间戳 + 5路W/B图案(左→右) + mode + 实际PWM"]
  dbgQ -- 否 --> ret2([本次update结束])
  dbgLog --> ret2
```

**9种运行模式一览表：**

| 模式 | 触发条件 | pwm_l | pwm_r | 更新_last_dir |
|---|---|---|---|---|
| STRAIGHT | 仅CH4压线或全白 | base | base | 否 |
| LEFT | CH5压线（缓转） | base×0.5 | base | 是→-1 |
| RIGHT | CH3压线（缓转） | base | base×0.5 | 是→+1 |
| SHARP_L | CH6压线（急转） | base×(-0.3) | base×outer_ratio | 是→-1 |
| SHARP_R | CH2压线（急转） | base×outer_ratio | base×(-0.3) | 是→+1 |
| CROSS | 左右同时压线（十字/宽线） | base | base | 否 |
| LOST_L | 全黑+上次左转 | base×(-0.3) | base | 否 |
| LOST_R | 全黑+上次右转 | base | base×(-0.3) | 否 |
| LOST_STOP | 全黑超过1.5s | 0 | 0 | 否 |

**发现的边界情况（非bug，仅记录）：** 开机后如果还没转过弯（`_last_dir==0`）就直接丢线（比如起点没对准线），`lost`分支里`dirQ`的两个条件都不满足，代码不会进入任何`if/else if`分支，`pwm_l/pwm_r`保持函数开头的初始值`base`（直行）——车会盲目直行直到1.5s超时才停，而不是像正常丢线那样带方向找线。实际场景中影响很小（开局对准线即可避免），但流程图里如实标出这个分支缺口。

### 关键设计约定

1. **两套输出通道，语义不同**：`print_module`控制的是"数据流"（高频、可关闭，如传感器读数、track调试log），`cmd_module`的`reply()`是"命令响应"（始终可见，不受开关影响）。新增输出前要想清楚属于哪一类。
2. **配置写入分两步**：所有`set`类接口（`config_set_speed`、`sensor_set_threshold`）只改内存，必须显式调用`config_save()`才落盘到`/config.json`，避免频繁写Flash。
3. **物理左右映射在两处独立处理**：`sensor_module.cpp`的`sensor_position()`和`track_module.cpp`的index分配，两处都因传感器180°反装做了各自的符号/顺序调整，修改任一处时要同时检查另一处是否也需要同步（`README.txt`开头有详细记录）。
4. **速度档位是唯一的电机强度旋钮**：所有驱动电机的命令（`go`/`back`/`spin`/`track`）都通过`config_get_speed()` + `motor_level_to_pwm()`换算PWM，没有独立的每命令速度参数，改速度只需要改一处。
