# 主程序→bang-bang算法调用流程

**日期：** 2026-07-31

本文档梳理从`sensor_debug.ino`开机/主循环，到`track_module.cpp`里bang-bang差速分支
真正把PWM下发给电机为止的完整调用链，只覆盖当前默认算法**bang-bang**这一条路径
（`algo=0`）；PID分支（`algo=1`）不在本文档展开，见"设计说明.md"的"track_module PID
算法详解"节和"8路模拟PID算法.md"。硬件背景（8路传感器、BT停用）见"设计说明.md"。

---

## 1. 一图看懂：从开机到电机转动

```mermaid
flowchart TD
  subgraph boot["开机 setup()（sensor_debug.ino:27-63）"]
    A1[关闭欠压检测] --> A2[esp_bt_controller_mem_release<br/>释放BT内存]
    A2 --> A3[pinMode按钮 + Serial.begin]
    A3 --> A4["print_begin → sensor_begin →<br/>motor_begin → config_begin → track_begin"]
    A4 --> A5["config_begin()读/config.json<br/>恢复speed/turn_ratio/medium_ratio/<br/>sharp_ratio/xsharp_ratio/thresh/white/black等"]
    A5 --> A6[ram_log_auto_init<br/>分配RAM日志缓冲区]
    A6 --> A7[cmd_begin]
  end

  subgraph mainloop["主循环 loop()（sensor_debug.ino:65-134），每圈都跑"]
    B1[loop耗时统计] --> B2["cmd_poll()<br/>处理USB/BT命令行"]
    B2 --> B3{Boot按钮<br/>刚按下?}
    B3 -- 是 --> B4["track_set(!track_is_on())<br/>切换巡线开关"]
    B3 -- 否 --> B5
    B4 --> B5["track_update()<br/>★核心，见下方第2节"]
    B5 --> B6{距上次≥200ms?}
    B6 -- 是 --> B7[打印8路原始ADC+两种加权位置<br/>受print_module开关控制]
    B6 -- 否 --> B1
    B7 --> B1
  end

  A7 --> mainloop
```

`track on`可以由两个入口触发，两者最终都调同一个`track_set(bool)`：
- **命令行**：USB/BT敲`track on`/`track off` → `cmd_module.cpp`的`handle_command()`
  → `track_set(true/false)`
- **物理按钮**：GPIO0(Boot按钮)消抖后 → `sensor_debug.ino:96` → `track_set(!track_is_on())`

`track_set(true)`（`track_module.cpp:205-258`）做的事：清空`_last_dir`/`_lost_since`、
调`pid_reset()`/`slew_reset()`清掉残留状态、调`ram_log_begin()`清空内存日志缓冲区并
写入`TRACK_ON`+`PARAMS`两条标记行（`config_build_params_line()`生成当前生效的全部
参数快照）。`track on`之后，每次`loop()`都会调用一次`track_update()`，这是bang-bang
真正被执行的地方。

---

## 2. `track_update()` 内部详细流程（`track_module.cpp:264-454`）

```mermaid
flowchart TD
  start([track_update被调用]) --> onCheck{"_on == false?<br/>(:265)"}
  onCheck -- 是 --> ret0([直接return，不做任何事])
  onCheck -- 否 --> readAdc["sensor_read(vals)<br/>8路原始ADC，只读一次(:269-270)"]

  readAdc --> toBinary["sensor_binary_from(vals,is_white)<br/>按每路THRESHOLD二值化(:271-272)<br/>纯函数，不重新读ADC，跟PID共用这份vals"]

  toBinary --> mapPhys["物理左右映射(:277-284)<br/>xsharp_l=is_white[7](CH8) sharp_l=is_white[6](CH7)<br/>medium_l=is_white[5](CH6) mild_l=is_white[4](CH5)<br/>mild_r=is_white[3](CH4) medium_r=is_white[2](CH3)<br/>sharp_r=is_white[1](CH2) xsharp_r=is_white[0](CH1)<br/>（180°反装：index0=CH1在物理最右，index7=CH8在物理最左）"]

  mapPhys --> calcFlags["计算lost=8路全黑(:286-287)<br/>计算cross=左半(index4~7)AND右半(index0~3)同时有白(:289-292)"]

  calcFlags --> calcBase["base=motor_level_to_pwm(speed)(:294)<br/>mild_turn=base×0.7 (:295, TURN_RATIO_MILD固定)<br/>medium_turn=base×_medium_ratio (:296, 默认35%)<br/>sharp_turn=base×_sharp_ratio (:297, 默认-30%)<br/>xsharp_turn=base×_xsharp_ratio (:298, 默认-60%)<br/>初始 pwm_l=pwm_r=base (:300-301)"]

  calcBase --> lostQ{"lost? (:307)"}

  lostQ -- 是 --> lostBranch["丢线分支(:307-321)<br/>记lost_since、pid_reset()<br/>超时1.5s→LOST_STOP(停车,stop_now=true)<br/>否则按_last_dir延续方向找线(LOST_L/LOST_R)"]

  lostQ -- 否 --> crossQ{"cross? (:322)"}
  crossQ -- 是 --> crossBranch["CROSS分支(:322-324)<br/>清零lost_since，不改pwm，不更新_last_dir<br/>直行穿过十字路口/宽线"]

  crossQ -- 否 --> algoQ{"_algo == PID? (:325)"}
  algoQ -- 是 --> pidBranch["PID分支(:325-362)<br/>见'track_module PID算法详解'节<br/>本文档不展开"]

  algoQ -- 否 --> bbBranch["★bang-bang分支(:363-404)<br/>见下方第3节'4级差速判定树'"]

  lostBranch --> slewStep
  crossBranch --> slewStep
  pidBranch --> slewStep
  bbBranch --> slewStep

  slewStep{"stop_now?<br/>(:407-415)"}
  slewStep -- 是 --> slewReset["slew_reset()<br/>丢线超时停车不走限速，安全优先"]
  slewStep -- 否 --> slewToward["slew_toward(pwm_l,pwm_r,...)<br/>限速逼近，_slew_rate×dt限制变化量<br/>(track_module.cpp:106-125)"]

  slewReset --> motorOut
  slewToward --> motorOut

  motorOut["motor_set(pwm_l,pwm_r)<br/>(:417，实际下发PWM)"] --> logQ{"档位变化 或<br/>心跳到期(500ms)?<br/>(:422-423)"}

  logQ -- 否 --> ret1([本次update结束])
  logQ -- 是 --> buildLog["拼一行紧凑log:<br/>E/H<dt> <8路位图hex> <modeCode> <L> <R>[<PID误差>]<br/>(:427-449)"]
  buildLog --> outLog["out_usb(rec) + out_bt(rec)<br/>（受print开关控制，BT已停用无实际输出）<br/>+ ram_log_append(rec)（无条件写内存，:450-452）"]
  outLog --> ret1
```

---

## 3. bang-bang分支：8路4级差速判定树（`track_module.cpp:363-404`）

`_algo != TRACK_ALGO_PID`时，且未丢线、未在十字路口，进入这一段：由外到内依次判断，
**命中最外层就不再看内层**（`if / else if`短路），对称的左右各4档：

```mermaid
flowchart TD
  enter(["进入bang-bang分支<br/>(lost=false, cross=false, 未丢线未十字)"]) --> q1{"xsharp_l?<br/>CH8压线"}
  q1 -- 是 --> m7["HAIRPIN_L (modeCode='7')<br/>pwm_l=xsharp_turn (反转,默认-60%)<br/>pwm_r=base×turn_outer_ratio (外轮降速,默认65%)<br/>_last_dir=-1"]

  q1 -- 否 --> q2{"sharp_l?<br/>CH7压线"}
  q2 -- 是 --> m5["SHARP_L (modeCode='5')<br/>pwm_l=sharp_turn (反转,默认-30%)<br/>pwm_r=base×turn_outer_ratio<br/>_last_dir=-1"]

  q2 -- 否 --> q3{"medium_l?<br/>CH6压线"}
  q3 -- 是 --> m3["MEDIUM_L (modeCode='3')<br/>pwm_l=medium_turn (减速不反转,默认35%)<br/>_last_dir=-1"]

  q3 -- 否 --> q4{"mild_l?<br/>CH5压线"}
  q4 -- 是 --> m1["LEFT缓转 (modeCode='1')<br/>pwm_l=mild_turn (减速,固定70%)<br/>_last_dir=-1"]

  q4 -- 否 --> q5{"xsharp_r?<br/>CH1压线"}
  q5 -- 是 --> m8["HAIRPIN_R (modeCode='8')<br/>pwm_r=xsharp_turn<br/>pwm_l=base×turn_outer_ratio<br/>_last_dir=+1"]

  q5 -- 否 --> q6{"sharp_r?<br/>CH2压线"}
  q6 -- 是 --> m6["SHARP_R (modeCode='6')<br/>pwm_r=sharp_turn<br/>pwm_l=base×turn_outer_ratio<br/>_last_dir=+1"]

  q6 -- 否 --> q7{"medium_r?<br/>CH3压线"}
  q7 -- 是 --> m4["MEDIUM_R (modeCode='4')<br/>pwm_r=medium_turn<br/>_last_dir=+1"]

  q7 -- 否 --> q8{"mild_r?<br/>CH4压线"}
  q8 -- 是 --> m2["RIGHT缓转 (modeCode='2')<br/>pwm_r=mild_turn<br/>_last_dir=+1"]

  q8 -- 否 --> m0["STRAIGHT (modeCode='0')<br/>pwm_l=pwm_r=base（初始值不变）<br/>_last_dir不变<br/>（全黑已被lost分支挡掉，走到这里通常是<br/>仅CH4/CH5之间的中心区域或全白）"]
```

**关于"由外到内"的含义**：从最外侧的CH1/CH8（发卡弯）到最内侧的CH4/CH5（缓转）依次
判断，只要外层命中就不再看内层——例如CH8和CH6同时压线时，只会触发`HAIRPIN_L`，`
medium_l`那一档完全不会被看到。这是判定优先级设计的核心：**车头偏得越狠，越需要最
外层最激进的修正**，内层的档位只在外层都没压线时才轮到。

**8路顺序与物理左右对应关系**（因传感器180°反装）：

```
物理左 ← CH8   CH7   CH6   CH5  |  CH4   CH3   CH2   CH1 → 物理右
        (idx7)(idx6)(idx5)(idx4) (idx3)(idx2)(idx1)(idx0)
        发卡弯 急转  中转  缓转     缓转  中转  急转  发卡弯
```

---

## 4. 关键代码位置速查表

| 内容 | 位置 |
|---|---|
| 主程序开机顺序 | `sensor_debug.ino:27-63` |
| 主循环调用顺序 | `sensor_debug.ino:65-134` |
| track on/off触发（按钮） | `sensor_debug.ino:92-101` |
| track on/off触发（命令行） | `cmd_module.cpp`：`"track on"`/`"track off"`分支 |
| `track_set()`实现 | `track_module.cpp:205-258` |
| `track_update()`总入口 | `track_module.cpp:264` |
| 8路ADC采样+二值化（只采样一次） | `track_module.cpp:269-272` |
| 物理左右映射（8路index翻转） | `track_module.cpp:277-284` |
| lost/cross判定 | `track_module.cpp:286-292` |
| base/各档PWM预计算 | `track_module.cpp:294-301` |
| 丢线分支 | `track_module.cpp:307-321` |
| 十字路口分支 | `track_module.cpp:322-324` |
| PID分支（本文档不展开） | `track_module.cpp:325-362` |
| **bang-bang 4级差速分支** | `track_module.cpp:363-404` |
| PWM限速（slew） | `track_module.cpp:106-125`（`slew_toward`实现），`:406-415`（调用点） |
| 电机下发 | `motor_module.cpp:42-59` `motor_set()`；`track_module.cpp:417`调用点 |
| 速度档位→PWM换算 | `motor_module.cpp:11-15` `motor_level_to_pwm()`（1~40线性映射到0~255） |
| 事件/心跳日志拼装与输出 | `track_module.cpp:419-453` |
| 各转向比例默认值 | `track_module.cpp:16-30`（`TURN_RATIO_MILD`固定0.7、`TURN_RATIO_MEDIUM_DEFAULT`=0.35、`TURN_RATIO_SHARP_DEFAULT`=-0.3、`TURN_RATIO_XSHARP_DEFAULT`=-0.6、`TURN_OUTER_RATIO_DEFAULT`=0.65） |
| 8路阈值/白黑参考值 | `sensor_module.cpp:6-19`（`PINS`/`THRESHOLD`/`WHITE_REF`/`BLACK_REF`） |

---

## 5. 一次调用只读一次传感器，两种算法共用同一份采样

`track_update()`开头只调一次`sensor_read(vals)`+`sensor_binary_from(vals,is_white)`
（`:269-272`），后面无论走丢线/十字路口/bang-bang/PID哪个分支，都复用这同一份
`vals[]`/`is_white[]`，不会在一次`loop()`里对8路传感器重复采样——这是2026-07-22
修复"打印代码多次独立采样导致log自相矛盾"问题后延续下来的设计约定（PID分支的
`sensor_position_analog_from(vals)`同样吃这份`vals`，不重新读ADC），改动这段代码时
需要保持这个约定，不要在某个分支里又调一次`sensor_read()`/`sensor_binary()`。

---

## 相关文档

- 8路传感器物理布局、引脚映射、4级差速的设计取舍：**8路传感器方案.md**
- PID分支的详细算法分析：**8路模拟PID算法.md**、设计说明.md的"track_module PID算法详解"节
- PWM限速(`slewrate`)设计背景：设计说明.md的"PWM限速（slewrate），压制左右摇摆"节
- 硬件引脚、命令行完整列表：设计说明.md的"硬件参考"、"代码结构与命令参考"节
