# 01 · loop 周期性卡顿（疑似 flash 文件日志 flush）

| | |
|---|---|
| 发现时间 | 2026-08-10 |
| 来源 | log24（首次用 `lograte=10ms` 高频采样） |
| 严重度 | 高（控制回路周期性冻结，影响所有巡线调参的稳定性/可复现性） |
| 状态 | **已改代码，待编译/实车验证**（2026-08-10 按根因去掉了 loop 里的持续 flash 写） |

## 现象
用 10ms 采样抓的 log24 里，心跳行 dt 绝大多数是 10~11ms（正常），但混着 **~36 次 45~211ms 的大间隔**（本该 10ms），最大 **211ms**，平均**约每 1~2 秒一次**（不严格周期）。
- 判定为**真·loop 卡顿**：两条连续行之间本该有 6~7 条 10ms 心跳却一条没有 → 那几十~两百毫秒 loop 没执行。
- **直行(STRAIGHT)时也卡** → 与车在做什么无关，是外部/周期性原因阻塞了 Arduino loop。
- 以前 500ms 心跳完全看不见（一次 200ms 卡顿藏在两条心跳之间），是 10ms 采样才暴露出来。

## 影响
卡顿期间**整个控制回路冻结**：不读传感器、不重算、不更新电机。车**带上一刻的 PWM 盲跑 50~211ms**，足以在弯道冲过头/丢线。这会让任何巡线参数调优的结果都不稳定、不可复现——**优先级高于继续调参**。

## 排查过程
- **排除 WiFi/BT**：`grep -i wifi|bt` 全项目无匹配；`.ino` 里还主动 `esp_bt_controller_mem_release()` 释放 BT 内存、`bt_module.cpp` 是空 stub。→ 不是 WiFi/BT。
- **排除按钮**：真按 Boot 键会 `track_set` 切换（track off），下一次 track on 时 log 会重新计。而 log24 是连续一段、没重启 → 不是按钮按下（GPIO0 干净，无毛刺）。

## 根因（强嫌疑）：`print file`（flash 文件日志）在 loop 里周期性 flush
`print_module.cpp` 的 `out()` 除了 `out_usb/out_bt`，还调 **`out_file()`**（`print file on` 控制，写 `/track.log.*` flash 文件）。而 `loop()`（`sensor_debug.ino`）里**每圈都调 `out()`**：
- **LOOP 统计块**：每 **1 秒** `out(stat_buf)`（≈96 字节）。
- **200ms 传感器块**：每 **200ms** `out(buf)`（≈176 字节）。

若 `file_log` 开着：这些字节堆进 `_file_buf`（**仅 512 字节**），满了 → `file_flush()` → **写 flash**。ESP32 上一次 flash program/erase 会**阻塞 CPU 几十~上百毫秒**。按 ~1KB/秒写入，约**每 0.5~1 秒 flush 一次** → 正好对上"约每 1~2 秒一次、50~211ms、直行也卡"的现象。
- 这是 loop 里**唯一会写 flash 的路径**（`config_save` 只在 `save`、RAM log 只在 track off 落盘），最匹配。

## 确认方法（待做）
1. `config` 看 **`file_log` 是否 ON**。若是 → 基本坐实。
2. `print file off` 后重跑，看 log 里大间隔是否消失；或临时 `print usb on` 盯每秒的 `LOOP N/s avg=.. max=..us`（`.ino` 已内置此监测），看 `max` 是否从几万 us 掉下来。

## 已采取的修复（2026-08-10）
把 loop 里持续写 flash 的路径去掉，只保留 track off 一次性落盘：
1. **`print_module.cpp`：`out()` 去掉 `out_file()`** —— 现在 `out()` 只走 USB/BT，不再往 flash 缓冲堆、不再周期性 `file_flush()`。loop 里的 LOOP统计/200ms传感器块调用 `out()` 时不再触发 flash 写。
2. **保留**：track off 时 `ram_log_flush_to_file()` **直接**调 `out_file()` 一次性把整趟 RAM log 落盘（不经过 `out()`），完全不受影响——"saved to flash / log dump" 照常。
3. **顺带修一个残留坑**：`ram_log_flush_to_file()` 落盘时会临时 `print_set_file(true)`，原来**没还原**，导致 `_file` 一直粘 true（这可能就是 file 日志"莫名一直开着"的根源）。现改为落完盘还原 (`print_set_file(false)`)。

> 注：`print file` 命令/`file_log` 配置暂保留但已基本失效（out() 不再读它、track off 落盘自管理 `_file`）。BT 已停用、连续文件日志已无意义，后续可考虑彻底移除该命令。

## 待验证
用 `lograte 10` 重跑一趟，确认 log 里 45~211ms 的大间隔消失（或 `print usb on` 看 `LOOP ... max=` 掉回正常）。

## 相关
- `../日志分析/log24分析.md`（异常首次发现，其中"WiFi 猜测"应更正为本条结论）
- `../设计/LOG高频采样方案.md`（10ms 采样正是它把问题照出来的）
- 代码：`sensor_debug/print_module.cpp`(`out/out_file/file_flush`)、`sensor_debug/sensor_debug.ino`(`loop()` 的 LOOP 统计块 + 200ms 块)
