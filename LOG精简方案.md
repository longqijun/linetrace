# LOG精简方案

**状态**：代码已按本文档实施完成（新增`ram_log_module.h/cpp`；改动`track_module.cpp`/`print_module.h/cpp`/`config_module.h/cpp`/`cmd_module.cpp`/`sensor_debug.ino`），**尚未实车验证**。心跳间隔取的是建议区间下限500ms。

**追加变更（2026-07-31再次补充）**：内存缓冲区大小原计划是写死的编译期常量，实测过一轮（64KB时min free heap≈190540字节，据此手动调到128KB）之后，用户反馈这样太麻烦——代码经常改，每次改完可用堆内存都会变，写死的数字就得跟着手动重调。改成**开机时自动定容量**：`setup()`最后调用`ram_log_auto_init()`，读一次`ESP.getFreeHeap()`，留够安全余量（64KB）后一次性`malloc()`剩下的部分（上限192KB封顶，避免异常情况下吃掉过多内存；分配失败会打折重试，最后仍失败就直接判定日志功能关闭，不影响巡线本身）。这次`malloc`只在开机时做一次，之后终生不再malloc/free，不会有堆碎片问题——这跟"LOG精简方案.md"原先建议"用静态数组、不要malloc"的出发点（怕反复分配/释放的堆碎片）并不冲突，只分配一次和写死大小在安全性上是等价的，但省去了每次改代码都要手动改数字的麻烦。开机时会在Serial上打印一行`>>> RAM log buffer: N bytes (free heap was M)`，`mem`命令里也能随时查（`cmd_module.cpp`新增）。

**追加变更（2026-07-31三次补充）**：用户确认BT是否真的完全没占用额外内存。代码层面能确认的是`bt_begin()`全项目只有一处调用点（`sensor_debug.ino`）且被注释掉，`bt_module.cpp`里`_began`标志因此永远是false，其余函数全部短路——运行时不会启动BT协议栈。但`BluetoothSerial.h`确实被链接进了固件，底层SDK是否因此在编译期就预留了一块BT控制器内存（不受"是否调用.begin()"影响），只看源码无法100%确认，需要实机验证。为了不管这块预留是否存在都能拿回来，在`sensor_debug.ino`的`setup()`最开头（早于一切其他初始化）新增了`esp_bt_controller_mem_release(ESP_BT_MODE_BTDM)`调用——这是把BT+BLE控制器内存都释放回通用堆的标准做法，调用之后这次开机周期内BT就不能再启动了（跟本项目"BT已彻底停用"的既有决定一致）。**这处改动没有实际编译验证过**（当前环境没有可用的arduino-cli/platformio工具链），需要用户编译一次确认没有报错，如果目标板的ESP32核心版本对这个API的头文件路径/签名有出入，可能需要微调`#include`。

**追加变更（2026-07-31四次补充）**：用户一开始想通过注释掉`.ino`里的`#include "bt_module.h"`来让BT彻底不编译进去——这个思路有个Arduino编译模型上的误区：sketch文件夹里的`.cpp`文件不管有没有被`#include`都会被编译，光删`.ino`里的include既不能阻止`bt_module.cpp`链接`BluetoothSerial`库，反而会让`cmd_module.cpp`/`print_module.cpp`/`sensor_debug.ino`里现有的`bt_send()`等调用变成编译错误。真正的做法是把`bt_module.cpp`本身改成空实现（stub）：不再`#include "BluetoothSerial.h"`、不再有`BluetoothSerial`对象，四个函数（`bt_begin`/`bt_connected`/`bt_send`/`bt_poll_line`）签名不变但什么也不做——这样`BluetoothSerial`/Bluedroid host协议栈那部分代码不会被链接进固件，其他调用这几个函数的地方一行都不用改。这跟前面`esp_bt_controller_mem_release()`是两个层面的优化叠加：一个管controller层的内存池，一个管host协议栈代码要不要被链接，加起来才是"BT真的一点都不占用"。同样没有实际编译验证过，需要用户编译确认。以后要恢复BT，除了取消`bt_begin("LineTrace")`的注释，还要把`bt_module.cpp`换回真正调`BluetoothSerial`的实现（查git历史），并删掉`esp_bt_controller_mem_release()`那一行。

**追加变更（2026-07-31五次补充，重要修正）**：用户实机跑出`Free heap was 327892`但`RAM log buffer`只分到`100842`字节，一开始看起来像"算法浪费了两百多KB"。查了一下`ram_log_auto_init()`原来的重试轨迹（196608→172032→150528→131712→115248→100843，每次`malloc()`失败打8折重试），确认了根因：**ESP32上`ESP.getFreeHeap()`（所有零散空闲块总和）和"`malloc()`一次能要到的最大连续块"是两回事**，这次实测最大连续块大概只有100~115KB，中间那两百多KB空闲是碎片，本来就要不到，不是判断逻辑选错了留白比例。

真正的修正是**换判断基准**：从`ESP.getFreeHeap()`改成`ESP.getMaxAllocHeap()`（ESP32 Arduino核心自带的API，直接返回"最大可一次性分配的连续块"），按用户提的思路重写：
- `max_alloc`超过阈值（100KB）：只固定留一个绝对余量（32KB），其余全部给日志用
- `max_alloc`不超过阈值：改成只拿一个百分比（50%），不把能分配到的最大块几乎占满

换成这个基准之后，`malloc()`正常情况下第一次就该成功（不用再像之前那样靠5次失败重试才摸到真实上限），原来的重试循环保留作为兜底（防止`setup()`后续代码在两次调用之间碰巧又抢占了一部分内存这种边缘情况），不再是主要依赖的手段。开机打印和`mem`命令现在都会同时显示`free heap`和`max alloc`两个数字，方便以后一眼看出这两者是否有明显差距。这处改动同样没有实际编译验证过，需要用户编译烧录后用`mem`确认新分配到的数字比之前更合理（预期这次应该能拿到接近`max_alloc - 32KB`的量级，而不是又被打好几次折）。

**追加变更（2026-07-31六次补充，改成分块分配）**：实机验证`ESP.getMaxAllocHeap()`方案后，`max_alloc`只有110580字节（留32KB余量后分到77812字节），跟总空闲堆327892差距依然很大——说明碎片化程度比预期更严重，"找一整块连续内存"这个思路本身就在ESP32这块板子上吃亏。用户提出改成**分块分配**：把一次大`malloc()`换成很多份20KB的小`malloc()`，一份一份分配，直到剩余空闲堆逼近安全余量（100KB）为止；这些小chunk落盘时按分配顺序拼成一份完整log，`track_module.cpp`等上层调用方感知不到底下是分块存的。

`ram_log_module.h/cpp`已重写：
- `_chunks[]`（`char*`数组）+ `_chunk_used[]`（每份已写字节数）+ `_chunk_count`（实际分到几份）+ `_cur_chunk`（当前写到第几份）
- `ram_log_auto_init()`循环`malloc(20KB)`，每次分配前检查"分完这份剩余空闲堆会不会跌破100KB"，跌破就停；某一份分配失败也停（数组容量上限16份=320KB，只是保险丝，不是目标）
- `ram_log_append()`：当前chunk装不下这一行就整体挪到下一份chunk（不把一行拆开跨chunk写，这样每份chunk内部按行扫描就够了）；所有chunk都写满了才静默丢弃
- `ram_log_flush_to_file()`：按chunk顺序（0..当前写到的那份）逐份、每份内部逐行扫描，跟之前单块设计一样地喂给`out_file()`

这样理论上能拿到的总容量应该比"一次性大malloc"高不少，更接近"总空闲堆-100KB余量"这个理想值（因为20KB粒度的碎片远比100KB+粒度的连续块好找）。同样没有实际编译验证过，需要用户编译烧录后看开机那行`>>> RAM log buffer: N bytes in M x 20KB chunks (free heap before X, after Y)`，确认分到的份数和总字节数是不是比之前的77812明显高。

**追加变更（2026-07-31七次补充，flash落盘先停用）**：用户反馈现阶段不想要flash这一层了，希望`log dump`直接从内存打印。改动：

- `ram_log_module.h/cpp`新增`ram_log_dump()`：不经过flash，直接把当前内存里的内容（按chunk顺序、原样`Serial.write()`）打印出来，随时可调用——包括track还在跑、10秒窗口还没到、还没执行track off的时候，看到的就是"目前为止已经捕捉到的内容"。开头会先打印一次`print_log_legend()`（modeCode编码表），dump出来的行不再带`#ID`前缀（内存里本来就是按写入顺序连续存放，不像flash分段环形缓冲区那样需要靠`#ID`才能判断时间顺序，直接顺序输出就是时间顺序）
- `track_module.cpp`的`track_set(false)`不再调用`ram_log_flush_to_file()`：track off只是停电机、往内存里补一行`TRACK_OFF`标记，然后直接`Serial.print`一行`>>> Captured N lines, type 'log dump' to view`。**内存里这一趟的内容不会被清空**，一直留到下一次`track on`调用`ram_log_begin()`才会被清空覆盖——这意味着`log dump`能看到的永远是"最近这一次track on到现在（或者到下次track on之前）"的内容，不再是"flash里累积的多趟历史"
- `cmd_module.cpp`的`log dump`命令改成调用`ram_log_dump()`，不再调用`print_file_dump()`；help文本同步
- **`ram_log_flush_to_file()`/`print_file_dump()`等flash相关函数保留在代码里，只是没有任何地方调用了**（不是删掉，是停用）——如果以后又想要"多趟测试累积在flash里、事后一次性dump"这个能力，把`track_set(false)`里加回`ram_log_flush_to_file()`的调用、`log dump`改回调`print_file_dump()`就行，不用重新实现
- **副作用**：`log clear`/`log status`这两个命令还是对着flash那一套状态操作的，现在flash没人写了，`log status`会一直显示"OFF"、`log clear`清的也是一份不会再变化的空文件——这两个命令目前处于"能跑但没什么意义"的状态，暂时没有改，如果以后确认不需要flash了可以考虑一起清理

这处改动同样没有实际编译验证过，需要用户编译烧录后测一遍：`track on`跑一会儿，不用等track off，直接敲`log dump`应该就能看到内容；`track off`之后再`log dump`，内容应该还在，不受影响。

**追加变更（2026-07-31八次补充，flash恢复成"只留最新一趟"的备份）**：用户希望track off时仍然写一份进flash，但语义改成"flash里永远只保留最新一次的log"，不是之前设计里"多趟测试累积在同一份flash里"那种环形缓冲区语义。改动：

- `ram_log_module.cpp`的`ram_log_flush_to_file()`去掉了结尾清空RAM chunk状态那几行——现在这个函数**只负责把内存内容写进flash，不再清空RAM**，RAM的生命周期完全交给`ram_log_begin()`/10秒窗口自己管，跟要不要落盘无关
- `track_module.cpp`的`track_set(false)`里，在调用`ram_log_flush_to_file()`之前先调用一次`print_file_clear()`（清空所有`/track.log.*`段+`#ID`归零），这样每次落盘前flash都是空的，写完之后flash里就只有这一趟的内容，不会跨多趟累积
- 效果：`log dump`（读RAM）和flash备份（读不到，flash只是静默备份，当前没有命令读它）现在内容是一致的，都是"最近这一趟"；track off之后哪怕立刻断电重启，flash里也留了一份刚才那趟的备份（RAM会丢，flash不会）——弥补了纯RAM方案"没来得及dump就断电就彻底丢失"这个风险
- `log status`/`log clear`这两个命令现在又变得有意义了（之前"flash落盘先停用"那阵子它们空转），不用再改
- 如果以后想从flash里把这份备份读出来（比如真遇到了"忘按track off就断电，RAM丢了"这种情况，想从flash抢救），`print_file_dump()`还在代码里、没删，只是目前没有命令接到它——需要的话加一个`log dumpflash`之类的命令接上就行

同样没有实际编译验证过，需要用户编译烧录后测一遍：track一趟、track off、`log dump`看到内容后，重新上电（不要`log clear`），确认flash里那份备份还在（比如先看`log status`的`next_id`不是0，或者以后加了读flash的命令再验证内容一致）。

**追加变更（2026-07-31九次补充，TRACK_OFF加duration）**：用户要求track off时在log最后记一个"这份log一共覆盖了多长时间"。实现时发现一个隐藏坑：TRACK_ON/PARAMS/TRACK_OFF这几行标记之前都是走跟事件/心跳行一样的`ram_log_append()`，同样受10秒窗口限制——但现实中track off经常发生在track on之后10秒以外（人要先把车捡回来才按得到按键），这意味着**TRACK_OFF这一行很可能一直被悄悄丢弃，log里从来就没有真正落地过**，只是因为之前没人去验证"最后一行是不是真的有TRACK_OFF"才没发现。

修法：把`ram_log_module.cpp`的写入逻辑拆成`append_internal(line, windowed)`共享实现，`windowed=true`（对外暴露成`ram_log_append()`，事件/心跳行用）才检查10秒窗口，`windowed=false`（新增`ram_log_append_marker()`，TRACK_ON/PARAMS/TRACK_OFF用）不检查窗口，只受chunk空间限制——`track_module.cpp`里这三行标记全部从`ram_log_append()`换成了`ram_log_append_marker()`。

duration的计算：新增`_last_append_ms`，只在`windowed=true`的调用里更新（标记行不算），代表"最后一条事件/心跳行相对track on的时间差"，也就是log里真正有数据那一段的时长（最大不超过10秒，跟"track on到track off经过了多久"是两回事，后者可能因为人捡车而远超10秒）。`track_set(false)`里先取一次`ram_log_duration_ms()`和`ram_log_line_count()`，再拼进TRACK_OFF这一行：
```
>>> TRACK_OFF t=<ms> duration=<ms>ms lines=<n>
```
Serial上的即时回显也同步加了这两个数字。

这次改动顺带修了一个之前没被注意到的潜在问题（TRACK_OFF可能从未真正写进log过），一起记在这里。同样没有实际编译验证过，需要用户编译烧录后测一遍：track on之后隔久一点（比如等超过10秒之后）再按track off，`log dump`确认最后一行确实是`TRACK_OFF ... duration=...`，不是缺失或者停在最后一条事件行。

**追加变更（2026-07-31十次补充，E行dt改成"距上次切换"）**：用户指出"上一次切换到这一次切换中间的时间间隔很重要"。查了一下发现`dt`原来记的是"距上一条记录行"的间隔——档位切换够快、中间没插心跳时这两者是一样的（判断摆动频率的典型场景，切换间隔通常远小于500ms心跳间隔），但如果某个档位保持超过500ms、中间插了一条心跳(`H`)行，下一条`E`行的`dt`就会变成"距上一条心跳"而不是"距上一次真正切换"，被心跳打断——信息没丢（把中间H行的dt和这条E行的dt加起来还是能算出真实切换间隔），但不够直接，容易被当成就是切换间隔而读错。

改法：`track_module.cpp`新增`_last_switch_ms`（只在`mode_changed`时更新，跟`_last_log_ms`分开）。现在：
- **`E`行的`dt`＝距上一次真正切换的间隔**，不受中间插入的心跳打断，直接就是"这一档位保持了多久才切到下一档"
- **`H`行的`dt`＝距上一条记录行的间隔**，代表心跳节奏本身，跟切换无关

`print_log_legend()`（`log format`命令/`log dump`开头都会打印）同步更新了这个区别的说明，避免以后翻log时把两种`dt`的语义搞混。

同样没有实际编译验证过，需要用户编译烧录后测一遍：故意造出"某个档位保持超过500ms、中间至少插一条心跳，然后再切到下一档"这个场景（比如放慢速度让车在直道多走一会儿再入弯），确认心跳后面那条切换行的`dt`是"距上次切换"的真实间隔，不是被心跳截断的一小段。

**追加变更（2026-07-31十一次补充，去掉10秒窗口+加真实挂钟时长）**：用户拿手机对log9计时，实测track on~off一共21秒，但`TRACK_OFF`报的`duration`一直是9秒多，一开始以为是bug。查证后确认是设计如此——`duration`原本就被10秒窗口卡住了，只反映"内存里实际捕捉到的数据跨度"，不是"track on到track off经过的真实时间"（后者要用`TRACK_OFF t=`减`TRACK_ON t=`才能算，log9算出来是26504ms，跟手机的21秒对得上、有些误差是按键反应时间）。用户确认两点后续要求：①`TRACK_OFF`直接把真实挂钟时长打印出来，不用自己减；②10秒窗口不要了，改成"内存写到剩余空间不多了（比如1KB）才停止"，不再按时间限制。

改动：
- `ram_log_module.cpp`去掉`RAM_LOG_WINDOW_MS`和相关的时间窗口判断，新增`RAM_LOG_RESERVE_BYTES`（1KB）：事件/心跳行（`ram_log_append()`）写到整个缓冲区剩余空间低于1KB就不再写，把这块余量专门留给收尾的`TRACK_OFF`标记（`ram_log_append_marker()`可以动用这块余量，只受chunk硬容量限制）——这样即使事件数据把缓冲区吃到快满，收尾标记还是能写进去，不会重蹈"标记行被截止条件误伤"的覆辙（跟上次那个10秒窗口误伤TRACK_OFF是同一类问题，只是这次换成了空间维度）
- `track_module.cpp`新增`_track_on_ms`：track on按下时记一次`millis()`，track off时用`millis()-_track_on_ms`算出真实挂钟时长`elapsed`，跟`duration`（事件数据实际覆盖的时长，现在受限于"缓冲区还剩多少"而不是"时间过了多久"）一起打进`TRACK_OFF`这一行：
  ```
  >>> TRACK_OFF t=<ms> elapsed=<ms>ms duration=<ms>ms lines=<n>
  ```
  Serial上track off那一刻的即时回显、`log format`的格式说明都同步加了这两个字段的区别说明
- 效果：正常情况下（缓冲区没写满）`elapsed`和`duration`应该很接近；如果测试跑得很久、事件切换非常频繁导致缓冲区提前写满，`duration`会明显小于`elapsed`——这本身也是有意义的信号（说明这一趟数据量远超预期，可能是摆动/切换特别剧烈，或者车跑的时间特别长）

这次改动同样没有实际编译验证过，需要用户编译烧录后测一遍：正常测一趟，确认`elapsed`跟自己掐表/手机计时的时间基本对得上；`duration`不再固定卡在9点几秒，应该能跟着`elapsed`一起变长（除非中途缓冲区真的写满了）。

**目的**：现在T行按`DEBUG_INTERVAL_MS=30ms`定时打印，一次实车测试2~3分钟就能把256KB的4段flash环形缓冲区绕完一圈（覆盖掉更早的数据），装不下"一次通电测多趟、事后一次性dump对比"的需求。本方案把单条日志的体积和频率都压下来，让同样的flash容量能装下更多趟测试。

**追加约束（2026-07-31补充）**：不管单行多小，只要track运行期间去碰flash（哪怕只是`out_file()`攒够512字节缓冲区就`flush()`一次），就会有一次几百毫秒量级的阻塞，这段时间里`loop()`卡住、电机指令跟着卡住，car反应不过来——这个代价比日志体积本身更不能接受。所以本方案追加一条硬性要求：**track on期间全程不碰flash，只写内存**；用户实测确认track on之后10秒内基本就能看完想看的问题，所以内存缓冲只需要覆盖"这一趟的前10秒"，10秒到了（或者缓冲区提前写满）就停止继续采集，不需要无限大。真正落盘的时机挪到track off那一刻——这时电机已经停了（`track_set(false)`一进来就会`motor_stop()`），落盘哪怕卡两三百毫秒也不影响巡线响应。

---

## 1. 现状与问题

现在写入flash的每一行是：`#ID T <ms> <8字符W/B> <9字符mode串> L:<pwm> R:<pwm>[ E:<err>]`

举例（bang-bang模式，实测约53字节/行，含`#ID `前缀）：
```
#       12 T   1530 BWBBWBBW MEDIUM_L L: -30 R: 102
```

- `#%8lu `固定10字节（右对齐补空格，出自`print_module.cpp`的`out_file()`）
- 8个W/B字符表示8路传感器状态，占8字节
- mode串是`%-9s`左对齐补空格到9字节（如`STRAIGHT `/`HAIRPIN_L`）
- `L:`/`R:`标签各占3字节

`DEBUG_INTERVAL_MS=30ms`，即约33行/秒，53字节/行 ≈ 1.7~2KB/秒。256KB总容量约150秒（2.5分钟）就绕完一圈。而且大部分行内容是重复的——直线段会连续打印几十条一模一样的`STRAIGHT`，真正有信息量的是**档位切换的那一刻**（这恰好也是诊断"摆动幅度/频率"最需要看的东西）。

---

## 2. 设计原则

1. **事件触发，不再定时轮询**：只在`mode`发生变化时才写一行。档位切换越频繁＝摆动越剧烈，这正是要看的信号；直线巡航时不再重复刷屏。
2. **可选心跳**：避免长直道几十秒完全没有输出、事后看log怀疑程序卡死，配一个低频心跳（建议500ms~1s一条），跟事件行共用同一种紧凑格式，用不同前缀区分，纯兜底用，不是本方案的核心。
3. **字段全部紧凑化**：能用1个字符表示的不用9个字符，能用hex表示的不用8个ASCII字符，能省的分隔符/标签全部去掉。
4. **仍然保留实际下发的PWM(L/R)**：不能只记mode，因为slew限速会让实际PWM滞后/跟不上目标值，而这个滞后/超调正是要诊断的东西，不能丢。
5. **TRACK_ON/TRACK_OFF标记保持现状不缩**：一次测试只有开头结尾各一条，体积无所谓；具体内容见第4节（改成纯标记+单独一行参数快照）。
6. **采集只在track on期间进行**：track on时开始收集、track off时结束，不需要用户提前手动开关，也不会在两次测试之间的调参过程中把命令回显之类的噪声一起录进去。
7. **track on期间只写内存，不碰flash**：`out_file()`不管有没有精简过格式，只要在track运行期间调用就会有flush阻塞的风险（哪怕平均只有几十~百来毫秒，见开头"追加约束"）。改成track on期间所有日志行都先追加进一块预先分配好的内存缓冲区，track off时再一次性批量落盘，把flash的阻塞挪到电机已经停下、不再有实时性要求的时间点。

---

## 3. 新行格式

### 3.1 事件行 / 心跳行

```
E<dt> <patHex> <modeCode> <L> <R>[ <err>]\r\n     ← 档位变化时触发
H<dt> <patHex> <modeCode> <L> <R>[ <err>]\r\n     ← 心跳：档位未变，但到了心跳间隔
```

字段说明：

| 字段 | 说明 |
|------|------|
| `E`/`H` | 1字符前缀，区分"这行是因为档位变了"还是"纯心跳" |
| `dt` | 距离上一条记录行（不含TRACK_ON/OFF）的毫秒数，十进制变长，`track_set(true)`时清零参照点（第一条事件行的dt=距TRACK_ON的毫秒数） |
| `patHex` | 8路`is_white[]`拼成的位图（bit7=CH8...bit0=CH1，即`track_module.cpp`里已有的物理左→右顺序），打印成2位大写hex（如`A5`），比8个W/B字符省6字节 |
| `modeCode` | 见下表3.2，1个字符，比如`STRAIGHT`→`0`，省8字节 |
| `L`/`R` | 实际下发PWM，带符号十进制，不带`L:`/`R:`标签 |
| `err` | 仅PID模式附加，`%+.2f`；bang-bang模式没有这个字段 |

### 3.2 modeCode编码表

| 字符 | 含义 | 对应现有mode字符串 |
|------|------|---------------------|
| `0` | 直行 | STRAIGHT |
| `1` | 缓转-左 | LEFT（mild_l，CH5） |
| `2` | 缓转-右 | RIGHT（mild_r，CH4） |
| `3` | 中转-左 | MEDIUM_L（CH6） |
| `4` | 中转-右 | MEDIUM_R（CH3） |
| `5` | 急转-左 | SHARP_L（CH7） |
| `6` | 急转-右 | SHARP_R（CH2） |
| `7` | 发卡-左 | HAIRPIN_L（CH8） |
| `8` | 发卡-右 | HAIRPIN_R（CH1） |
| `9` | 十字/宽线直穿 | CROSS |
| `A` | 丢线-延续左转找线 | LOST_L |
| `B` | 丢线-延续右转找线 | LOST_R |
| `C` | 丢线超时停车 | LOST_STOP |
| `P` | PID闭环（连续误差，不分档） | PID |

### 3.3 `#ID`前缀顺带缩短

现在`out_file()`里是`"#%8lu %s"`，固定10字节右对齐补空格。改成`"#%lu "`变长，不影响解析（仍靠`#`和空格定界，dump仍按段号0..N-1输出、靠ID大小判断时间序——这个机制完全不变），只是ID小的时候能省几个字节。

### 3.4 示例对比

现有一行（53字节，含ID前缀）：
```
#       12 T   1530 BWBBWBBW MEDIUM_L L: -30 R: 102
```

新方案等价内容（约20字节，含ID前缀）：
```
#12 E15 4A 3 -30 102
```
（含义：ID=12，距上条记录15ms，传感器位图0x4A，中转-左，L=-30 R=102）

---

## 4. 采集生命周期：内存缓冲 + track off批量落盘

### 4.1 采集窗口跟track on/off绑定，但"停止采集"和"落盘"是两件解耦的事

实车只有一个物理按键（`sensor_debug.ino`里的`BUTTON_PIN`），按一下`track_set(!track_is_on())`切换一次，第一下开、第二下关。但实际测试流程里，车一旦track on跑起来就会往前冲，人够不到按键，**第二下"关"往往要等车跑完/冲出去停下之后，人过去把车捡回来才按得到**——不是每次都能在"想让它停"的那个精确时刻按到off。所以不能假设"10秒记录窗口关闭"和"按到off"是同一时刻，两者要解耦成两件独立的事：

- **停止采集**（第4.2节）：track on后满10秒，或内存缓冲区提前写满，两者谁先到就自动停止继续记录——这一步全程只是内存里的判断，不碰flash，不需要等off，也没有办法人为提前打断（用户确认过这点：写满/到点之前没有办法直接停）
- **落盘**：停止采集之后，缓冲区里的内容原地留着（不清空、不覆盖），一直等到真正的`track_set(false)`发生（不管是过一会儿人走过去按到按键、还是回来接上USB敲`track off`命令）才触发第4.4节的批量落盘。中间这段"已经停止采集，但还没落盘"的空当可能持续几秒到几分钟不等，这没关系，缓冲区在这段时间里就是安静地待在RAM里，不会有任何额外开销

`out_file()`本身仍然靠`_file`标志位控制要不要真的写盘（这个开关机制不用改），但track on期间完全不会调用到`out_file()`（全部写内存），所以`_file`在track on期间是什么状态无所谓。真正要紧的是**批量落盘那一刻**（第4.4节）：`ram_log_flush_to_file()`内部在开始写之前先确保`_file`为true（如果当时是false就临时置true，写完不用改回去——文件通道保持常开对多趟测试场景更省心），避免用户忘了手动`print set file on`导致辛辛苦苦攒了10秒内存日志、最后却因为文件通道没开而悄悄丢在flush这一步。

### 4.2 内存缓冲区：只覆盖track on后的前10秒

**核心变化**：track on期间产生的每一行日志（`E`/`H`事件行，见第3节），只`memcpy`追加进一块预先分配好的内存缓冲区，不做任何flash操作——纯内存写是微秒级的，不会拖慢`loop()`，不影响电机响应。

停止继续采集的条件是下面两个中先到的那个：
- **track on后满10秒**：用户实测确认，track on之后10秒内基本能看完这一趟想检查的问题，超过10秒的部分不需要再录
- **缓冲区提前写满**：不是bug，是设计如此——如果10秒内产生的日志量小于缓冲区容量，正好覆盖完整10秒；如果因为异常剧烈的抖动导致提前写满，那"提前写满"这件事本身就是有意义的信号（说明档位切换频率远超预期），track本身继续正常跑、正常控制，只是后续不再有新的日志行写进缓冲区

缓冲区大小按"尽量大"来定，但不写死成编译期常量——用户改代码比较频繁，每次改完可用堆内存都会变，写死一个数字就得每次手动重调，太麻烦。改成**开机时自动定容量**（`ram_log_module.cpp`的`ram_log_auto_init()`，`sensor_debug.ino`的`setup()`最后调用，见4.1节追加变更）：

1. 放在其他模块的`_begin()`都跑完之后调用，这时候读到的`ESP.getFreeHeap()`才是"稳态下真实能用的余量"（BT已关闭，可用堆内存比开BT时宽裕不少）
2. 留够安全余量（当前取绝对值64KB，给栈/LittleFS/其他运行时用途留够）后，用`malloc()`**一次性**分配剩下的部分；上限192KB封顶，避免以后代码改动导致某次实测空闲堆异常大时缓冲区跟着无限制地吃掉大部分内存；分配失败就打折重试，最后仍不成功直接判定容量为0——**只在开机时malloc这一次，运行期间不再malloc/free，不会有堆碎片问题**，这一点上跟"只用编译期静态数组"是等价的，只是不用每次手动改数字
3. 分配到的实际字节数会在开机时打印一行`>>> RAM log buffer: N bytes (free heap was M)`，也能随时用`mem`命令查（`cmd_module.cpp`，同时显示free heap和ram log buffer大小）
4. 按第3节的紧凑事件行算，一行约20字节（RAM暂存阶段没带`#ID`前缀，落盘时才统一编号，省几个字节）。之前手动实测过一轮：64KB时`mem`测得min free heap≈190540字节，倒推系统总空闲≈250KB——这个量级下自动分配出来的容量大概率会落在100KB+这个档位（≈5000行以上），覆盖10秒的话平均每秒能扛住500行左右都不丢，远高于正常巡线时估算的每秒7~12行（见第6节），异常抖动到每秒上百行也有很大余量。具体数字以每次开机时Serial打印的那一行或`mem`命令为准，不用再回头翻这份文档里的旧数字

### 4.3 每次track on固定写两行开场（写进内存缓冲区，不是直接写flash）

一次track on不再是一条"TRACK_ON"就把所有参数塞在一行里，而是固定两行：

**第1行——纯标记**，只带时间戳，不带任何参数：
```
>>> TRACK_ON t=<ms>
```

**第2行——当前生效的全部参数快照**，复用`config_module.cpp`里`config_print()`已经在维护的那一整套字段（不用另起一套，`config_print()`目前是直接写USB/BT，这里改成先写进内存缓冲区，落盘时跟其他行一起走）：
```
>>> PARAMS speed=<n> algo=<BANGBANG|PID> turn_ratio=<f> medium_ratio=<f> sharp_ratio=<f> xsharp_ratio=<f> slew_rate=<f> pid_kp=<f> pid_ki=<f> pid_kd=<f> thresh=[<8个int>] white=[<8个int>] black=[<8个int>]
```
具体是照抄JSON格式还是换成更紧凑的`key=value`纯文本，属于实现细节，两种都行——反正一次测试只出现一行，体积不敏感，怎么写以实现时顺手为准，但字段集合要跟`config_print()`保持一致（不要漏掉thresh/white/black这三个数组，之前调试阈值问题时就是靠这些）。

**收尾——TRACK_OFF标记**（也是先写进内存缓冲区，而不是直接落盘）：
```
>>> TRACK_OFF t=<ms>
```

### 4.4 track off时批量落盘

`track_set(false)`的执行顺序：①先`motor_stop()`（现有逻辑，已经是这个顺序）②把TRACK_OFF行追加进内存缓冲区 ③把整块内存缓冲区的内容，按现有`out_file()`的`#ID`编号+分段轮转逻辑，一次性顺序写入flash（相当于把原来"逐行调用`out_file()`"改成"对缓冲区里的每一行循环调用一次`out_file()`"，`#ID`分配、分段切换等下游逻辑完全不变，只是触发时机从"track运行中随时"改成"track off这一刻集中触发"）④清空内存缓冲区，为下一次track on做准备。

这样一次flash log里如果存了"多趟"测试，靠TRACK_ON/PARAMS/TRACK_OFF三行加上各自区间内的`#ID`范围就能完整切分：TRACK_ON定位起点，PARAMS给出这一趟事件行（`E`/`H`）需要的全部解码上下文（比如`patHex`要对照哪一组threshold来理解），TRACK_OFF定位终点、之后没有内容直到下一个TRACK_ON。多趟测试之间只要都做完了各自的track off落盘，flash里积累的历史完全不受影响，跟之前的设计目标一致。

---

## 5. 涉及改动的文件（已实施）

| 文件 | 改动 |
|------|------|
| `print_module.cpp` | `out_file()`里`"#%8lu %s"`改成`"#%lu %s"` |
| `ram_log_module.h/cpp`（新文件） | 内存缓冲区改成`malloc()`动态分配（不是编译期静态数组）：`ram_log_auto_init()`（`setup()`末尾调用一次，读`ESP.getFreeHeap()`留够安全余量后分配，只分配这一次、不再malloc/free）、`ram_log_capacity()`（实际分配到的字节数）、`ram_log_begin()`（track on时调用，清空指针、记录起始时间）、`ram_log_append(const char*)`（纯内存追加，超过10秒/空间不够/分配失败时直接忽略、不报错、不阻塞）、`ram_log_flush_to_file()`（track off时调用：先确保`_file`为true（第4.1节），再把缓冲区内容按行循环喂给现有`out_file()`逻辑一次性批量落盘，最后清空缓冲区） |
| `sensor_debug.ino` | `setup()`里其他`_begin()`都跑完之后，新增一行`ram_log_auto_init()` |
| `track_module.cpp` | `track_update()`末尾的调试log部分重写：判断`mode`是否较上次调用发生变化→触发`E`行；否则判断是否到心跳间隔→触发`H`行；都不满足则本次不写。命中时调用`ram_log_append()`而不是`out_file()`。新增mode→modeCode的映射（简单switch/查表）、8位W/B转2位hex（位运算+`%02X`）、dt计算（`now - _last_log_ms`，`track_set(true)`时连同`_last_dbg`一起清零参照点） |
| `track_module.cpp` | `track_set()`重写：开启时先`ram_log_begin()`、再把TRACK_ON纯标记行、PARAMS参数快照行都喂给`ram_log_append()`；关闭时先`motor_stop()`（已有逻辑不变）、把TRACK_OFF行喂给`ram_log_append()`、最后调用`ram_log_flush_to_file()`完成批量落盘 |
| `config_module.cpp` | `config_print()`目前只走`Serial.print`/`bt_send`，需要拆出一个可复用的"生成参数快照字符串"的部分，供`track_module.cpp`的PARAMS行调用，避免字段定义在两处各写一遍、以后改参数容易漏改一处 |
| `print_module.cpp` | `print_file_dump()`开头的说明行里，顺带打印一次modeCode简表（表3.2），避免脱离这份文档就看不懂dump出来的内容 |
| `cmd_module.cpp` | 新增`log format`命令，随时能在USB上打印一遍表3.2，不用非得翻文档；新增`mem`命令，打印`ESP.getFreeHeap()`/`ESP.getMinFreeHeap()`/`ram_log_capacity()` |

---

## 6. 预期效果（估算，非精确保证）

- **track on期间对实时性零影响**：全程只做内存追加，不再有任何flash操作，原来"写文件几百毫秒延迟、车反应不过来"的问题从根上消失，落盘的一次性阻塞被挪到电机已停的track off时刻
- 假设正常巡线时平均每80~150ms切换一次档位，事件行频率约7~12条/秒，比现在固定33条/秒少了2/3~3/4
- 单行体积从~53字节降到~20字节左右，再打4折
- 内存缓冲区按第4.2节实测调到128KB，覆盖track on后完整10秒，正常工况下有很大余量（约能扛住每秒650行，远高于估算的7~12行/秒），异常抖动导致提前写满也不算失败，本身就是有效信号
- flash总容量（256KB，4段）没变，但因为单条记录更小、且只有真正有信息量的事件才会落盘，同样容量能装下的"趟数"比之前的定时轮询方案更多，具体倍数需要实测校验
- 如果开了心跳，直道上每500ms~1s还有一条兜底记录（先进内存缓冲区，随批量落盘一起写入flash），不会出现"长时间完全没输出、怀疑程序卡死"的空窗

---

## 7. 风险与待办

- dt用变长十进制，理论上"长时间无事件+不开心跳"会让数字变大，但心跳机制兜底后不会出现这种情况；如果最终决定不要心跳，需要考虑dt要不要限位
- modeCode表是本方案新定义的，脱离这份文档看dump内容会看不懂，建议实施时按第5节把简表打印进`log dump`的输出里
- 心跳间隔（本方案建议500ms~1s）、要不要心跳，都还没实车验证，可能需要按实际直道时长调整
- PID模式是连续误差量，没有离散档位可比较，不适合"档位变化触发"。本方案暂不改动PID的打印逻辑（继续用旧的定时节流，或者单独降频），等PID真正进入实车验证阶段再单独设计，不在这次范围内
- 这只是压缩"单条记录"，没有改变256KB总容量本身；如果以后还是不够装多趟测试，可以另外考虑调大`LOG_SEGMENT_MAX_BYTES`/`LOG_SEGMENT_COUNT`（`print_module.cpp`），但那是flash容量维度的事，跟本方案是两个独立的旋钮
- PARAMS行依赖`config_module.cpp`当前的字段集合，以后新增运行时可调参数时容易忘记同步进这一行（现有`config_print()`本身也有这个"改参数要记得同时改打印"的老问题），建议实施时按第5节把两处收敛成一个共用的"生成快照"函数，不要维护两份字段列表
- **内存缓冲区大小已经改成开机自动分配**（见文档开头"追加变更"），不用再手动改数字，但`ram_log_auto_init()`分配时读到的`ESP.getFreeHeap()`只是"开机静置那一刻"的空闲堆，还没有实车跑过track on/off、多趟测试累积之后再测一次——建议正式测试前先按第4.1节流程完整走一轮（track on→10秒→捡回车→track off→`mem`），确认`min free heap`没有比刚烧录时掉得更多，排除运行时另有内存占用没算进去的可能。如果`RAM_LOG_SAFETY_MARGIN_BYTES`(64KB)/`RAM_LOG_MAX_BYTES`(192KB封顶)这两个参数以后发现留得不够/太保守，在`ram_log_module.cpp`顶部改这两个数字就行，不用改分配逻辑本身
- **重要取舍：track on期间如果发生crash/掉电/看门狗复位，这一趟的内存缓冲区会完全丢失**（RAM不掉电保存，来不及落盘）——这跟当初做flash环形缓冲区的初衷（"BT断连/事后也能补一份log"，见`print_module.h`注释）部分冲突，等于牺牲了"crash那一趟的日志"来换取"日常测试时的零延迟响应"。之前`log5分析.md`分析的就是一次真实crash，如果类似情况在新方案下发生，是拿不到那一趟的log的。这是用户明确要的取舍（优先保证实时响应），但实施前需要跟用户确认过一遍这个后果，别等真出现crash时才发现log丢了感到意外
- **同样的道理，不只是crash会丢数据——只有一个物理开关，"忘了/来不及按第二下off"就直接断电，这一趟也会丢**（第4.1节）。停止采集本身不需要off（10秒/写满自动触发），但落盘一定要等到`track_set(false)`真正被调用（按键第二下，或者接上USB敲`track off`）才会发生。这意味着测试流程里必须补一步操作纪律：**车跑完、捡回来之后，先按一下按键（或者接USB发`track off`）把track正式关掉，再断电/关机**，不然刚才那一趟即使完整采集满了10秒内存，也永远进不了flash。建议实施时在这一步加个明显提示（比如按键off时候的Serial回显里顺带打印"已落盘，共N行"之类的确认信息），让用户在现场就能确认这一趟真的存住了，而不是走远了才发现忘按
- 上面这条crash丢日志的问题，如果以后又变得重要，折中方案是"低频周期性落盘"（比如每2~3秒把缓冲区里新增的部分批量flush一次，而不是全部攒到track off），能把丢失窗口从"整个10秒"缩小到"最后2~3秒"，但代价是重新引入几次track on期间的flash阻塞，跟本方案"全程零flash"的目标直接矛盾，所以本方案不采用，只在这里记一笔，供以后如果优先级变化时参考
