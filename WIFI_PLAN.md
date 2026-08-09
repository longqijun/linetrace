# WiFi打印log方案评估

**状态：评估中，尚未实施。** 本文档只是方案设计和可行性评估，代码还没有开始写。
背景：BT一直不稳定（连不上的根因还没查出来），想看看用WiFi代替/补充BT做log传输
是否可行，如果flash空间不够，删掉BT也可以接受。

---

## 1. 现状（当前代码里已经有什么）

- **通信方式：** 只有BT SPP（`BluetoothSerial`库），封装在`bt_module.h/cpp`，
  接口`bt_begin(name)` / `bt_connected()` / `bt_send(msg)` / `bt_poll_line(buf,maxlen)`
- **当前完全没有用到WiFi**，项目里没有`#include <WiFi.h>`
- **其他已用的库：** `LittleFS`（config_module持久化+track.log文件log用）、
  `ArduinoJson`（config_module）
- **输出通道架构：** `print_module`统一管理USB/BT/文件三个通道，`out(msg)`一次
  分发到所有开启的通道；`cmd_module`分别从`bt_poll_line()`和自己实现的
  `serial_poll_line()`读命令，走同一个`handle_command()`处理

这个架构对"加一种新通道"非常友好——`bt_module`的接口设计本身就是一套可以直接
照抄的模板，只要新模块实现同样的四个函数签名，`print_module`/`cmd_module`改动
量都很小。

---

## 2. 可行性评估

### 硬件/协议栈层面：WiFi和BT能不能同时开

**可以，但要留意。** ESP32经典款（这个项目用的型号）WiFi和经典蓝牙（Classic BT，
也就是`BluetoothSerial`用的那套）共用同一颗2.4GHz射频，芯片内部靠ESP-IDF/Arduino
core自动做"时分复用"（coexistence），SDK层面已经处理好了，不需要自己写调度代码，
两者理论上可以同时跑。实际项目里"WiFi+BT同时开"是很常见的组合，不算冷门用法。

代价：
- 两个协议栈同时初始化会占用更多RAM（WiFi运行时堆内存开销大致40~70KB量级，
  BT栈本身也要30~50KB量级，ESP32经典款总共约320KB SRAM，两个一起开通常够用，
  但会明显挤占本来就不算宽裕的堆空间——本项目目前对内存的使用都很轻量
  （`_file_buf`才1KB这种量级），本身占用不大，所以理论上有空间同时跑，
  但**没有实测验证过**，需要实际编译烧录后看实际可用堆内存）
- 射频时分复用意味着WiFi和BT同时高负载传输时会互相"抢时间片"，吞吐量/延迟会
  比单独开一个差一些——但本项目场景是"低频文本log"，不是视频流之类的高带宽
  应用，这点影响预计可以忽略

### Flash占用层面：加WiFi会不会放不下

这是**最需要实测才能确定的部分，我这边没法给出精确数字**，原因：
- 本环境没有安装`arduino-cli`，没法直接编译出实际固件大小给你看
- Flash实际可用空间取决于两个你需要在Arduino IDE里确认的设置：
  1. **开发板实际Flash芯片大小**（常见4MB，也有2MB/8MB/16MB的型号，看具体是
     哪块板子/哪颗芯片，需要看板子丝印或者用`esptool.py flash_id`查）
  2. **Tools菜单里选的Partition Scheme**（分区方案决定了app可用空间，从
     "Default"约1.2~1.3MB、到"No OTA (large APP)"约1.9MB、"Huge APP"约3MB不等）

已知的大概量级（基于ESP32 Arduino core的经验值，仅供参考，不是精确编译结果）：
- `BluetoothSerial`本身就是ESP32库里偏大的一个，经验上编译进去大概占几百KB
- `WiFi.h`（底层是lwIP协议栈）同样是几百KB量级
- `LittleFS` + `ArduinoJson`相对较小

粗略估计WiFi+BT+LittleFS+ArduinoJson一起编译，成品固件大概率落在1.2~1.6MB这个
区间，**如果当前Partition Scheme选的是较小的方案（比如Default约1.3MB上限），
加上WiFi后有一定概率会超**；如果本来就选了较大方案（No OTA/Huge APP），大概率
够用。

**建议的第一步（不用改代码，现在就能做）：** 用Arduino IDE把当前代码（还没加
WiFi）编译一次，看输出窗口最后那行"Sketch uses XXXXX bytes (XX%) of program
storage space"——这个百分比就是现在的flash占用率，如果已经超过70~80%，加WiFi
大概率会放不下，需要先考虑换更大的Partition Scheme或者直接走"删BT换WiFi"这条
路；如果现在占用率还比较低（比如低于50%），加WiFi同时保留BT问题应该不大。

### 供电层面

WiFi发射功率比BT高不少，活跃收发时电流可以到80~180mA量级的峰值（具体看发射
功率和距离），比BT典型工作电流明显更高。本项目是18650双电池方案，电机走独立
电池、ESP32+传感器走另一块电池供电——如果WiFi只是短时间用来`log dump`/偶尔
连接查看状态，这点额外功耗影响不大；如果打算让WiFi像BT现在这样"整场比赛全程
保持连接"，需要额外评估电池续航是否够，这个目前没有实测数据。

---

## 3. 两个方向的选择

### 方向A：WiFi和BT并存（新增第三种通信方式）

- 优点：BT作为已经跑通的手机蓝牙串口助手方案继续保留，WiFi作为"备用/补充"，
  两边都能用，互相不影响使用习惯
- 缺点：flash/RAM都要多担一份BT的开销，前面说的"放不下"的风险主要来自这个方向；
  代码里两套通信逻辑都要维护

### 方向B：WiFi完全替代BT（删掉BluetoothSerial）

- 优点：省下`BluetoothSerial`那部分flash/RAM开销，射频也不用应付两个协议栈的
  时分复用，代码维护也只有一套通信逻辑
- 缺点：**如果BT不稳定的根因本身跟WiFi无关（比如是电源纹波、天线、干扰源
  之类的硬件问题），换成WiFi不一定能解决问题**，只是换了一种可能同样受硬件
  问题影响的通信方式；另外如果比赛现场对WiFi/2.4GHz有额外限制或干扰源比较多
  （人多设备多的比赛现场很常见），WiFi也不是绝对比BT稳定，需要留意

### 建议

**先做第2节里"现在就能做"的flash占用率检查**，再决定选A还是B——如果占用率
本来就低，没必要为了省空间牺牲BT这个已经用惯的通道，直接选方向A更稳妥；如果
占用率已经偏高，方向B可能是唯一现实的选项。

另外，BT不稳定的根因目前还没查出来（`sensor_debug.ino`加的BT连接诊断log还
没收集到有效数据），**在没搞清楚BT为什么不稳定之前就直接删掉它，会丢失掉
"对比诊断"这个手段**——如果保留BT一段时间、同时加上WiFi做对照，之后哪个稳定
用哪个，可能是更稳妥的过渡方式，而不是一步到位删BT。这个取舍最终还是要看
flash空间够不够，够的话建议先按方向A过渡。

---

## 4. 架构设计（如果决定要做）

沿用项目现有的"每种通信方式一个模块，接口对齐bt_module"的模式，新增
`wifi_module.h/cpp`：

```
void wifi_begin(const char* ap_ssid, const char* ap_password);
                                    // 初始化，ESP32开自己的WiFi热点（AP模式）
bool wifi_connected();             // 是否有客户端连上
void wifi_send(const char* msg);   // 发送字符串给已连接的客户端
bool wifi_poll_line(char* buf, int maxlen);
                                    // 带回显的行读取，收到完整行返回true
```

接口签名跟`bt_module`完全对齐，这样`print_module`/`cmd_module`的改动可以
照抄BT那部分的写法：

- `print_module`新增第四个通道`_wifi`（或者如果走方向B替代BT，直接把`_bt`
  改名/替换成`_wifi`，通道数量不变）
- `cmd_module`的`cmd_poll()`里新增一次`wifi_poll_line()`调用，跟现在的
  `bt_poll_line()`/`serial_poll_line()`并列
- `handle_command()`不用改，本来就是跟通道无关的纯字符串分发逻辑

### AP模式 vs STA模式

**建议选AP模式**（ESP32自己开WiFi热点，手机/电脑直接连这个热点，不依赖比赛
现场有没有可用WiFi网络）：

- 优点：跟BT现在的使用体验一致——不依赖外部基础设施，ESP32上电就能连，IP地址
  固定（ESP32 SoftAP模式默认网关`192.168.4.1`），比赛现场没有WiFi路由器也能用
- 缺点：默认没有加密的话，现场任何人都能连上你的车（如果在意这点，AP可以设
  密码，`WiFi.softAP(ssid, password)`一行的事）

如果选STA模式（连接现场已有WiFi网络），优点是可能可以用WiFi网络本身的路由，
理论上传输范围更大；但依赖现场有WiFi、且需要知道SSID/密码、IP地址会随网络
DHCP分配而变化，不如AP模式简单可靠，不建议作为比赛现场用的主方案。

### 传输协议

**建议用最简单的原始TCP socket**（`WiFiServer`/`WiFiClient`），文本行协议，
跟现在BT SPP的使用体验完全一样——用手机装个TCP终端类的App（或者电脑用
`nc`/`telnet`连`192.168.4.1:端口号`），敲命令、看回显，跟现在连BT串口助手
体验一致。不建议一开始就做HTTP/WebSocket这类更重的协议，复杂度不成比例，
现阶段的需求（看log、敲命令）用最简单的TCP文本协议就够。

---

## 5. 实施步骤（尚未开始，仅供后续参考）

1. 检查当前flash占用率（Arduino IDE编译后看输出百分比）——**这是唯一一步
   现在不改代码就能做的**
2. 根据占用率结果，决定方向A（并存）还是方向B（替代）
3. 写`wifi_module.h/cpp`，AP模式+TCP文本行协议，接口对齐`bt_module`
4. `print_module`加`_wifi`通道（或替换`_bt`）
5. `cmd_module`加`wifi_poll_line()`调用点
6. 实车测试：确认WiFi AP能连、命令能敲、log能看到，如果方向A（并存），
   还要测BT+WiFi同时开会不会互相影响
7. 如果测试稳定，再考虑要不要正式删掉BT（方向B）

---

## 6. 风险汇总

- **不确定项：** flash实际占用率、WiFi+BT并存的实测RAM/稳定性，这两项都需要
  用户自己编译/实测才能确定，我这边没有硬件也没有编译环境，给不出精确数字
- **BT不稳定根因未知**：换WiFi不保证能解决问题，如果根因是硬件层面的（供电/
  天线/干扰），WiFi也可能遇到类似问题
- **供电**：WiFi比BT更耗电，长时间保持连接需要评估电池续航影响，目前没有
  实测数据
- **安全性**：AP模式默认无密码的话现场任何设备都能连上，需要的话加个AP密码
