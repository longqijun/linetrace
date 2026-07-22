============================
巡线小车项目 - 说明文档
============================

------------------------------------------------------------
★★★【下次打开优先处理】机械结构改造 ★★★
------------------------------------------------------------
  已完成：传感器挪到后驱动轮轴约50mm处（原140mm，轮距仍100mm），
          落在30~50mm的目标余量内。

  改造带来的副作用：传感器安装方向物理反装（180°翻转），左右两侧
  互换（原CH2在左→现在CH2在右，原CH6在右→现在CH6在左）。代码已同步：
    - track_module.cpp：sharp_l/mild_l/mild_r/sharp_r 对应的index已互换
      （CH4居中不受影响）
    - track_module.cpp：调试log打印顺序改为按物理左→右（index 4→0）
    - sensor_module.cpp：sensor_position() 符号取反，-1/+1 含义不变
    - threshold CH VALUE 命令、config.json阈值顺序不受影响（仍按
      CH2~CH6电气接线走，跟装反方向无关）

  还要做：
    1. 传感器离地高度变了，CH2~CH6阈值需要重新校准
       （print on，实测过线看白/黑ADC值，threshold CH VALUE 调整，save持久化）
    2. 用track_module.cpp里已有的调试log（格式 "T <ms> <WB图案> <模式> L: R:"）
       实测R70发卡弯能不能过
    3. 过不去可用BT命令 sharpratio N 现场调整内轮反转比例（0~100%，默认30，运行时可调，不用重新烧录）

------------------------------------------------------------

【硬件平台】
  主控：ESP32（Arduino IDE 开发）
  传感器：AE-NJL5901AR-8CH（8路红外反射传感器）
  电机驱动：DRV8833
  传感器间距：16mm
  比赛线宽：19mm（白线）

------------------------------------------------------------
【引脚定义】（PCB固定，不可修改）
------------------------------------------------------------

传感器（AE-NJL5901AR-8CH）：
  CH1 → D32  (ADC1) 蓝牙可用
  CH2 → D33  (ADC1) 蓝牙可用  ← 使用
  CH3 → D34  (ADC1) 蓝牙可用  ← 使用
  CH4 → D35  (ADC1) 蓝牙可用  ← 使用
  CH5 → VP   (GPIO36, ADC1)   ← 使用
  CH6 → VN   (GPIO39, ADC1)   ← 使用
  CH7 → D13  (ADC2) 蓝牙启用时不可用
  CH8 → D14  (ADC2) 蓝牙启用时不可用
  注：CH7/CH8为ADC2，蓝牙开启时读值出错，禁用

电机驱动（DRV8833）：
  GPIO17 → EEP  使能（HIGH=工作，LOW=睡眠）
  GPIO18 → IN1  右电机
  GPIO19 → IN2  右电机
  GPIO21 → IN3  左电机
  GPIO22 → IN4  左电机

其他：
  GPIO2  → 板载LED（BT已连接=常亮，未连接=常灭，sensor_debug.ino里是digitalWrite直接
                    跟随bt_connected()，不是闪烁；另外开机会打印"BT started: LineTrace"，
                    loop()里每隔1秒打印一次当前BT连接状态："BT connected"或
                    "ERROR: BT not connected"，USB接串口监视器可以看这两处log排查BT连接问题）
  GPIO0  → Boot按钮（按一下切换track on/off，物理开关，见下方巡线说明）

------------------------------------------------------------
【传感器说明】
------------------------------------------------------------
  - 使用中间5路：CH2~CH6，跨度 4×16 = 64mm，左右对称
  - 输出特性：白色=低值，黑色=高值（NPN光电晶体管active-low）
  - 实测阈值（各路独立，代码内默认值，可被/config.json覆盖）：
      CH2(D33):  白~660   黑~2400  代码默认阈值1545（已实测校准，/config.json中为1530）
      CH3(D34):  白~660   黑~2500  阈值1580
      CH4(D35):  白~545   黑~2300  阈值1422
      CH5(VP36): 白~600   黑~2200  阈值1400
      CH6(VN39): 白~600   黑~2400  代码默认阈值1115（已实测校准，/config.json中为1500）
  注：CH2/CH6代码内默认值尚未同步更新，实际跑的是/config.json里save过的1530/1500；
      如需换新设备/清空flash，记得先跑一遍threshold+save，或手动同步sensor_module.cpp默认值
  - 加权位置：-1.0（最左）~ 0.0（居中）~ +1.0（最右），NAN=丢线

------------------------------------------------------------
【电源】
------------------------------------------------------------
  双18650方案：
    电池1 → 升压5V → ESP32 + 传感器
    电池2 → DRV8833 → 电机
  注意：上电瞬间电压抖动，代码中关闭欠压检测器：
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

------------------------------------------------------------
【代码模块结构】
------------------------------------------------------------

  sensor_debug/
  ├── sensor_debug.ino     主程序入口
  ├── bt_module.h/cpp      蓝牙模块
  ├── print_module.h/cpp   输出控制模块
  ├── sensor_module.h/cpp  传感器模块
  ├── motor_module.h/cpp   电机模块
  ├── config_module.h/cpp  配置持久化模块（LittleFS+JSON）
  ├── track_module.h/cpp   自动巡线模块（bangbang/pid双算法，运行时可切换）
  └── cmd_module.h/cpp     命令行模块

------------------------------------------------------------
【模块接口说明】
------------------------------------------------------------

[bt_module] 蓝牙SPP通信
  bt_begin("DeviceName")     初始化，设置蓝牙设备名
  bt_connected()             返回bool，是否已连接
  bt_send(msg)               发送字符串到BT
  bt_poll_line(buf, maxlen)  带回显的行读取，收到完整行返回true

[print_module] 集中输出控制（默认USB和BT均关闭）
  print_begin()              初始化，USB=OFF, BT=OFF
  print_set_usb(bool)        开关USB数据流
  print_set_bt(bool)         开关BT数据流
  out(msg)                   向所有已开启通道输出
  out_usb(msg)               仅USB输出
  out_bt(msg)                仅BT输出

[sensor_module] 5路传感器
  sensor_begin()             初始化ADC
  sensor_read(values[5])     读取原始ADC值
  sensor_binary(is_white[5]) 二值判断，true=白线
  sensor_position()          加权位置 -1.0~+1.0，NAN=丢线（内部重新采样一次）
  sensor_position_from(w[]) 纯函数版，用调用方已采样的is_white[]算位置，不重新读ADC
                             （track_module的PID模式用这个，避免同一loop周期重复采样）
  sensor_get_threshold(i)    获取第i路阈值
  sensor_set_threshold(i,v)  覆盖第i路阈值（仅内存，供config_module加载配置用）

[motor_module] DRV8833电机驱动
  motor_begin()              初始化引脚，使能DRV8833
  motor_stop()               滑行停止（coast）
  motor_brake()              制动停止
  motor_set(pwm_l, pwm_r)   设置左右PWM，-255~255，负值=反转
  motor_level_to_pwm(level)  1~40档转换为PWM值（0~255，原1~10档等比*4扩展分辨率）

[config_module] 配置持久化（LittleFS + JSON，速度档位+急弯外轮比例+急弯内轮反转比例
                +算法选择+PID三个增益+5路传感器阈值）
  config_begin()             挂载LittleFS，从/config.json加载配置（无文件/无字段则用默认值）
  config_get_speed()         获取当前速度档位(1~40)
  config_set_speed(level)    设置当前速度档位（仅内存生效）
  config_save()              把当前所有可调参数写入/config.json
  config_print()             打印当前配置的JSON内容（Serial+BT）
  注：阈值默认值在sensor_module内置，json有threshold字段时才覆盖；
      save时阈值取sensor_module当前值；可用BT命令 threshold CH VALUE 现场调整（仅内存，需save持久化）
  注：急弯外轮比例默认值在track_module内置(0.65)，json有turn_ratio字段时才覆盖；
      可用BT命令 turnspeed N 现场调整（仅内存，需save持久化）
  注：急弯内轮反转比例默认值在track_module内置(-0.3)，json有sharp_ratio字段时才覆盖；
      可用BT命令 sharpratio N 现场调整（仅内存，需save持久化）
  注：算法选择默认值在track_module内置(0=bangbang)，json有algo字段时才覆盖；
      可用BT命令 algo bangbang/pid 现场切换（仅内存，需save持久化）
  注：PID三个增益默认值在track_module内置(Kp40/Ki0/Kd5)，json有pid_kp/pid_ki/pid_kd字段时才覆盖；
      可用BT命令 pid kp/ki/kd N 现场调整（仅内存，需save持久化）

[track_module] 自动巡线，两种算法并列、运行时可切换（默认bangbang）：
  ● bangbang：三级差速，CH4居中，CH3/CH5缓转，CH2/CH6急转
  ● pid：连续加权位置(sensor_position_from)作误差，PID闭环差速转向
  track_begin()              初始化，默认关闭，默认算法bangbang
  track_set(bool)            开启/关闭巡线（关闭时会停止电机）
  track_is_on()              返回bool，是否巡线中
  track_update()             每次loop调用，仅开启时生效，非阻塞
  注：bangbang模式下，CH3/CH5压线=缓转（内轮减速50%）；CH2/CH6压线=急转（内轮反转默认30%，
      外轮同步降速默认65%，应对R70急弯——外轮全速会带着直道动能冲进弯道，同步降速减少入弯动能，
      两个比例均运行时可调，见BT命令 turnspeed N / sharpratio N）
  注：pid模式下，误差=sensor_position_from()算出的加权位置(-1~+1，正=线偏右)，setpoint=0，
      output = Kp*error + Ki*积分 + Kd*微分，直接叠加到左右轮PWM上做差速修正（正output=左轮加速/
      右轮减速），积分限幅±2.0防止饱和；三个增益默认值(Kp40/Ki0/Kd5)未经实车验证，需要现场试调，
      见BT命令 pid kp/ki/kd N；用 algo bangbang / algo pid 切换算法，切换或track on/off时
      都会清空PID的积分/微分历史，不会带着上一轮的状态继续算
  注：无论哪种算法，都共用同一套丢线/十字路口判定：左右两侧同时压线（十字路口/宽线）判定为
      直行穿过，不触发转向；丢线（5路全黑）延续上次转向方向继续找线，超时1.5s停车
  注：内含调试log（30ms节流，格式 "T <ms> <5路WB图案> <模式> L: R:"，pid模式额外附加当前
      误差"E:"字段），模式包括 STRAIGHT/LEFT/RIGHT/SHARP_L/SHARP_R/PID/CROSS/LOST_L/LOST_R/LOST_STOP
  注：也可按Boot按钮（GPIO0）切换track on/off，不需要BT/USB命令

[cmd_module] 命令行（USB串口和BT均可输入，有回显）
  cmd_begin()                初始化
  cmd_poll()                 每次loop调用，处理输入
  注：命令响应始终可见，不受print_module控制

------------------------------------------------------------
【BT命令列表】
------------------------------------------------------------

  print on/off          USB+BT 数据流同时开关
  print usb on/off      仅控制USB数据流
  print bt  on/off      仅控制BT数据流
  go N                  前进N秒（1~60），使用当前速度
  back N                后退N秒（1~60），使用当前速度
  stop                  立即停止电机（同时会退出巡线模式）
  track on/off          开启/关闭自动巡线
  speed N               设置速度档位（1~40），仅内存生效，需save才写入Flash
  turnspeed N           设置急弯外轮速度比例（0~100%，默认65），仅内存生效，需save才写入Flash
  sharpratio N          设置急弯内轮反转比例（0~100%，默认30），仅内存生效，需save才写入Flash
  algo bangbang/pid     切换巡线算法（默认bangbang），仅内存生效，需save才写入Flash
  pid kp N              设置PID比例增益（浮点数，默认40.0），仅内存生效，需save才写入Flash
  pid ki N              设置PID积分增益（浮点数，默认0.0），仅内存生效，需save才写入Flash
  pid kd N              设置PID微分增益（浮点数，默认5.0），仅内存生效，需save才写入Flash
  save                  把当前所有可调参数（速度/急弯外轮比例/急弯内轮反转比例/算法/PID三个增益/
                        5路阈值）保存到/config.json
  config                打印当前配置的JSON内容
  threshold CH VALUE    设置CHx(2~6)的阈值，仅内存生效，需save才写入Flash
  help                  显示命令帮助

  注：开机自动从/config.json加载速度档位和阈值；文件不存在或无该字段时用默认值（速度12）。
  注：速度档位2026-07-08起从1~10改为1~40（分辨率*4，原N档=新N*4档），
      旧/config.json里保存的speed字段是旧档位数值，升级后含义已变，
      需要重新用speed命令设置并save，否则实际PWM会比预期小很多。

------------------------------------------------------------
【待完成】
------------------------------------------------------------
  - 实测R70发卡弯能否通过（结构已改，见文档最上方，待实车验证）
  - PID模式（algo pid）刚加上，Kp/Ki/Kd默认值未经实车验证，需要现场试调后再评估
    是否比bangbang更平稳

------------------------------------------------------------
【依赖库】（Arduino Library Manager安装）
------------------------------------------------------------
  - ArduinoJson（Benoit Blanchon，v6）  ← config_module使用
  LittleFS 和 BluetoothSerial 为ESP32核心自带，无需安装
