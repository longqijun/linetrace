============================
巡线小车项目 - 硬件信息
============================

【主控】
  ESP32（Arduino IDE 开发）

【传感器】
  AE-NJL5901AR-8CH（8路红外反射传感器）
  传感器间距：16mm
  比赛线宽：19mm（白线）

【传感器引脚映射】（PCB固定，不可修改）
  CH1 → D32  (ADC1) ✓ 蓝牙可用
  CH2 → D33  (ADC1) ✓ 蓝牙可用
  CH3 → D34  (ADC1) ✓ 蓝牙可用
  CH4 → D35  (ADC1) ✓ 蓝牙可用
  CH5 → VP   (GPIO36, ADC1) ✓ 蓝牙可用
  CH6 → VN   (GPIO39, ADC1) ✓ 蓝牙可用
  CH7 → D13  (ADC2) ✗ 蓝牙启用时不可用
  CH8 → D14  (ADC2) ✗ 蓝牙启用时不可用

【传感器使用策略】
  - 开蓝牙时最多用 CH1~CH6（6路，全ADC1）
  - 推荐取中间5路：CH2~CH6（D33/D34/D35/VP/VN），跨度64mm，左右对称
  - 最少需要3路传感器才能判断方向（3路只能bang-bang控制）
  - 5路可做加权位置计算，支持PID平滑控制

【电源】
  双18650方案：
  - 电池1：供 ESP32 + 传感器
  - 电池2：供 电机
  注意：上电瞬间电压抖动，需关闭ESP32欠压检测器
  代码：WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

【蓝牙】
  协议：BT SPP（BluetoothSerial）
  设备名：LineTrace
  已封装为独立模块 bt_module.h / bt_module.cpp
  接口：bt_begin(name) / bt_connected() / bt_send(msg)
  LED指示：快闪=未连接，慢闪=已连接

【其他引脚】
  LED：GPIO2（板载）

【代码结构】
  sensor_debug/
  ├── sensor_debug.ino   主程序
  ├── bt_module.h        蓝牙模块接口
  └── bt_module.cpp      蓝牙模块实现
