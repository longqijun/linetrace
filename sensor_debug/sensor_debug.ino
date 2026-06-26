// AE-NJL5901AR-8CH 红外传感器调试（蓝牙版）
// 只使用中心两路：D35(CH4) 和 VP/GPIO36(CH5)，均为 ADC1，蓝牙兼容
// 手机安装 "Serial Bluetooth Terminal" 连接设备名 "ESP32-Sensor"

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

const int PIN_L = 35;  // D35 - 左中传感器
const int PIN_R = 36;  // VP  - 右中传感器

// 阈值：低于此值视为检测到黑线（根据实际环境调整）
const int THRESHOLD = 2000;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // 12位：0~4095

  SerialBT.begin("ESP32-Sensor");

  Serial.println("蓝牙已启动，等待连接...");
  SerialBT.println("=== 传感器调试就绪 ===");
  SerialBT.println("  左(D35)   右(VP)   状态");
  SerialBT.println("---------------------------");
}

void loop() {
  int valL = analogRead(PIN_L);
  int valR = analogRead(PIN_R);

  bool onLineL = valL < THRESHOLD;
  bool onLineR = valR < THRESHOLD;

  // 判断位置
  const char* status;
  if (onLineL && onLineR)       status = "正中";
  else if (onLineL && !onLineR) status = "偏右";
  else if (!onLineL && onLineR) status = "偏左";
  else                          status = "脱线";

  char msg[48];
  snprintf(msg, sizeof(msg), "  %4d      %4d     %s", valL, valR, status);

  Serial.println(msg);
  SerialBT.println(msg);

  delay(200);  // 改为200ms，调试更流畅
}
