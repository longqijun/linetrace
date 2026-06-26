// 电源测试代码（无蓝牙）
// 通过 LED 闪烁确认 2节电池+升压模块能否稳定供电
// LED 每秒闪一次 = 正常运行
// LED 停止闪烁或闪烁不规律 = 电源不稳定

const int LED_PIN = 2;   // ESP32 板载 LED
const int PIN_L   = 35;  // 左中传感器 D35
const int PIN_R   = 36;  // 右中传感器 VP

int count = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12);
  Serial.println("启动成功（无蓝牙）");
}

void loop() {
  // LED 闪烁：亮 100ms 灭 900ms
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(900);

  // 读传感器并打印（USB 连接时可查看）
  int valL = analogRead(PIN_L);
  int valR = analogRead(PIN_R);
  count++;
  Serial.printf("[%d] 左:%4d  右:%4d\n", count, valL, valR);
}
