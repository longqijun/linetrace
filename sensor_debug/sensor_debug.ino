// AE-NJL5901AR-8CH 红外传感器调试
// D32 D33 D34 D35 VP(36) VN(39) D13 D14 → CH1~CH8
// 注意：D13/D14 是 ADC2 引脚，WiFi/BT 启用时不可用

const int sensorPins[8] = {32, 33, 34, 35, 36, 39, 13, 14};
const char* chNames[8]  = {"CH1", "CH2", "CH3", "CH4", "CH5", "CH6", "CH7", "CH8"};

// 判断是否触线的阈值（根据实际环境调整）
const int THRESHOLD = 2000;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // 12位：0~4095
  delay(500);
  Serial.println("\n=== AE-NJL5901AR-8CH 传感器调试 ===");
  Serial.println(" CH1   CH2   CH3   CH4   CH5   CH6   CH7   CH8   | 触线状态");
  Serial.println("---------------------------------------------------+----------");
}

void loop() {
  char line[16];  // 触线可视化：1=触线，0=未触线

  for (int i = 0; i < 8; i++) {
    int val = analogRead(sensorPins[i]);
    Serial.printf("%4d  ", val);
    line[i] = (val < THRESHOLD) ? '1' : '0';  // 低值=检测到黑线（视传感器而定）
  }
  line[8] = '\0';
  Serial.printf("| %s\n", line);

  delay(1000);
}
