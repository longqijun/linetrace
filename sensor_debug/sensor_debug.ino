// 电源测试代码（无蓝牙，关闭欠压保护）
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

const int LED_PIN = 2;
const int PIN_L   = 35;
const int PIN_R   = 36;
int count = 0;

void setup() {
  // 关闭欠压检测器，防止启动瞬间电压抖动导致重启
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  analogReadResolution(12);
  Serial.println("启动成功");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(900);

  int valL = analogRead(PIN_L);
  int valR = analogRead(PIN_R);
  count++;
  Serial.printf("[%d] 左:%4d  右:%4d\n", count, valL, valR);
}
