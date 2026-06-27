#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "bt_module.h"

const int LED_PIN = 2;
const int PIN_L   = 35;
const int PIN_R   = 36;
int count = 0;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  analogReadResolution(12);

  bt_begin("LineTrace");
  Serial.println("蓝牙已启动，设备名: LineTrace");
}

void loop() {
  bool connected = bt_connected();
  int onTime  = connected ? 900 : 100;
  int offTime = connected ? 100 : 900;

  digitalWrite(LED_PIN, HIGH);
  delay(onTime);
  digitalWrite(LED_PIN, LOW);
  delay(offTime);

  int valL = analogRead(PIN_L);
  int valR = analogRead(PIN_R);
  count++;

  char buf[64];
  snprintf(buf, sizeof(buf), "[%d] L:%4d  R:%4d\n", count, valL, valR);

  Serial.print(buf);
  bt_send(buf);
}
