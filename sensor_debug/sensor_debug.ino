#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "bt_module.h"
#include "sensor_module.h"

const int LED_PIN = 2;
int count = 0;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  sensor_begin();
  bt_begin("LineTrace");
  Serial.println("传感器模块测试启动");
}

void loop() {
  digitalWrite(LED_PIN, bt_connected() ? HIGH : LOW);

  int vals[SENSOR_COUNT];
  bool is_white[SENSOR_COUNT];
  sensor_read(vals);
  sensor_binary(is_white);
  float pos = sensor_position();

  count++;
  char buf[96];

  // 原始值
  snprintf(buf, sizeof(buf), "[%d] 原始: %4d %4d %4d %4d %4d",
           count, vals[0], vals[1], vals[2], vals[3], vals[4]);
  Serial.print(buf); bt_send(buf);

  // 二值（W=白 B=黑）+ 位置
  snprintf(buf, sizeof(buf), "  |  %c%c%c%c%c  pos:%.2f\n",
           is_white[0]?'W':'B', is_white[1]?'W':'B',
           is_white[2]?'W':'B', is_white[3]?'W':'B',
           is_white[4]?'W':'B',
           isnan(pos) ? 0.0f : pos);
  Serial.print(buf); bt_send(buf);

  delay(200);
}
