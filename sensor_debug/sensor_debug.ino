#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "bt_module.h"
#include "print_module.h"
#include "sensor_module.h"
#include "motor_module.h"
#include "cmd_module.h"

const int LED_PIN = 2;
int count = 0;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  print_begin();    // USB和BT打印默认关闭
  sensor_begin();
  motor_begin();
  bt_begin("LineTrace");

  // 阈值提示走 reply 路径（始终可见）
  Serial.println("Boot complete. Type help for commands");
  Serial.println("Type 'print on' to enable sensor data stream");

  cmd_begin();
}

void loop() {
  digitalWrite(LED_PIN, bt_connected() ? HIGH : LOW);

  cmd_poll();

  static unsigned long last_print = 0;
  if (millis() - last_print >= 200) {
    last_print = millis();

    int vals[SENSOR_COUNT];
    bool is_white[SENSOR_COUNT];
    sensor_read(vals);
    sensor_binary(is_white);
    float pos = sensor_position();

    count++;
    char buf[96];
    snprintf(buf, sizeof(buf), "[%d] %4d%c %4d%c %4d%c %4d%c %4d%c  pos:%.2f\r\n",
             count,
             vals[0], is_white[0]?'W':'B',
             vals[1], is_white[1]?'W':'B',
             vals[2], is_white[2]?'W':'B',
             vals[3], is_white[3]?'W':'B',
             vals[4], is_white[4]?'W':'B',
             isnan(pos) ? 0.0f : pos);
    out(buf);  // 受 print_module 控制，默认不输出
  }
}
