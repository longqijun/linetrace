#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "bt_module.h"
#include "print_module.h"
#include "sensor_module.h"
#include "motor_module.h"
#include "config_module.h"
#include "track_module.h"
#include "cmd_module.h"

const int LED_PIN = 2;
const int BUTTON_PIN = 0;  // Boot按钮，按下=低电平，用作track on/off物理开关
int count = 0;
bool last_button_state = HIGH;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  print_begin();    // USB和BT打印默认关闭
  sensor_begin();
  motor_begin();
  config_begin();   // 从/config.json加载速度档位和传感器阈值（无文件/无字段则用默认值）
  track_begin();
  bt_begin("LineTrace");

  // 阈值提示走 reply 路径（始终可见）
  Serial.println("Boot complete. Type help for commands");
  Serial.println("Type 'print on' to enable sensor data stream");
  config_print();

  cmd_begin();
}

void loop() {
  digitalWrite(LED_PIN, bt_connected() ? HIGH : LOW);

  cmd_poll();

  bool button_state = digitalRead(BUTTON_PIN);
  if (button_state == LOW && last_button_state == HIGH) {
    delay(30); // 消抖
    if (digitalRead(BUTTON_PIN) == LOW) {
      track_set(!track_is_on());
      const char* msg = track_is_on() ? "Track ON (button)\r\n" : "Track OFF (button)\r\n";
      Serial.print(msg);
      bt_send(msg);
    }
  }
  last_button_state = button_state;

  track_update();

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
