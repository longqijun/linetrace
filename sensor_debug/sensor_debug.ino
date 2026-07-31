#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_bt.h"
#include "bt_module.h"
#include "print_module.h"
#include "sensor_module.h"
#include "motor_module.h"
#include "config_module.h"
#include "track_module.h"
#include "cmd_module.h"
#include "ram_log_module.h"

const int BUTTON_PIN = 0;  // Boot按钮，按下=低电平，用作track on/off物理开关
int count = 0;
bool last_button_state = HIGH;

// loop()性能统计：每圈记一次耗时(micros()差值)，累计min/max/avg，每秒打印一次汇总后清零。
// 开销只有一次micros()调用+几次比较/加法，跟analogRead()本身的耗时比可以忽略，
// 不会影响巡线控制的实时性
unsigned long loop_stat_last_us = 0;
unsigned long loop_stat_count = 0;
unsigned long loop_stat_sum_us = 0;
unsigned long loop_stat_min_us = 0xFFFFFFFF;
unsigned long loop_stat_max_us = 0;
unsigned long loop_stat_last_print_ms = 0;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // BT已经彻底不用了（bt_begin()从未被调用，见下面注释），显式把BT控制器预留的内存
  // （经典BT+BLE都不用，用BTDM一次性全释放）还给通用堆，让ram_log_auto_init()能多分到
  // 一些空间。必须在任何BT初始化动作之前调用最早、最先执行；调用之后这次开机周期内
  // BT就再也起不来了——这跟本项目"BT已彻底停用"的既有决定是一致的，如果以后要恢复BT，
  // 必须先删掉这一行，不然bt_begin()会失败
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  print_begin();    // USB和BT打印默认关闭
  sensor_begin();
  motor_begin();
  config_begin();   // 从/config.json加载速度档位和传感器阈值（无文件/无字段则用默认值）
  track_begin();
  // BT关闭：连接一直不稳定，反复诊断也没能根治，干脆不开BT模块，只用USB Serial
  // （命令输入、log dump等全部功能USB本来就都支持，见cmd_module.cpp的cmd_poll()）。
  // bt_module.cpp现在是空实现（stub，不再链接BluetoothSerial库，见该文件注释），
  // 光取消下面这行注释不够用了——如果以后想恢复BT，要先把bt_module.cpp换回真正调
  // BluetoothSerial的实现（查git历史），再把上面的esp_bt_controller_mem_release()
  // 删掉（释放过controller内存之后这次开机周期内BT不可能再启动成功），最后才取消这行注释：
  // bt_begin("LineTrace");

  // 放在其他_begin()都跑完之后：这时候读到的空闲堆才是"稳态下真实能用的余量"，
  // 缓冲区大小按这个自动定，不用每次改代码都手动改一个写死的字节数（见"LOG精简方案.md"4.2节）
  ram_log_auto_init();

  // 阈值提示走 reply 路径（始终可见）
  Serial.println("Boot complete. Type help for commands");
  Serial.println("Type 'print on' to enable sensor data stream");
  config_print();

  cmd_begin();
}

void loop() {
  // 测的是"上一圈loop()从开始到这一圈开始"的耗时，即上一圈的完整执行时间
  unsigned long loop_now_us = micros();
  if (loop_stat_last_us != 0) {
    unsigned long dur_us = loop_now_us - loop_stat_last_us;
    loop_stat_count++;
    loop_stat_sum_us += dur_us;
    if (dur_us < loop_stat_min_us) loop_stat_min_us = dur_us;
    if (dur_us > loop_stat_max_us) loop_stat_max_us = dur_us;
  }
  loop_stat_last_us = loop_now_us;

  unsigned long loop_stat_now_ms = millis();
  if (loop_stat_now_ms - loop_stat_last_print_ms >= 1000 && loop_stat_count > 0) {
    loop_stat_last_print_ms = loop_stat_now_ms;
    char stat_buf[96];
    snprintf(stat_buf, sizeof(stat_buf), "LOOP %lu/s avg=%luus min=%luus max=%luus\r\n",
             loop_stat_count, loop_stat_sum_us / loop_stat_count, loop_stat_min_us, loop_stat_max_us);
    out(stat_buf);  // 受print_module控制，默认不输出
    loop_stat_count = 0;
    loop_stat_sum_us = 0;
    loop_stat_min_us = 0xFFFFFFFF;
    loop_stat_max_us = 0;
  }

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
    sensor_binary_from(vals, is_white);
    float pos = sensor_position_from(is_white);
    float apos = sensor_position_analog_from(vals);  // 模拟量加权版，标定whiteref/blackref时对照用

    count++;
    char buf[176];
    // 打印顺序按物理左→右排列（index 7→0，即CH8..CH1），跟track_module.cpp的T行保持一致
    snprintf(buf, sizeof(buf), "[%d] %4d%c %4d%c %4d%c %4d%c %4d%c %4d%c %4d%c %4d%c  pos:%.2f apos:%.2f\r\n",
             count,
             vals[7], is_white[7]?'W':'B',
             vals[6], is_white[6]?'W':'B',
             vals[5], is_white[5]?'W':'B',
             vals[4], is_white[4]?'W':'B',
             vals[3], is_white[3]?'W':'B',
             vals[2], is_white[2]?'W':'B',
             vals[1], is_white[1]?'W':'B',
             vals[0], is_white[0]?'W':'B',
             isnan(pos) ? 0.0f : pos,
             isnan(apos) ? 0.0f : apos);
    out(buf);  // 受 print_module 控制，默认不输出
  }
}
