#include "print_module.h"
#include "bt_module.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <string.h>

#define LOG_FILE_PREFIX "/track.log."
// 段数和单段上限，总容量 = LOG_SEGMENT_COUNT * LOG_SEGMENT_MAX_BYTES，约256KB
// （原来想用单个大文件+seek()做真环形缓冲区，2026-07-29实测LittleFS对大文件随机偏移写
// 极慢——单次flush能卡3~4秒，把整个loop()冻结住，巡线电机指令跟着一起卡顿，
// 比"晃"严重得多。改成多段文件轮转：每段从头到尾只做顺序追加，写满一段就切到下一段
// （先删掉该段旧内容），彻底避开LittleFS随机写的性能坑，写满同样不会停，继续绕回覆盖最老段）
#define LOG_SEGMENT_COUNT 4
#define LOG_SEGMENT_MAX_BYTES (64L * 1024L)
// 攒够这么多字节才落盘一次，减少flash写入频率
#define LOG_BUF_CAPACITY 512
// 一条log行加上"#ID "前缀之后的缓冲区，用LOG_LINE_MAX（print_module.h，跨模块共用）

static bool _usb = false;
static bool _bt  = false;
static bool _file = false;

static unsigned long _next_id = 0;        // 下一条要写的记录的全局递增ID，只在print_file_clear()时归零，重启也归零（不持久化）
static char _file_buf[LOG_BUF_CAPACITY];
static int  _file_buf_len = 0;

static int  _active_segment = 0;          // 当前正在追加写入的段号，重启归0（不持久化，跟#ID一起归零，语义一致）
static long _active_segment_bytes = 0;    // 当前段已落盘的字节数（不含缓冲区里还没落盘的部分）
static bool _active_segment_synced = false; // 是否已经从flash同步过_active_segment_bytes
static unsigned long _rotation_count = 0; // 累计切换过多少次段，用于判断是否已经绕完一整圈

static void segment_path(int idx, char* out_path, size_t out_size) {
  snprintf(out_path, out_size, "%s%d", LOG_FILE_PREFIX, idx);
}

static long query_segment_size(int idx) {
  char path[24];
  segment_path(idx, path, sizeof(path));
  if (!LittleFS.exists(path)) return 0;
  File f = LittleFS.open(path, "r");
  if (!f) return 0;
  long size = f.size();
  f.close();
  return size;
}

static void sync_active_segment() {
  if (_active_segment_synced) return;
  _active_segment_bytes = query_segment_size(_active_segment);
  _active_segment_synced = true;
}

// 把缓冲区内容顺序追加写入当前段（只用"a"追加模式，不做seek，这是避开LittleFS
// 随机写性能坑的关键）
static void file_flush() {
  if (_file_buf_len == 0) return;
  sync_active_segment();

  char path[24];
  segment_path(_active_segment, path, sizeof(path));
  File f = LittleFS.open(path, "a");
  if (f) {
    f.write((const uint8_t*)_file_buf, _file_buf_len);
    _active_segment_bytes += _file_buf_len;
    f.close();
  }
  _file_buf_len = 0;
}

// 当前段写满了，切到下一段（先把当前段剩余内容落盘，再删掉下一段的旧内容重新开始）
static void rotate_segment() {
  file_flush();
  _active_segment = (_active_segment + 1) % LOG_SEGMENT_COUNT;
  char path[24];
  segment_path(_active_segment, path, sizeof(path));
  LittleFS.remove(path);  // 绕回覆盖最老的一段：清空重新开始
  _active_segment_bytes = 0;
  _rotation_count++;
}

void print_begin() {
  _usb = false;
  _bt = false;
  _file = false;
  _next_id = 0;
  _file_buf_len = 0;
  _active_segment = 0;
  _active_segment_bytes = 0;
  _active_segment_synced = false;
  _rotation_count = 0;
  // 不在这里查LittleFS：print_begin()在setup()里排在config_begin()（挂载LittleFS）
  // 之前，此时文件系统还没mount，改为print_set_file(true)第一次开启时再同步
}

void print_set_usb(bool en) { _usb = en; }
void print_set_bt(bool en)  { _bt  = en; }

void print_set_file(bool en) {
  if (en && !_file) {
    sync_active_segment();  // 只是查一次当前段文件大小，很快，不会卡顿
  }
  if (_file && !en) file_flush();   // 关闭前把缓冲区剩余内容落盘，不丢数据
  _file = en;
}

bool print_usb_enabled()  { return _usb; }
bool print_bt_enabled()   { return _bt;  }
bool print_file_enabled() { return _file; }

void out_usb(const char* msg) { if (_usb) Serial.print(msg); }
void out_bt(const char* msg)  { if (_bt)  bt_send(msg); }

void out_file(const char* msg) {
  if (!_file) return;

  int len = strlen(msg);
  if (len <= 0) return;

  sync_active_segment();

  // 组装成"#ID 原内容"，原内容自带的\r\n不用动。ID用变长十进制（不补空格），
  // 不影响解析——仍然靠"#"和空格定界，dump仍按段号0..N-1输出、靠ID大小判断时间序，
  // 这个机制不变，只是ID小的时候能少写几个字节（见"LOG精简方案.md"3.3节）
  char rec[LOG_LINE_MAX];
  int n = snprintf(rec, sizeof(rec), "#%lu %s", _next_id, msg);
  if (n < 0) n = 0;
  if (n > (int)sizeof(rec) - 1) n = (int)sizeof(rec) - 1;
  _next_id++;

  // 这一行加上当前段已落盘+缓冲区里还没落盘的内容会超过单段上限，先转到下一段再写，
  // 保证不会有一行内容被硬切成两半分在两个段文件里
  if (_active_segment_bytes + _file_buf_len + n > LOG_SEGMENT_MAX_BYTES) {
    rotate_segment();
  }

  if (_file_buf_len + n > LOG_BUF_CAPACITY) file_flush();
  memcpy(_file_buf + _file_buf_len, rec, n);
  _file_buf_len += n;
}

void out(const char* msg) { out_usb(msg); out_bt(msg); out_file(msg); }

void print_file_flush() { file_flush(); }

long print_file_size() { return (long)LOG_SEGMENT_COUNT * LOG_SEGMENT_MAX_BYTES; }

unsigned long print_file_next_id() { return _next_id; }

bool print_file_wrapped() { return _rotation_count >= (unsigned long)LOG_SEGMENT_COUNT; }

// modeCode/事件行格式说明，脱离"LOG精简方案.md"也能看懂dump出来的内容（见该文档第7节风险）
void print_log_legend() {
  Serial.println(">>> log line format: E<dt> <patHex> <modeCode> <L> <R>[ <err>]  (H=heartbeat instead of E)");
  Serial.println(">>>   E行dt=ms since previous mode switch (not reset by heartbeats in between)");
  Serial.println(">>>   H行dt=ms since previous logged line (heartbeat cadence)");
  Serial.println(">>>   patHex=2-digit hex bitmask (bit7=CH8..bit0=CH1, 1=white); L/R=actual PWM sent; err=PID error (PID mode only)");
  Serial.println(">>> modeCode: 0=STRAIGHT 1=LEFT(mild) 2=RIGHT(mild) 3=MEDIUM_L 4=MEDIUM_R 5=SHARP_L 6=SHARP_R");
  Serial.println(">>>           7=HAIRPIN_L 8=HAIRPIN_R 9=CROSS A=LOST_L B=LOST_R C=LOST_STOP P=PID");
  Serial.println(">>> markers: >>> TRACK_ON t=<ms> / >>> PARAMS ... / >>> TRACK_OFF t=<ms> elapsed=<ms>ms duration=<ms>ms lines=<n>");
  Serial.println(">>>   elapsed=track on~off挂钟时长, duration=事件/心跳数据实际覆盖的时长(缓冲区提前写满时会比elapsed短)");
}

void print_file_dump() {
  file_flush();
  print_log_legend();

  bool any = false;
  for (int i = 0; i < LOG_SEGMENT_COUNT; i++) {
    char path[24];
    segment_path(i, path, sizeof(path));
    if (LittleFS.exists(path)) any = true;
  }
  if (!any) {
    Serial.println(">>> No log file (/track.log.* not found)");
    return;
  }

  Serial.print(">>> --- log dump start (");
  Serial.print(LOG_SEGMENT_COUNT);
  Serial.print(" segments, next_id=");
  Serial.print(_next_id);
  Serial.println(") 按段号0..N-1固定顺序输出，不代表时间顺序，行首#ID断层处就是绕回点 ---");

  char buf[128];
  for (int i = 0; i < LOG_SEGMENT_COUNT; i++) {
    char path[24];
    segment_path(i, path, sizeof(path));
    if (!LittleFS.exists(path)) continue;
    File f = LittleFS.open(path, "r");
    if (!f) continue;

    Serial.print(">>> --- segment ");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(f.size());
    Serial.println(" bytes) ---");

    int n;
    while ((n = f.read((uint8_t*)buf, sizeof(buf))) > 0) {
      Serial.write((const uint8_t*)buf, n);
      yield();  // 大文件在115200波特率下要传输好几秒，不yield会喂不到任务看门狗导致复位
    }
    f.close();
  }
  Serial.println(">>> --- log dump end ---");
}

void print_file_clear() {
  _file_buf_len = 0;    // 直接丢弃还没落盘的部分，没必要写了马上又删
  _next_id = 0;
  _active_segment = 0;
  _active_segment_bytes = 0;
  _active_segment_synced = true;  // 清空后就是0字节，不用再查一次flash
  _rotation_count = 0;
  for (int i = 0; i < LOG_SEGMENT_COUNT; i++) {
    char path[24];
    segment_path(i, path, sizeof(path));
    LittleFS.remove(path);
  }
}
