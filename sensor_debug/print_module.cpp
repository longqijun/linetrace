#include "print_module.h"
#include "bt_module.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <string.h>

#define LOG_FILE "/track.log"
// 攒够这么多字节才落盘一次，减少flash写入频率（30ms节流的调试log很密，
// 直接每条都写flash会拖慢track_update()，间接影响巡线控制的实时性）
#define LOG_BUF_CAPACITY 1024
// 单个log文件大小上限，防止忘记log clear导致一直写、把flash分区写满
#define LOG_FILE_MAX_BYTES (256L * 1024L)

static bool _usb = false;
static bool _bt  = false;
static bool _file = false;

static char _file_buf[LOG_BUF_CAPACITY];
static int  _file_buf_len = 0;
static long _file_total_bytes = 0;   // 已落盘的字节数缓存，避免每次out_file都查文件系统
static bool _file_full_warned = false;

static long query_file_size() {
  if (!LittleFS.exists(LOG_FILE)) return 0;
  File f = LittleFS.open(LOG_FILE, "r");
  if (!f) return 0;
  long size = f.size();
  f.close();
  return size;
}

static void file_flush() {
  if (_file_buf_len == 0) return;
  File f = LittleFS.open(LOG_FILE, "a");
  if (f) {
    f.write((const uint8_t*)_file_buf, _file_buf_len);
    _file_total_bytes = f.size();
    f.close();
  }
  _file_buf_len = 0;
}

void print_begin() {
  _usb = false;
  _bt = false;
  _file = false;
  _file_buf_len = 0;
  _file_full_warned = false;
  // 不在这里查LittleFS文件大小：print_begin()在setup()里排在config_begin()（挂载LittleFS）
  // 之前，此时文件系统还没mount，改为print_set_file(true)第一次开启时再同步
}

void print_set_usb(bool en) { _usb = en; }
void print_set_bt(bool en)  { _bt  = en; }

void print_set_file(bool en) {
  if (en && !_file) {
    _file_total_bytes = query_file_size();
    _file_full_warned = false;
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

  if (_file_total_bytes + _file_buf_len + len > LOG_FILE_MAX_BYTES) {
    if (!_file_full_warned) {
      _file_full_warned = true;
      Serial.println("ERROR: log file full (256KB), run 'log clear' to reset");
    }
    return;
  }

  if (_file_buf_len + len > LOG_BUF_CAPACITY) file_flush();

  int copy_len = len > LOG_BUF_CAPACITY ? LOG_BUF_CAPACITY : len;  // 正常log行不会触发截断
  memcpy(_file_buf + _file_buf_len, msg, copy_len);
  _file_buf_len += copy_len;
}

void out(const char* msg) { out_usb(msg); out_bt(msg); out_file(msg); }

void print_file_flush() { file_flush(); }

long print_file_size() { return _file_total_bytes + _file_buf_len; }

void print_file_dump() {
  file_flush();
  if (!LittleFS.exists(LOG_FILE)) {
    Serial.println(">>> No log file (/track.log not found)");
    return;
  }
  File f = LittleFS.open(LOG_FILE, "r");
  if (!f) {
    Serial.println(">>> Failed to open log file");
    return;
  }
  Serial.print(">>> --- log dump start (");
  Serial.print(f.size());
  Serial.println(" bytes) ---");

  char buf[128];
  int n;
  while ((n = f.read((uint8_t*)buf, sizeof(buf))) > 0) {
    Serial.write((const uint8_t*)buf, n);
    yield();  // 大文件在115200波特率下要传输好几秒，不yield会喂不到任务看门狗导致复位
  }
  f.close();
  Serial.println(">>> --- log dump end ---");
}

void print_file_clear() {
  _file_buf_len = 0;    // 直接丢弃还没落盘的部分，没必要写了马上又删
  LittleFS.remove(LOG_FILE);
  _file_total_bytes = 0;
  _file_full_warned = false;
}
