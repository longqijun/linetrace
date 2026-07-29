#include "print_module.h"
#include "bt_module.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <string.h>

#define LOG_FILE "/track.log"
// 每条记录固定长度（不足补空格，超长截断），环形缓冲区靠"记录序号×固定长度"
// 直接算出文件偏移量，所以必须定长——按目前最长的一行（10位ms时间戳+LOST_STOP等9字符
// mode+#ID前缀）留够余量，正常长度的行只是尾部多几个空格，不影响阅读
#define LOG_RECORD_SIZE 96
// 环形缓冲区总槽位数，总容量 = LOG_RECORD_SIZE * LOG_RECORD_COUNT，约256KB
// （原来是"写满256KB就停"，现在写满后绕回槽位0继续覆盖最老的记录，不会再停）
#define LOG_RECORD_COUNT 2730L
#define LOG_FILE_MAX_BYTES (LOG_RECORD_SIZE * LOG_RECORD_COUNT)
// 攒够这么多条记录才落盘一次，减少flash写入频率（原因同之前：30ms节流的调试log很密，
// 每条都单独seek+write会拖慢track_update()，间接影响巡线控制的实时性）
#define LOG_BATCH_RECORDS 10

static bool _usb = false;
static bool _bt  = false;
static bool _file = false;

static unsigned long _next_id = 0;     // 下一条要写的记录的全局递增ID，只在print_file_clear()时归零，重启也归零（不持久化）
static char _batch[LOG_BATCH_RECORDS * LOG_RECORD_SIZE];
static int  _batch_records = 0;        // 缓冲区里已攒的记录条数
static unsigned long _batch_first_id = 0; // 缓冲区里第一条记录对应的ID，据此算它在环形文件里的槽位
static bool _file_inited = false;      // /track.log是否已经预分配到LOG_FILE_MAX_BYTES大小

static long query_file_size() {
  if (!LittleFS.exists(LOG_FILE)) return 0;
  File f = LittleFS.open(LOG_FILE, "r");
  if (!f) return 0;
  long size = f.size();
  f.close();
  return size;
}

// 把文件整份预填成空白槽位（末尾都是"\r\n"，没有"#ID"前缀），之后才能用seek()
// 精确定位写入任意槽位——LittleFS不支持seek到当前文件末尾之后再写（不是稀疏文件），
// 环形缓冲区要求文件从一开始就是满尺寸的
static void ensure_file_preallocated() {
  if (_file_inited) return;
  if (query_file_size() == LOG_FILE_MAX_BYTES) {
    _file_inited = true;
    return;
  }

  File f = LittleFS.open(LOG_FILE, "w");
  if (!f) return;

  char blank[LOG_RECORD_SIZE];
  memset(blank, ' ', LOG_RECORD_SIZE - 2);
  blank[LOG_RECORD_SIZE - 2] = '\r';
  blank[LOG_RECORD_SIZE - 1] = '\n';
  for (long i = 0; i < LOG_RECORD_COUNT; i++) {
    f.write((const uint8_t*)blank, LOG_RECORD_SIZE);
    if ((i & 0x1F) == 0) yield();  // 预分配约256KB耗时数秒，定期yield避免喂不到看门狗触发复位
  }
  f.close();
  _file_inited = true;
}

// 把_batch里攒的record_count条记录（对应ID为first_id..first_id+record_count-1，
// 在环形文件里是连续槽位）落盘。可能跨过文件末尾绕回槽位0，此时拆成两段写
static void flush_records(const char* data, int record_count, unsigned long first_id) {
  if (record_count <= 0) return;
  ensure_file_preallocated();

  File f = LittleFS.open(LOG_FILE, "r+");
  if (!f) return;  // 打开失败没法恢复，只能丢弃这批，避免阻塞巡线控制循环

  long start_slot = first_id % LOG_RECORD_COUNT;
  long first_part = LOG_RECORD_COUNT - start_slot;
  if (first_part > record_count) first_part = record_count;

  f.seek((uint32_t)(start_slot * LOG_RECORD_SIZE), SeekSet);
  f.write((const uint8_t*)data, (size_t)first_part * LOG_RECORD_SIZE);

  int remain = record_count - (int)first_part;
  if (remain > 0) {
    // 这一批记录跨过了环形缓冲区末尾，绕回文件开头继续写剩下的部分
    f.seek(0, SeekSet);
    f.write((const uint8_t*)(data + (size_t)first_part * LOG_RECORD_SIZE),
            (size_t)remain * LOG_RECORD_SIZE);
  }
  f.close();
}

static void flush_batch() {
  if (_batch_records == 0) return;
  flush_records(_batch, _batch_records, _batch_first_id);
  _batch_records = 0;
}

void print_begin() {
  _usb = false;
  _bt = false;
  _file = false;
  _next_id = 0;
  _batch_records = 0;
  _file_inited = false;
  // 不在这里预分配/查LittleFS：print_begin()在setup()里排在config_begin()（挂载LittleFS）
  // 之前，此时文件系统还没mount，改为print_set_file(true)第一次开启时再同步
}

void print_set_usb(bool en) { _usb = en; }
void print_set_bt(bool en)  { _bt  = en; }

void print_set_file(bool en) {
  if (en && !_file) {
    ensure_file_preallocated();  // 首次开启（或log clear之后重新开启）可能要预分配整个环形文件，耗时数秒
  }
  if (_file && !en) flush_batch();   // 关闭前把缓冲区剩余内容落盘，不丢数据
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

  // 组装成固定长度记录："#ID 内容"，不足补空格，超长截断，末尾统一"\r\n"
  char rec[LOG_RECORD_SIZE];
  int n = snprintf(rec, LOG_RECORD_SIZE - 1, "#%8lu %s", _next_id, msg);
  if (n < 0) n = 0;
  if (n > LOG_RECORD_SIZE - 2) n = LOG_RECORD_SIZE - 2;
  while (n > 0 && (rec[n - 1] == '\r' || rec[n - 1] == '\n')) n--;  // 原内容自带的\r\n，统一在下面重新补
  for (int i = n; i < LOG_RECORD_SIZE - 2; i++) rec[i] = ' ';
  rec[LOG_RECORD_SIZE - 2] = '\r';
  rec[LOG_RECORD_SIZE - 1] = '\n';

  if (_batch_records == 0) _batch_first_id = _next_id;
  memcpy(_batch + (size_t)_batch_records * LOG_RECORD_SIZE, rec, LOG_RECORD_SIZE);
  _batch_records++;
  _next_id++;

  if (_batch_records >= LOG_BATCH_RECORDS) flush_batch();
}

void out(const char* msg) { out_usb(msg); out_bt(msg); out_file(msg); }

void print_file_flush() { flush_batch(); }

long print_file_size() { return _file_inited ? LOG_FILE_MAX_BYTES : 0; }

unsigned long print_file_next_id() { return _next_id; }

bool print_file_wrapped() { return _next_id >= (unsigned long)LOG_RECORD_COUNT; }

void print_file_dump() {
  flush_batch();
  if (!_file_inited || !LittleFS.exists(LOG_FILE)) {
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
  Serial.print(" bytes, ring buffer, next_id=");
  Serial.print(_next_id);
  Serial.println(") 行首#ID按时间递增，物理行顺序不代表时间顺序，ID断层处就是绕回点 ---");

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
  _batch_records = 0;    // 直接丢弃还没落盘的部分，没必要写了马上又删
  _next_id = 0;
  _file_inited = false;
  LittleFS.remove(LOG_FILE);
  if (_file) ensure_file_preallocated();  // file log还开着的话立刻重新预分配，避免下次flush时在巡线过程中才卡住
}
