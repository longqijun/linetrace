#include "ram_log_module.h"
#include "print_module.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// 分块分配，而不是一次性malloc一大块：ESP32上"总空闲堆"和"malloc()一次能要到的最大
// 连续块"经常对不上（实测过327892字节总空闲，一次性能分配到的连续块只有100~115KB，
// 中间两百多KB是碎片，要不到）。改成每份只要20KB，同样碎片化的堆上命中率高得多，
// 一份一份分配、分到剩余空闲堆逼近安全余量为止，能拿到的总容量更接近"总空闲堆-余量"
// 这个理想值，而不是被"必须一整块连续"这个多余的限制卡住。
#define RAM_LOG_CHUNK_BYTES (20UL * 1024UL)
// chunk指针数组的上限，只是数组容量的保险丝（16*20KB=320KB，远超过实际可能分到的量），
// 不是目标值，不用刻意去够这个数
#define RAM_LOG_MAX_CHUNKS 16
// 留给栈/LittleFS/其他运行时开销的安全余量：分配下一份chunk前，如果分完之后剩余空闲堆
// 会低于这个值，就停止继续分配（绝对值）
#define RAM_LOG_STOP_MARGIN_BYTES (100UL * 1024UL)
// 不再按时间窗口截止，改成按剩余空间截止：事件/心跳行写到整个缓冲区剩余空间低于这个值
// 就不再写了，把这部分空间留给收尾的TRACK_OFF标记行（不管track on跑了多久，TRACK_OFF
// 总要能写进去），只有track off时的收尾标记（走ram_log_append_marker()）能动用这块余量
#define RAM_LOG_RESERVE_BYTES (1UL * 1024UL)

static char*  _chunks[RAM_LOG_MAX_CHUNKS];
static size_t _chunk_used[RAM_LOG_MAX_CHUNKS];  // 这一趟track on里，每份chunk已经写了多少字节
static size_t _chunk_count = 0;                 // 开机时实际分配到的chunk数量
static size_t _cur_chunk = 0;                   // 当前正在写的chunk下标
static unsigned long _start_ms = 0;   // 0=尚未begin，非0=这一趟采集已经开始
static unsigned long _last_append_ms = 0;  // 最近一条"事件/心跳"行的时间戳（不含TRACK_ON/PARAMS/TRACK_OFF这些标记行），用来算log实际覆盖的时长
static unsigned long _line_count = 0;

static size_t total_used_bytes() {
  size_t used = 0;
  for (size_t i = 0; i < _chunk_count; i++) used += _chunk_used[i];
  return used;
}

// setup()末尾调用一次：一份一份malloc(RAM_LOG_CHUNK_BYTES)，直到剩余空闲堆逼近安全余量、
// 或者某一份分配失败、或者数组分满为止。之后不再malloc/free，不会有堆碎片问题。
void ram_log_auto_init() {
  uint32_t free_before = ESP.getFreeHeap();

  _chunk_count = 0;
  while (_chunk_count < RAM_LOG_MAX_CHUNKS) {
    uint32_t free_now = ESP.getFreeHeap();
    if (free_now < RAM_LOG_STOP_MARGIN_BYTES + RAM_LOG_CHUNK_BYTES) break;  // 再分一份就会跌破安全余量，停

    char* chunk = (char*)malloc(RAM_LOG_CHUNK_BYTES);
    if (chunk == NULL) break;  // 这份都要不到，说明碎片已经细到这个粒度也扛不住了，停

    _chunks[_chunk_count] = chunk;
    _chunk_used[_chunk_count] = 0;
    _chunk_count++;
  }

  char msg[144];
  snprintf(msg, sizeof(msg), ">>> RAM log buffer: %u bytes in %u x %uKB chunks (free heap before %u, after %u)\r\n",
           (unsigned)(_chunk_count * RAM_LOG_CHUNK_BYTES), (unsigned)_chunk_count,
           (unsigned)(RAM_LOG_CHUNK_BYTES / 1024UL), (unsigned)free_before, (unsigned)ESP.getFreeHeap());
  Serial.print(msg);
}

size_t ram_log_capacity() {
  return _chunk_count * RAM_LOG_CHUNK_BYTES;
}

void ram_log_begin() {
  for (size_t i = 0; i < _chunk_count; i++) _chunk_used[i] = 0;
  _cur_chunk = 0;
  _line_count = 0;
  _last_append_ms = 0;
  _start_ms = millis();
  if (_start_ms == 0) _start_ms = 1;  // 避免跟"尚未begin"的0语义冲突（millis()==0极小概率但不是不可能）
}

// reserve_for_marker=false：普通事件/心跳行（ram_log_append()）——写到整个缓冲区剩余
// 空间低于RAM_LOG_RESERVE_BYTES时就不再写，把这块余量让给收尾标记；
// reserve_for_marker=true：TRACK_ON/PARAMS/TRACK_OFF这些标记行（ram_log_append_marker()）——
// 可以动用那块预留余量，只受chunk硬容量限制（不然万一事件数据把缓冲区吃到只剩几十字节，
// 结尾的TRACK_OFF反而写不进去，log就没有收尾标记了）
static void append_internal(const char* line, bool reserve_for_marker) {
  if (_chunk_count == 0 || _start_ms == 0) return;  // 没有chunk（分配失败）或没调用过ram_log_begin()，静默跳过

  // 所有chunk都已经写满（上一次调用把_cur_chunk推到了末尾）：这个判断必须放在最前面、
  // 每次调用都重新检查，不能只在"刚好写满那一刻"判断一次——不然后续调用会拿着一个
  // 越界的_cur_chunk去访问_chunks[]/_chunk_used[]，写到没分配过的空指针上
  if (_cur_chunk >= _chunk_count) return;

  size_t n = strlen(line);
  if (n == 0 || n > RAM_LOG_CHUNK_BYTES) return;  // 单行比一份chunk还大理论不会发生（LOG_LINE_MAX远小于20KB），防御一下

  if (!reserve_for_marker) {
    size_t total_capacity = _chunk_count * RAM_LOG_CHUNK_BYTES;
    size_t used = total_used_bytes();
    if (total_capacity - used < RAM_LOG_RESERVE_BYTES) return;  // 剩余空间不多了，让给收尾标记，静默丢弃
  }

  // 当前chunk装不下这一行就整体挪到下一份chunk（不把一行拆开写在两份chunk里，
  // 这样flush时每份chunk内部按行扫描就够了，不用处理跨chunk的半行）
  if (_chunk_used[_cur_chunk] + n > RAM_LOG_CHUNK_BYTES) {
    _cur_chunk++;
    if (_cur_chunk >= _chunk_count) return;  // 挪到下一份发现已经是最后一份，停，静默丢弃
  }

  memcpy(_chunks[_cur_chunk] + _chunk_used[_cur_chunk], line, n);
  _chunk_used[_cur_chunk] += n;
  _line_count++;
  // 只有真正的事件/心跳行才计入"log时长"，TRACK_ON/PARAMS/TRACK_OFF这些标记行的时间戳不算，
  // 不然track off如果拖得很晚，会把duration算成"track on到track off"的挂钟时间，
  // 而不是"实际捕捉到的数据到底覆盖了多久"（挂钟时间另有ram_log_elapsed_ms()/track_module.cpp算）
  if (!reserve_for_marker) _last_append_ms = millis();
}

void ram_log_append(const char* line) {
  append_internal(line, false);
}

void ram_log_append_marker(const char* line) {
  append_internal(line, true);
}

// 实际捕捉到的事件/心跳数据一共覆盖了多长时间（ms）：最后一条事件/心跳行相对于track on
// 的时间差。跟"track on到track off经过了多久"（挂钟时间，见track_module.cpp的elapsed）是
// 两回事——如果缓冲区提前写满，事件数据可能比track实际运行的时间短，这个函数返回的就是
// "log里真正有数据的那一段"有多长
unsigned long ram_log_duration_ms() {
  if (_last_append_ms == 0 || _start_ms == 0 || _last_append_ms < _start_ms) return 0;
  return _last_append_ms - _start_ms;
}

// 直接把内存里当前的内容打印到Serial，不经过flash。随时可调用——包括track还在跑、
// 还没off的时候，看到的就是"目前为止已经捕捉到的内容"，不需要等一个完整的track on~off周期
void ram_log_dump() {
  print_log_legend();

  if (_line_count == 0) {
    Serial.println(">>> No log captured yet (run track on first)");
    return;
  }

  Serial.print(">>> --- RAM log dump start (");
  Serial.print(_line_count);
  Serial.println(" lines, most recent track on so far) ---");

  for (size_t c = 0; c <= _cur_chunk && c < _chunk_count; c++) {
    Serial.write((const uint8_t*)_chunks[c], _chunk_used[c]);
    yield();  // 一次性输出可能有几十KB，115200波特率下要传输一段时间，不yield会喂不到任务看门狗导致复位
  }
  Serial.println(">>> --- RAM log dump end ---");
}

// 把内存里当前的内容写进flash，当"最新一趟"的备份用（配合调用方在这之前先print_file_clear()，
// 就能做到"flash里永远只留最新一次的log"）。这个函数只负责写，不清空RAM——RAM的生命周期
// 完全由ram_log_begin()自己管，写没写过flash不影响"log dump"继续从RAM读到内容
void ram_log_flush_to_file() {
  if (_chunk_count == 0 || _line_count == 0) return;

  if (!print_file_enabled()) print_set_file(true);  // 确保这次真的落盘，不依赖用户提前手动开

  char line[LOG_LINE_MAX];
  for (size_t c = 0; c <= _cur_chunk && c < _chunk_count; c++) {
    const char* buf = _chunks[c];
    size_t len = _chunk_used[c];
    size_t i = 0;
    while (i < len) {
      size_t start = i;
      while (i < len && buf[i] != '\n') i++;
      if (i < len) i++;  // 把'\n'算进这一行
      size_t n = i - start;
      if (n >= sizeof(line)) n = sizeof(line) - 1;
      memcpy(line, buf + start, n);
      line[n] = '\0';
      out_file(line);
    }
  }
  print_file_flush();
}

unsigned long ram_log_line_count() {
  return _line_count;
}
