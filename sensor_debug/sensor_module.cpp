#include "sensor_module.h"
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>   // malloc/free
#include <stdio.h>    // snprintf
#include <string.h>   // strcpy

// 索引i对应CH(i+1)，即index0=CH1...index7=CH8（8路扩展方案，见"8路传感器方案.md"）
static const int PINS[SENSOR_COUNT] = {32, 33, 34, 35, 36, 39, 13, 14};
// 各路独立阈值（黑白中点）：CH2~CH6是已实测校准值（代码内默认值，实际跑的是config.json里
// save过的值，见设计说明.md）；CH1/CH7/CH8是全新接入的3路，下面是未校准的占位值，
// 实车必须先"print on"看原始ADC白/黑值、再用"threshold"命令逐路标定，否则这3路判断不可信
static int THRESHOLD[SENSOR_COUNT] = {1545 /*CH1,占位未校准*/, 1545, 1580, 1422, 1400, 1115,
                                       1500 /*CH7,占位未校准*/, 1500 /*CH8,占位未校准*/};

// 白/黑参考值，给sensor_position_analog_from()做模拟量加权用（见sensor_module.h顶部说明）。
// CH2~CH6是设计说明.md记录过的实测白/黑值；CH1/CH7/CH8是全新接入的3路，同样是未校准占位值，
// 跟THRESHOLD一样必须实车用"print on"看原始ADC值、再用"whiteref"/"blackref"命令逐路标定
static int WHITE_REF[SENSOR_COUNT] = {600 /*CH1,占位未校准*/, 660, 660, 545, 600, 600,
                                       600 /*CH7,占位未校准*/, 600 /*CH8,占位未校准*/};
static int BLACK_REF[SENSOR_COUNT] = {2400 /*CH1,占位未校准*/, 2400, 2500, 2300, 2200, 2400,
                                       2400 /*CH7,占位未校准*/, 2400 /*CH8,占位未校准*/};

void sensor_begin() {
  analogReadResolution(12);
}

void sensor_read(int values[SENSOR_COUNT]) {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    values[i] = analogRead(PINS[i]);
  }
}

void sensor_binary_from(const int vals[SENSOR_COUNT], bool is_white[SENSOR_COUNT]) {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    is_white[i] = (vals[i] < THRESHOLD[i]);
  }
}

void sensor_binary(bool is_white[SENSOR_COUNT]) {
  int vals[SENSOR_COUNT];
  sensor_read(vals);
  sensor_binary_from(vals, is_white);
}

int sensor_get_threshold(int index) {
  return THRESHOLD[index];
}

void sensor_set_threshold(int index, int value) {
  if (index < 0 || index >= SENSOR_COUNT) return;
  THRESHOLD[index] = value;
}

int sensor_get_white_ref(int index) { return WHITE_REF[index]; }
void sensor_set_white_ref(int index, int value) {
  if (index < 0 || index >= SENSOR_COUNT) return;
  WHITE_REF[index] = value;
}
int sensor_get_black_ref(int index) { return BLACK_REF[index]; }
void sensor_set_black_ref(int index, int value) {
  if (index < 0 || index >= SENSOR_COUNT) return;
  BLACK_REF[index] = value;
}

// ================= 传感器阈值自动标定（扫描式，见"传感器阈值自动标定方案.md"方案B）=================
static int   CALIB_K            = 100;    // 极值群大小K（<=SENSOR_CALIB_MAX_K）
static float CALIB_RATIO        = 0.5f;   // threshold = white + (black-white)*ratio
static int   CALIB_SWEEP_SEC    = 5;      // 默认扫描秒数
static const int CALIB_MIN_CONTRAST = 300; // black-white 小于此值判为标定失败（对比度不足）

int   sensor_get_calib_k()            { return CALIB_K; }
void  sensor_set_calib_k(int k)       { if (k < 1) k = 1; if (k > SENSOR_CALIB_MAX_K) k = SENSOR_CALIB_MAX_K; CALIB_K = k; }
float sensor_get_calib_ratio()        { return CALIB_RATIO; }
void  sensor_set_calib_ratio(float r) { if (r < 0.0f) r = 0.0f; if (r > 1.0f) r = 1.0f; CALIB_RATIO = r; }
int   sensor_get_calib_sweep_sec()    { return CALIB_SWEEP_SEC; }
void  sensor_set_calib_sweep_sec(int s){ if (s < 1) s = 1; if (s > 60) s = 60; CALIB_SWEEP_SEC = s; }

// 插入到升序数组(保留最大的K个)：arr[0]=当前群里最小(淘汰边界)。去旧取新只看arr[0]。
static void calib_push_max(int* arr, int* count, int K, int x) {
  if (*count < K) {                       // 未满，插入排序放进去
    int i = *count - 1;
    while (i >= 0 && arr[i] > x) { arr[i + 1] = arr[i]; i--; }
    arr[i + 1] = x; (*count)++;
  } else if (x > arr[0]) {                 // 满了，x打败当前最小者才进：踢掉arr[0]再插入
    int i = 1;
    while (i < K && arr[i] < x) { arr[i - 1] = arr[i]; i++; }
    arr[i - 1] = x;
  }
}

// 插入到降序数组(保留最小的K个)：arr[0]=当前群里最大(淘汰边界)。镜像。
static void calib_push_min(int* arr, int* count, int K, int x) {
  if (*count < K) {
    int i = *count - 1;
    while (i >= 0 && arr[i] < x) { arr[i + 1] = arr[i]; i--; }
    arr[i + 1] = x; (*count)++;
  } else if (x < arr[0]) {
    int i = 1;
    while (i < K && arr[i] > x) { arr[i - 1] = arr[i]; i++; }
    arr[i - 1] = x;
  }
}

void sensor_calib_sweep(int seconds, void (*out)(const char*)) {
  if (seconds < 1) seconds = 1;
  if (seconds > 60) seconds = 60;
  const int K = CALIB_K;
  char buf[112];

  // 每路K个max + K个min，只在标定期间临时占堆(K=100时约6.4KB)，用完即free，不进永久内存
  int* maxK = (int*)malloc(sizeof(int) * SENSOR_COUNT * K);
  int* minK = (int*)malloc(sizeof(int) * SENSOR_COUNT * K);
  if (!maxK || !minK) {
    if (maxK) free(maxK);
    if (minK) free(minK);
    out(">>> calib: out of memory, try smaller K (calib k VALUE)\r\n");
    return;
  }
  int maxCount[SENSOR_COUNT], minCount[SENSOR_COUNT];
  for (int i = 0; i < SENSOR_COUNT; i++) { maxCount[i] = 0; minCount[i] = 0; }

  snprintf(buf, sizeof(buf), ">>> calib sweep: %ds @100Hz, K=%d. Move the car so every channel crosses black & white...\r\n",
           seconds, K);
  out(buf);

  // 采集：固定100Hz(10ms/tick)，用millis调度避免delay漂移；期间不跑巡线(调用方已停车)
  int N = 0;
  unsigned long t_end      = millis() + (unsigned long)seconds * 1000UL;
  unsigned long next_tick  = millis();
  unsigned long next_report= millis() + 1000UL;
  while ((long)(millis() - t_end) < 0) {
    unsigned long now = millis();
    if ((long)(now - next_tick) >= 0) {
      next_tick += 10;                     // 下一个10ms tick
      int vals[SENSOR_COUNT];
      sensor_read(vals);
      N++;
      for (int i = 0; i < SENSOR_COUNT; i++) {
        calib_push_max(maxK + i * K, &maxCount[i], K, vals[i]);
        calib_push_min(minK + i * K, &minCount[i], K, vals[i]);
      }
    }
    if ((long)(now - next_report) >= 0) {  // 每秒报一下剩余时间
      next_report += 1000;
      snprintf(buf, sizeof(buf), ">>> ...%lds left (N=%d)\r\n",
               (long)((t_end - now) / 1000) + 1, N);
      out(buf);
    }
  }

  // 结果表头（必须显示总样本数N，见方案第7.5节Q1）
  out(">>> Calibration result (memory only, use 'savesensor' to persist):\r\n");
  float nk = (K > 0) ? (float)N / K : 0.0f;
  snprintf(buf, sizeof(buf), ">>> Samples: N=%d (%.1fs @100Hz), K=%d   [N/K=%.1f, suggest N/K>=3]\r\n",
           N, N / 100.0f, K, nk);
  out(buf);
  out("CH  white  black  thr    contrast  nBlack  nWhite  status\r\n");

  // 逐路导出：black=最大群平均, white=最小群平均, 校验通过才写入(否则保留旧值)
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int mc = maxCount[i], wc = minCount[i];
    long bsum = 0, wsum = 0;
    for (int j = 0; j < mc; j++) bsum += maxK[i * K + j];
    for (int j = 0; j < wc; j++) wsum += minK[i * K + j];
    int black = (mc > 0) ? (int)(bsum / mc) : 0;
    int white = (wc > 0) ? (int)(wsum / wc) : 0;
    int contrast = black - white;
    int mid = (black + white) / 2;
    // nBlack/nWhite: 极值群里真正落在阈值正确一侧的样本数(上限K)，越接近K说明该路黑/白都扫够了
    int nBlack = 0, nWhite = 0;
    for (int j = 0; j < mc; j++) if (maxK[i * K + j] >= mid) nBlack++;
    for (int j = 0; j < wc; j++) if (minK[i * K + j] <= mid) nWhite++;

    bool ok = (black > white) && (contrast >= CALIB_MIN_CONTRAST);
    char thr_s[8];
    const char* status;
    if (ok) {
      int thr = white + (int)lroundf(contrast * CALIB_RATIO);
      sensor_set_white_ref(i, white);
      sensor_set_black_ref(i, black);
      sensor_set_threshold(i, thr);
      snprintf(thr_s, sizeof(thr_s), "%d", thr);
      status = "OK";
    } else {
      strcpy(thr_s, "----");
      status = (black <= white) ? "FAIL(polarity)" : "FAIL(low contrast)";
    }
    snprintf(buf, sizeof(buf), "%-3d %-6d %-6d %-6s %-9d %-7d %-7d %s\r\n",
             i + 1, white, black, thr_s, contrast, nBlack, nWhite, status);
    out(buf);
  }

  free(maxK);
  free(minK);
  out(">>> Done. 'savesensor' to persist; re-run 'calib sweep' if any channel FAILed.\r\n");
}

// 加权位置（基于二值判断，离散阶梯值）：传感器索引0~(SENSOR_COUNT-1)，中心为(SENSOR_COUNT-1)/2，
// 归一化到-1~+1（5路时中心=2，8路时中心=3.5，公式通用不依赖具体路数，见"8路传感器方案.md"第5节）
// 传感器物理反装（180°翻转）：index 0(CH1)现在在最右侧，符号取反使-1仍代表物理最左
// 纯函数版本：接收调用方已采样好的is_white[]，不重新读ADC
// PID模式2026-07-29后改用下方的sensor_position_analog_from()，这个函数继续给bang-bang的
// lost/cross判定和调试打印用
float sensor_position_from(const bool is_white[SENSOR_COUNT]) {
  int sum = 0, count = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (is_white[i]) {
      sum += i;
      count++;
    }
  }
  if (count == 0) return NAN;          // 丢线
  const float center_index = (SENSOR_COUNT - 1) / 2.0f;
  return (center_index - sum / (float)count) / center_index; // 归一化：中心=0，符号反转对应物理反装
}

float sensor_position() {
  bool is_white[SENSOR_COUNT];
  sensor_binary(is_white);
  return sensor_position_from(is_white);
}

// 模拟量加权位置：每路先按(黑参考值-原始值)/(黑参考值-白参考值)算"在线权重"，
// 压线中心=1.0（原始值等于白参考值），完全没压到线=0.0（原始值等于黑参考值），
// 中间是连续的比例，不再是二值的开/关——离白线越近，权重越接近1，是慢慢变化的，
// 不是像is_white[]那样"跳档"。再按权重加权平均各路物理索引，公式跟
// sensor_position_from()的中心/归一化逻辑一致，只是拿连续权重代替0/1计数。
// 纯函数版本：接收调用方已读的原始ADC值，不重新读
float sensor_position_analog_from(const int vals[SENSOR_COUNT]) {
  float sum_weighted = 0.0f, sum_weight = 0.0f;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    float range = (float)(BLACK_REF[i] - WHITE_REF[i]);
    float w = (range != 0.0f) ? (BLACK_REF[i] - vals[i]) / range : 0.0f;
    if (w < 0.0f) w = 0.0f;   // 比黑参考值还黑（比如没标定/离得更远），当作0权重
    if (w > 1.0f) w = 1.0f;   // 比白参考值还白，封顶1.0
    sum_weighted += w * i;
    sum_weight += w;
  }
  // 权重总和太小，视为丢线；0.3是起调参考值（约等于"不到1路传感器完全压线"的量级），未实车验证
  if (sum_weight < 0.3f) return NAN;
  float avg_index = sum_weighted / sum_weight;
  const float center_index = (SENSOR_COUNT - 1) / 2.0f;
  return (center_index - avg_index) / center_index;
}

float sensor_position_analog() {
  int vals[SENSOR_COUNT];
  sensor_read(vals);
  return sensor_position_analog_from(vals);
}
