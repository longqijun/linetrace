#include "sensor_module.h"
#include <Arduino.h>
#include <math.h>

// 索引i对应CH(i+1)，即index0=CH1...index7=CH8（8路扩展方案，见"8路传感器方案.md"）
static const int PINS[SENSOR_COUNT] = {32, 33, 34, 35, 36, 39, 13, 14};
// 各路独立阈值（黑白中点）：CH2~CH6是已实测校准值（代码内默认值，实际跑的是config.json里
// save过的值，见设计说明.md）；CH1/CH7/CH8是全新接入的3路，下面是未校准的占位值，
// 实车必须先"print on"看原始ADC白/黑值、再用"threshold"命令逐路标定，否则这3路判断不可信
static int THRESHOLD[SENSOR_COUNT] = {1545 /*CH1,占位未校准*/, 1545, 1580, 1422, 1400, 1115,
                                       1500 /*CH7,占位未校准*/, 1500 /*CH8,占位未校准*/};

void sensor_begin() {
  analogReadResolution(12);
}

void sensor_read(int values[SENSOR_COUNT]) {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    values[i] = analogRead(PINS[i]);
  }
}

void sensor_binary(bool is_white[SENSOR_COUNT]) {
  int vals[SENSOR_COUNT];
  sensor_read(vals);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    is_white[i] = (vals[i] < THRESHOLD[i]);
  }
}

int sensor_get_threshold(int index) {
  return THRESHOLD[index];
}

void sensor_set_threshold(int index, int value) {
  if (index < 0 || index >= SENSOR_COUNT) return;
  THRESHOLD[index] = value;
}

// 加权位置：传感器索引0~(SENSOR_COUNT-1)，中心为(SENSOR_COUNT-1)/2，归一化到-1~+1
// （5路时中心=2，8路时中心=3.5，公式通用不依赖具体路数，见"8路传感器方案.md"第5节）
// 传感器物理反装（180°翻转）：index 0(CH1)现在在最右侧，符号取反使-1仍代表物理最左
// 纯函数版本：接收调用方已采样好的is_white[]，不重新读ADC
// （给track_module的PID模式用，避免同一个loop周期里对多路传感器重复采样两次）
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
