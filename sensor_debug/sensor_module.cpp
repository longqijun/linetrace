#include "sensor_module.h"
#include <Arduino.h>
#include <math.h>

static const int PINS[SENSOR_COUNT] = {33, 34, 35, 36, 39};
// 各路独立阈值（黑白中点）：CH6偏弱单独设低，代码内默认值；可被config_module加载的/config.json覆盖
static int THRESHOLD[SENSOR_COUNT] = {1545, 1580, 1422, 1400, 1115};

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

// 加权位置：传感器索引0~4，中心为2，归一化到-1~+1
// 传感器物理反装（180°翻转）：index 0(CH2)现在在右侧，符号取反使-1仍代表物理最左
// 纯函数版本：接收调用方已采样好的is_white[]，不重新读ADC
// （给track_module的PID模式用，避免同一个loop周期里对5路重复采样两次）
float sensor_position_from(const bool is_white[SENSOR_COUNT]) {
  int sum = 0, count = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (is_white[i]) {
      sum += i;
      count++;
    }
  }
  if (count == 0) return NAN;          // 丢线
  return (2 - sum / (float)count) / 2; // 归一化：中心=0，符号反转对应物理反装
}

float sensor_position() {
  bool is_white[SENSOR_COUNT];
  sensor_binary(is_white);
  return sensor_position_from(is_white);
}
