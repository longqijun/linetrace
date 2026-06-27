#include "sensor_module.h"
#include <Arduino.h>
#include <math.h>

static const int PINS[SENSOR_COUNT]      = {33, 34, 35, 36, 39};
// 各路独立阈值（黑白中点）：CH6偏弱单独设低
static const int THRESHOLD[SENSOR_COUNT] = {1545, 1580, 1422, 1400, 1115};

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

// 加权位置：传感器索引0~4，中心为2，归一化到-1~+1
float sensor_position() {
  bool is_white[SENSOR_COUNT];
  sensor_binary(is_white);

  int sum = 0, count = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (is_white[i]) {
      sum += i;
      count++;
    }
  }
  if (count == 0) return NAN;          // 丢线
  return (sum / (float)count - 2) / 2; // 归一化：中心=0
}
