#include "track_module.h"
#include "sensor_module.h"
#include "motor_module.h"
#include "config_module.h"
#include <Arduino.h>

// 差速转向：内侧轮减速比例（0.0~1.0），越小转向越急
#define TURN_RATIO 0.5f

static bool _on = false;

void track_begin() {
  _on = false;
}

void track_set(bool on) {
  _on = on;
  if (!on) motor_stop();
}

bool track_is_on() {
  return _on;
}

void track_update() {
  if (!_on) return;

  bool is_white[SENSOR_COUNT];
  sensor_binary(is_white);

  // 5路顺序：CH2,CH3,CH4,CH5,CH6 → index 0~4
  bool left   = is_white[1]; // CH3
  bool right  = is_white[3]; // CH5

  int base = motor_level_to_pwm(config_get_speed());
  int turn = (int)(base * TURN_RATIO);

  int pwm_l = base;
  int pwm_r = base;

  if (left && !right) {
    pwm_l = turn;   // 线偏左，左轮减速，向左修正
  } else if (right && !left) {
    pwm_r = turn;   // 线偏右，右轮减速，向右修正
  }
  // CH4压线或左右同时触发（宽线/丢线）：直行，暂不处理丢线找回

  motor_set(pwm_l, pwm_r);
}
