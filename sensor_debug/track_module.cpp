#include "track_module.h"
#include "sensor_module.h"
#include "motor_module.h"
#include "config_module.h"
#include "print_module.h"
#include <Arduino.h>
#include <stdio.h>

// 差速转向：内侧轮减速比例（0.0~1.0），越小转向越急
#define TURN_RATIO 0.5f

// 调试log节流间隔(ms)，定位问题用，确认后可调大或删除
#define DEBUG_INTERVAL_MS 30

static bool _on = false;
static unsigned long _last_dbg = 0;

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
  const char* mode = "STRAIGHT";

  if (left && !right) {
    pwm_l = turn;   // 线偏左，左轮减速，向左修正
    mode = "LEFT";
  } else if (right && !left) {
    pwm_r = turn;   // 线偏右，右轮减速，向右修正
    mode = "RIGHT";
  }
  // CH4压线或左右同时触发（宽线/丢线）：直行，暂不处理丢线找回

  motor_set(pwm_l, pwm_r);

  // 调试log：时间戳+5路W/B图案+判定模式+实际下发PWM，定位急弯失效用
  unsigned long now = millis();
  if (now - _last_dbg >= DEBUG_INTERVAL_MS) {
    _last_dbg = now;
    char buf[96];
    snprintf(buf, sizeof(buf), "T %6lu %c%c%c%c%c %-8s L:%4d R:%4d\r\n",
             now,
             is_white[0] ? 'W' : 'B',
             is_white[1] ? 'W' : 'B',
             is_white[2] ? 'W' : 'B',
             is_white[3] ? 'W' : 'B',
             is_white[4] ? 'W' : 'B',
             mode, pwm_l, pwm_r);
    out(buf);
  }
}
