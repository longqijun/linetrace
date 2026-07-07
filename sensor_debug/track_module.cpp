#include "track_module.h"
#include "sensor_module.h"
#include "motor_module.h"
#include "config_module.h"
#include "print_module.h"
#include <Arduino.h>
#include <stdio.h>

// 差速转向：CH3/CH5触发的缓转，内侧轮减速比例（0.0~1.0），越小转向越急
#define TURN_RATIO_MILD  0.5f
// CH2/CH6触发的急转，内侧轮反转比例（负值=反转），用于R70这类极小半径弯，需实车试调
#define TURN_RATIO_SHARP (-0.3f)
// 丢线后延续最后转向方向找线的超时(ms)，超时未找回则停车
#define LOST_TIMEOUT_MS 1500

// 调试log节流间隔(ms)，定位问题用，确认后可调大或删除
#define DEBUG_INTERVAL_MS 30

static bool _on = false;
static unsigned long _last_dbg = 0;
static int _last_dir = 0;          // -1=上次左转  0=无记录  +1=上次右转
static unsigned long _lost_since = 0;

void track_begin() {
  _on = false;
  _last_dir = 0;
  _lost_since = 0;
}

void track_set(bool on) {
  _on = on;
  if (!on) {
    motor_stop();
    _last_dir = 0;
    _lost_since = 0;
  }
}

bool track_is_on() {
  return _on;
}

void track_update() {
  if (!_on) return;

  bool is_white[SENSOR_COUNT];
  sensor_binary(is_white);

  // 5路顺序：CH2,CH3,CH4,CH5,CH6 → index 0~4
  bool sharp_l = is_white[0]; // CH2，最外侧，急转
  bool mild_l  = is_white[1]; // CH3，缓转
  bool center  = is_white[2]; // CH4
  bool mild_r  = is_white[3]; // CH5，缓转
  bool sharp_r = is_white[4]; // CH6，最外侧，急转

  bool lost = !sharp_l && !mild_l && !center && !mild_r && !sharp_r;
  // 十字路口/宽线：左右两侧同时压线，视为直行穿过，不触发转向
  bool cross = (sharp_l || mild_l) && (sharp_r || mild_r);

  int base = motor_level_to_pwm(config_get_speed());
  int mild_turn  = (int)(base * TURN_RATIO_MILD);
  int sharp_turn = (int)(base * TURN_RATIO_SHARP);

  int pwm_l = base;
  int pwm_r = base;
  const char* mode = "STRAIGHT";

  unsigned long now = millis();

  if (lost) {
    if (_lost_since == 0) _lost_since = now;
    if (now - _lost_since > LOST_TIMEOUT_MS) {
      pwm_l = 0;
      pwm_r = 0;
      mode = "LOST_STOP";
    } else if (_last_dir < 0) {
      pwm_l = sharp_turn;   // 延续上次左转方向找线
      mode = "LOST_L";
    } else if (_last_dir > 0) {
      pwm_r = sharp_turn;   // 延续上次右转方向找线
      mode = "LOST_R";
    }
  } else if (cross) {
    _lost_since = 0;
    mode = "CROSS";   // 十字路口/宽线，直行穿过，不更新_last_dir
  } else {
    _lost_since = 0;
    if (sharp_l) {
      pwm_l = sharp_turn;   // CH2压线，急转（内轮反转）
      mode = "SHARP_L";
      _last_dir = -1;
    } else if (mild_l) {
      pwm_l = mild_turn;    // CH3压线，缓转
      mode = "LEFT";
      _last_dir = -1;
    } else if (sharp_r) {
      pwm_r = sharp_turn;   // CH6压线，急转（内轮反转）
      mode = "SHARP_R";
      _last_dir = 1;
    } else if (mild_r) {
      pwm_r = mild_turn;    // CH5压线，缓转
      mode = "RIGHT";
      _last_dir = 1;
    }
    // CH4单独压线：直行，不更新_last_dir
  }

  motor_set(pwm_l, pwm_r);

  // 调试log：时间戳+5路W/B图案+判定模式+实际下发PWM，定位急弯失效用
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
