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
// 默认值，运行时可通过sharpratio命令调整（供config_module持久化用）
#define TURN_RATIO_SHARP_DEFAULT (-0.3f)
// 急弯时外轮速度比例（占base的百分比，0.0~1.0，运行时可调，默认值仅为起调参考）
// 目的：急弯只反转内轮、外轮仍全速时车身带着直道动能冲进弯道容易冲出赛道，
// 外轮同步降速能减少入弯动能，配合内轮反转更容易在R70内掉头
#define TURN_OUTER_RATIO_DEFAULT 0.65f
// 丢线后延续最后转向方向找线的超时(ms)，超时未找回则停车
#define LOST_TIMEOUT_MS 1500

// 调试log节流间隔(ms)，定位问题用，确认后可调大或删除
#define DEBUG_INTERVAL_MS 30

// PID模式默认增益，均为起调参考值，未实车验证过，需实车试调
// 误差用sensor_position_from()的-1.0~+1.0，输出直接是左右轮PWM的差速修正量（与base同量纲）
#define PID_KP_DEFAULT 40.0f
#define PID_KI_DEFAULT 0.0f
#define PID_KD_DEFAULT 5.0f
// 积分限幅，防止长时间小误差把积分项攒到失控（抗积分饱和），限幅值同样未实车验证
#define PID_INTEGRAL_CLAMP 2.0f

static bool _on = false;
static unsigned long _last_dbg = 0;
static int _last_dir = 0;          // -1=上次左转  0=无记录  +1=上次右转
static unsigned long _lost_since = 0;
static float _turn_outer_ratio = TURN_OUTER_RATIO_DEFAULT;
static float _sharp_ratio = TURN_RATIO_SHARP_DEFAULT;

static int _algo = TRACK_ALGO_BANGBANG;
static float _pid_kp = PID_KP_DEFAULT;
static float _pid_ki = PID_KI_DEFAULT;
static float _pid_kd = PID_KD_DEFAULT;
static float _pid_integral = 0.0f;
static float _pid_last_error = 0.0f;
static unsigned long _pid_last_ms = 0;

static void pid_reset() {
  _pid_integral = 0.0f;
  _pid_last_error = 0.0f;
  _pid_last_ms = 0;
}

static int clamp_pwm(int v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return v;
}

float track_get_turn_ratio() {
  return _turn_outer_ratio;
}

void track_set_turn_ratio(float ratio) {
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  _turn_outer_ratio = ratio;
}

float track_get_sharp_ratio() {
  return _sharp_ratio;
}

void track_set_sharp_ratio(float ratio) {
  if (ratio < -1.0f) ratio = -1.0f;
  if (ratio > 0.0f) ratio = 0.0f;
  _sharp_ratio = ratio;
}

int track_get_algo() {
  return _algo;
}

void track_set_algo(int algo) {
  if (algo != TRACK_ALGO_BANGBANG && algo != TRACK_ALGO_PID) algo = TRACK_ALGO_BANGBANG;
  _algo = algo;
  pid_reset();   // 切换算法时清掉上一次的积分/微分历史，避免用旧模式的误差历史误导新模式
}

float track_get_pid_kp() { return _pid_kp; }
void  track_set_pid_kp(float kp) { if (kp < 0.0f) kp = 0.0f; _pid_kp = kp; }
float track_get_pid_ki() { return _pid_ki; }
void  track_set_pid_ki(float ki) { if (ki < 0.0f) ki = 0.0f; _pid_ki = ki; }
float track_get_pid_kd() { return _pid_kd; }
void  track_set_pid_kd(float kd) { if (kd < 0.0f) kd = 0.0f; _pid_kd = kd; }

void track_begin() {
  _on = false;
  _last_dir = 0;
  _lost_since = 0;
  pid_reset();
}

void track_set(bool on) {
  _on = on;
  if (!on) {
    motor_stop();
    _last_dir = 0;
    _lost_since = 0;
  }
  pid_reset();   // 开启时重新起跑，关闭时清掉残留状态，两种情况都不该带着旧积分继续算
}

bool track_is_on() {
  return _on;
}

void track_update() {
  if (!_on) return;

  bool is_white[SENSOR_COUNT];
  sensor_binary(is_white);

  // 5路顺序：CH2,CH3,CH4,CH5,CH6 → index 0~4
  // 传感器物理反装（180°翻转）：index 0(CH2)现在在右侧，index 4(CH6)在左侧，center不受影响
  bool sharp_l = is_white[4]; // CH6，物理左侧最外，急转
  bool mild_l  = is_white[3]; // CH5，物理左侧，缓转
  bool center  = is_white[2]; // CH4
  bool mild_r  = is_white[1]; // CH3，物理右侧，缓转
  bool sharp_r = is_white[0]; // CH2，物理右侧最外，急转

  bool lost = !sharp_l && !mild_l && !center && !mild_r && !sharp_r;
  // 十字路口/宽线：左右两侧同时压线，视为直行穿过，不触发转向
  bool cross = (sharp_l || mild_l) && (sharp_r || mild_r);

  int base = motor_level_to_pwm(config_get_speed());
  int mild_turn  = (int)(base * TURN_RATIO_MILD);
  int sharp_turn = (int)(base * _sharp_ratio);

  int pwm_l = base;
  int pwm_r = base;
  const char* mode = "STRAIGHT";

  unsigned long now = millis();

  if (lost) {
    if (_lost_since == 0) _lost_since = now;
    pid_reset();   // 丢线期间不是连续跟踪，清掉积分/微分历史，避免重新压线后带着丢线期间的误差冲一把
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
  } else if (_algo == TRACK_ALGO_PID) {
    _lost_since = 0;
    // 误差=加权位置，setpoint=0（居中）；正=线偏右，需要向右修正（左轮加速/右轮减速）
    float error = sensor_position_from(is_white);
    unsigned long dt_ms = (_pid_last_ms == 0) ? DEBUG_INTERVAL_MS : (now - _pid_last_ms);
    float dt = dt_ms > 0 ? dt_ms / 1000.0f : 0.001f;
    _pid_last_ms = now;

    _pid_integral += error * dt;
    if (_pid_integral > PID_INTEGRAL_CLAMP) _pid_integral = PID_INTEGRAL_CLAMP;
    if (_pid_integral < -PID_INTEGRAL_CLAMP) _pid_integral = -PID_INTEGRAL_CLAMP;

    float derivative = (error - _pid_last_error) / dt;
    _pid_last_error = error;

    float output = _pid_kp * error + _pid_ki * _pid_integral + _pid_kd * derivative;
    pwm_l = clamp_pwm(base + (int)output);
    pwm_r = clamp_pwm(base - (int)output);
    mode = "PID";
    if (error > 0.0f) _last_dir = 1;
    else if (error < 0.0f) _last_dir = -1;
  } else {
    _lost_since = 0;
    if (sharp_l) {
      pwm_l = sharp_turn;   // CH6压线，急转（内轮反转）
      pwm_r = (int)(base * _turn_outer_ratio);  // 外轮同步降速，减少入弯动能
      mode = "SHARP_L";
      _last_dir = -1;
    } else if (mild_l) {
      pwm_l = mild_turn;    // CH5压线，缓转
      mode = "LEFT";
      _last_dir = -1;
    } else if (sharp_r) {
      pwm_r = sharp_turn;   // CH2压线，急转（内轮反转）
      pwm_l = (int)(base * _turn_outer_ratio);  // 外轮同步降速，减少入弯动能
      mode = "SHARP_R";
      _last_dir = 1;
    } else if (mild_r) {
      pwm_r = mild_turn;    // CH3压线，缓转
      mode = "RIGHT";
      _last_dir = 1;
    }
    // CH4单独压线：直行，不更新_last_dir
  }

  motor_set(pwm_l, pwm_r);

  // 调试log：时间戳+5路W/B图案+判定模式+实际下发PWM，定位急弯失效用
  // PID模式额外附上当前误差(E:)，方便实车调Kp/Ki/Kd时对照
  if (now - _last_dbg >= DEBUG_INTERVAL_MS) {
    _last_dbg = now;
    char buf[112];
    // 打印顺序按物理左→右排列（index 4→0），与实车左右保持一致
    if (_algo == TRACK_ALGO_PID) {
      snprintf(buf, sizeof(buf), "T %6lu %c%c%c%c%c %-8s L:%4d R:%4d E:%+.2f\r\n",
               now,
               is_white[4] ? 'W' : 'B',
               is_white[3] ? 'W' : 'B',
               is_white[2] ? 'W' : 'B',
               is_white[1] ? 'W' : 'B',
               is_white[0] ? 'W' : 'B',
               mode, pwm_l, pwm_r, _pid_last_error);
    } else {
      snprintf(buf, sizeof(buf), "T %6lu %c%c%c%c%c %-8s L:%4d R:%4d\r\n",
               now,
               is_white[4] ? 'W' : 'B',
               is_white[3] ? 'W' : 'B',
               is_white[2] ? 'W' : 'B',
               is_white[1] ? 'W' : 'B',
               is_white[0] ? 'W' : 'B',
               mode, pwm_l, pwm_r);
    }
    out(buf);
  }
}
