#include "track_module.h"
#include "sensor_module.h"
#include "motor_module.h"
#include "config_module.h"
#include "print_module.h"
#include <Arduino.h>
#include <stdio.h>
#include <math.h>

// 差速转向：CH3/CH5触发的缓转，内侧轮减速比例（0.0~1.0），越小转向越急
#define TURN_RATIO_MILD  0.7f
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
// PID计算周期(ms)，跟loop()本身的实际速度（实测约1ms一圈，见设计文档）解耦。
// 传感器位置只有9档离散值，loop()多快就重算一次PID、多快就换一次电机PWM，
// 对着一个离散信号在1ms尺度上求微分/频繁变更输出，本质是在放大传感器噪声，
// 跟Kd是否为0无关（纯P也会跟着这个节奏抖）。固定周期后dt恒定，微分才有意义
#define PID_INTERVAL_MS 10
// 微分项一阶低通滤波系数（0~1，越小滤波越强/越迟钝）。
// 误差是离散阶梯值，压线瞬间从一档跳到另一档时，未滤波的微分会在单个采样点上
// 炸出远超真实变化率的尖峰（"微分冲击"），滤波把这个跳变摊到后面几个周期
#define PID_DERIV_FILTER_ALPHA 0.3f

// PWM限速：下发给电机的pwm_l/pwm_r每秒最多变化这么多单位（0~255量程），运行时可通过
// slewrate命令调整。目的：bang-bang各档（STRAIGHT/mild/sharp）之间PWM差距很大且是硬跳变
// （比如从base=102直接跳到sharp_turn=-40反转），车身在检测边界附近来回穿越时会跟着硬来回
// 打，表现为"左右摇摆剧烈"；PID模式下也一样受益（模式切换/误差跳档时同样是硬跳变）。
// 限速后目标PWM需要一点时间才能到位，把这种硬摇摆摊开、变平滑。数值越大越接近不限速。
#define SLEW_RATE_DEFAULT 800.0f

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
static float _pid_last_error = 0.0f;       // 上一次参与计算的误差（仅供debug打印+下次求导用）
static float _pid_filtered_deriv = 0.0f;   // 低通滤波后的微分值
static unsigned long _pid_last_ms = 0;     // 上一次真正重算PID的时刻，配合PID_INTERVAL_MS节流
static int _pid_hold_output = 0;           // 上一次算出的差速修正量，未到计算周期时沿用，不跟着loop()抖

static float _slew_rate = SLEW_RATE_DEFAULT;
static float _slew_pwm_l = 0.0f;           // 限速后当前实际下发的pwm_l/r（浮点存，避免整数截断累积误差）
static float _slew_pwm_r = 0.0f;
static unsigned long _slew_last_ms = 0;    // 上一次做限速计算的时刻，用于算dt

static void pid_reset() {
  _pid_integral = 0.0f;
  _pid_last_error = 0.0f;
  _pid_filtered_deriv = 0.0f;
  _pid_last_ms = 0;
  _pid_hold_output = 0;
}

static void slew_reset() {
  _slew_pwm_l = 0.0f;
  _slew_pwm_r = 0.0f;
  _slew_last_ms = 0;
}

// 把target_l/target_r限速逼近，每次调用最多让当前值朝目标值移动
// _slew_rate*dt个单位；dt是这次调用距上次调用的真实间隔（ms）
static void slew_toward(int target_l, int target_r, int* out_l, int* out_r) {
  unsigned long now = millis();
  float dt = (_slew_last_ms == 0) ? 0.0f : (now - _slew_last_ms) / 1000.0f;
  _slew_last_ms = now;

  float max_step = _slew_rate * dt;

  float diff_l = target_l - _slew_pwm_l;
  if (diff_l > max_step) diff_l = max_step;
  if (diff_l < -max_step) diff_l = -max_step;
  _slew_pwm_l += diff_l;

  float diff_r = target_r - _slew_pwm_r;
  if (diff_r > max_step) diff_r = max_step;
  if (diff_r < -max_step) diff_r = -max_step;
  _slew_pwm_r += diff_r;

  *out_l = (int)lroundf(_slew_pwm_l);
  *out_r = (int)lroundf(_slew_pwm_r);
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

float track_get_slew_rate() { return _slew_rate; }
void  track_set_slew_rate(float rate) { if (rate < 1.0f) rate = 1.0f; _slew_rate = rate; }

void track_begin() {
  _on = false;
  _last_dir = 0;
  _lost_since = 0;
  pid_reset();
  slew_reset();
}

void track_set(bool on) {
  _on = on;
  if (!on) {
    motor_stop();
    _last_dir = 0;
    _lost_since = 0;
  }
  pid_reset();   // 开启时重新起跑，关闭时清掉残留状态，两种情况都不该带着旧积分继续算
  slew_reset();  // 限速状态同样清零，避免开关之间残留的旧pwm值影响下一次起跑

  // 只写文件通道：命令/按钮各自已经通过reply()把ON/OFF提示发到USB+BT了，这里再走out()
  // 会重复；但track_update()关闭时完全不打印，file log里没有别的地方能留下OFF的痕迹，
  // 所以单独给文件通道补一条，配合#ID可以准确定位一次track on~off覆盖的record范围。
  // ON时顺带记下当前算法和增益：T行本身不直接说"现在是哪个算法"（只能靠有没有E:字段间接猜），
  // 事后翻log时不用再去猜这一段测试跑的时候config是什么状态
  char buf[96];
  if (on && _algo == TRACK_ALGO_PID) {
    snprintf(buf, sizeof(buf), ">>> TRACK_ON t=%lu algo=PID speed=%d kp=%.2f ki=%.2f kd=%.2f\r\n",
             millis(), config_get_speed(), _pid_kp, _pid_ki, _pid_kd);
  } else if (on) {
    snprintf(buf, sizeof(buf), ">>> TRACK_ON t=%lu algo=BANGBANG speed=%d turn_ratio=%.2f sharp_ratio=%.2f\r\n",
             millis(), config_get_speed(), _turn_outer_ratio, _sharp_ratio);
  } else {
    snprintf(buf, sizeof(buf), ">>> TRACK_OFF t=%lu\r\n", millis());
  }
  out_file(buf);
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
  bool stop_now = false;  // true时跳过限速直接停车（安全优先，丢线超时停车不该被限速拖慢）

  unsigned long now = millis();

  if (lost) {
    if (_lost_since == 0) _lost_since = now;
    pid_reset();   // 丢线期间不是连续跟踪，清掉积分/微分历史，避免重新压线后带着丢线期间的误差冲一把
    if (now - _lost_since > LOST_TIMEOUT_MS) {
      pwm_l = 0;
      pwm_r = 0;
      mode = "LOST_STOP";
      stop_now = true;
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

    // PID只按固定周期重算，中间这些loop沿用_pid_hold_output——
    // 传感器位置只有9档离散值，跟着loop()裸奔重算等于在1ms尺度上对着一个台阶信号求导，
    // 算出来的全是噪声放大，跟Kd是否为0无关。固定周期后dt恒定，微分才有意义
    if (_pid_last_ms == 0 || now - _pid_last_ms >= PID_INTERVAL_MS) {
      float dt = (_pid_last_ms == 0) ? (PID_INTERVAL_MS / 1000.0f) : (now - _pid_last_ms) / 1000.0f;
      _pid_last_ms = now;

      _pid_integral += error * dt;
      if (_pid_integral > PID_INTEGRAL_CLAMP) _pid_integral = PID_INTEGRAL_CLAMP;
      if (_pid_integral < -PID_INTEGRAL_CLAMP) _pid_integral = -PID_INTEGRAL_CLAMP;

      float raw_derivative = (error - _pid_last_error) / dt;
      _pid_last_error = error;
      // 一阶低通滤波：把离散误差跳档瞬间产生的微分尖峰摊到后面几个周期，避免瞬间打满
      _pid_filtered_deriv += PID_DERIV_FILTER_ALPHA * (raw_derivative - _pid_filtered_deriv);

      float output = _pid_kp * error + _pid_ki * _pid_integral + _pid_kd * _pid_filtered_deriv;
      _pid_hold_output = (int)lroundf(output);  // 四舍五入而非向零截断，小误差不会被直接吃成死区

      if (error > 0.0f) _last_dir = 1;
      else if (error < 0.0f) _last_dir = -1;
    }

    pwm_l = clamp_pwm(base + _pid_hold_output);
    pwm_r = clamp_pwm(base - _pid_hold_output);
    mode = "PID";
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

  // PWM限速：把目标值平滑逼近，压住bang-bang/PID在检测边界附近来回穿越时的硬摇摆。
  // stop_now（丢线超时停车）例外，安全优先，直接停，不走限速
  if (stop_now) {
    slew_reset();
  } else {
    int slewed_l, slewed_r;
    slew_toward(pwm_l, pwm_r, &slewed_l, &slewed_r);
    pwm_l = slewed_l;
    pwm_r = slewed_r;
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
