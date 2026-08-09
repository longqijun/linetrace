#include "track_module.h"
#include "sensor_module.h"
#include "motor_module.h"
#include "config_module.h"
#include "print_module.h"
#include "ram_log_module.h"
#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// 8路传感器差速bang-bang（见"8路传感器方案.md"）：中心两路直行 + 对称三档转向
// CH4/CH5压线(任一或两路)→直行(不改变轮速)；CH3/CH6一档(medium) < CH2/CH7二档(sharp) < CH1/CH8三档(hairpin)
// 由外到内依次判定，命中最外层就不再看内层；只有中心两路压线(或全不命中)才直行

// CH3/CH6触发的一档中转，内侧轮减速比例（不反转），纯起调猜测未实车验证
// 默认值，运行时可通过mediumratio命令调整（供config_module持久化用）
#define TURN_RATIO_MEDIUM_DEFAULT 0.35f
// CH2/CH7触发的急转，内侧轮反转比例（负值=反转），用于R70这类极小半径弯，需实车试调
// 默认值，运行时可通过sharpratio命令调整（供config_module持久化用）
#define TURN_RATIO_SHARP_DEFAULT (-0.3f)
// CH1/CH8触发的发卡弯（新增，比sharp更激进的反转），内侧轮反转比例，纯起调猜测未实车验证
// 默认值，运行时可通过xsharpratio命令调整（供config_module持久化用）
#define TURN_RATIO_XSHARP_DEFAULT (-0.6f)
// 急弯（sharp/hairpin共用）时外轮速度比例（占base的百分比，0.0~1.0，运行时可调，默认值仅为起调参考）
// 目的：急弯只反转内轮、外轮仍全速时车身带着直道动能冲进弯道容易冲出赛道，
// 外轮同步降速能减少入弯动能，配合内轮反转更容易在R70内掉头
#define TURN_OUTER_RATIO_DEFAULT 0.65f

// 方案A（弯道降速，见"log分析/log17分析.md"第七节）：bang-bang按档位降低前进速度——
// 弯越急，整档base降得越多，让车进弯前先收速、给机械留出转向时间，避免以直线速度冲进弯冲到最外侧丢线。
// 系数=占满速base的比例(0~1)，越小越慢。内轮/外轮都基于"降速后的base"算，整档一起慢。
// 只作用于bang-bang，PID分支与丢线找线(LOST)不受影响。三个系数运行时可调(mediumspeed/sharpspeed/hairpinspeed)。
#define TURN_SPEED_MEDIUM_DEFAULT  0.85f  // MEDIUM档(CH3/CH6)前进速度系数
#define TURN_SPEED_SHARP_DEFAULT   0.65f  // SHARP档(CH2/CH7)
#define TURN_SPEED_HAIRPIN_DEFAULT 0.50f  // HAIRPIN档(CH1/CH8)

// 电机最小前进PWM(静摩擦死区地板)：实测speed<16(即PWM<~102)起步转不动；但运动中的外轮实测70能持续转，
// 故默认取70作"运动中不失速"地板。方案A把外轮降速后若低于此值会被抬回，避免弯道里外轮堵转停车。
// 只保护"必须持续前进"的外轮；反转的内轮(转向动作)不受此地板约束。运行时可调(minpwm命令)。
#define MOTOR_MIN_MOVE_PWM_DEFAULT 70
// 丢线后延续最后转向方向找线的超时(ms)，超时未找回则停车
#define LOST_TIMEOUT_MS 1500

// 内存日志采样周期(ms)：档位没变化时，每隔这么久也补一条记录。
// 运行时可调(lograte命令)：越小越密(近乎连续轨迹)，越大越省内存。见"LOG高频采样方案.md"。
// 默认10ms=整趟连续轨迹又几乎不漏；1=每loop级(抓瞬态,内存约1圈);500=旧的省内存兜底。
#define LOG_INTERVAL_DEFAULT 10

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
// slewrate命令调整。目的：bang-bang各档（STRAIGHT/一档/二档/三档）之间PWM差距很大且是硬跳变
// （比如从base=102直接跳到sharp_turn=-40反转），车身在检测边界附近来回穿越时会跟着硬来回
// 打，表现为"左右摇摆剧烈"；PID模式下也一样受益（模式切换/误差跳档时同样是硬跳变）。
// 限速后目标PWM需要一点时间才能到位，把这种硬摇摆摊开、变平滑。数值越大越接近不限速。
#define SLEW_RATE_DEFAULT 800.0f

static bool _on = false;
static int _log_interval_ms = LOG_INTERVAL_DEFAULT;  // 内存日志采样周期(ms)，全局调试选项，lograte命令可调
static char _last_mode_code = 0;   // 上一次记进内存log的modeCode，0=哨兵值(尚无记录)，用来判断这次是不是"档位变化"
static unsigned long _last_log_ms = 0;    // 上一条内存log记录行(E或H)的时间戳，H行的dt用这个算，代表心跳节奏
static unsigned long _last_switch_ms = 0; // 上一次真正档位切换(E)的时间戳，E行的dt用这个算，不受中间插入的心跳(H)打断——
                                           // 这样"上一次切换到这一次切换"的间隔（判断摆动频率最需要看的数）能直接从dt读出来，不用手动加H行的dt
static unsigned long _track_on_ms = 0;    // 这一趟track on按下的时刻，track off时算真实挂钟时长(t_off-t_on)用，不是ram_log的10秒窗口那套
static int _last_dir = 0;          // -1=上次左转  0=无记录  +1=上次右转
static unsigned long _lost_since = 0;
static float _turn_outer_ratio = TURN_OUTER_RATIO_DEFAULT;
static float _sharp_ratio = TURN_RATIO_SHARP_DEFAULT;
static float _medium_ratio = TURN_RATIO_MEDIUM_DEFAULT;
static float _xsharp_ratio = TURN_RATIO_XSHARP_DEFAULT;
static float _medium_speed  = TURN_SPEED_MEDIUM_DEFAULT;   // 方案A：各档前进速度系数
static float _sharp_speed    = TURN_SPEED_SHARP_DEFAULT;
static float _hairpin_speed  = TURN_SPEED_HAIRPIN_DEFAULT;
static int   _min_move_pwm   = MOTOR_MIN_MOVE_PWM_DEFAULT;  // 外轮最小前进PWM(防静摩擦堵转失速)

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

// ===== 算法管理单元（见"算法管理单元设计.md"）=====
// _algos[]为条目数组，_active为激活条目id。运行时"生效值"仍是上面各active静态量(+config的_speed)，
// 它们是 _algos[_active].params 的镜像：切换条目时capture旧、apply新；save/list前capture同步。
// 静态默认与各active静态量默认一致，保证无config文件时也自洽。speed默认12=config的DEFAULT_SPEED。
#define ALGO_DEFAULT_PARAMS \
  { 12, SLEW_RATE_DEFAULT, \
    TURN_OUTER_RATIO_DEFAULT, TURN_RATIO_MEDIUM_DEFAULT, TURN_RATIO_SHARP_DEFAULT, TURN_RATIO_XSHARP_DEFAULT, \
    TURN_SPEED_MEDIUM_DEFAULT, TURN_SPEED_SHARP_DEFAULT, TURN_SPEED_HAIRPIN_DEFAULT, MOTOR_MIN_MOVE_PWM_DEFAULT, \
    PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT }

static AlgoEntry _algos[ALGO_MAX] = {
  { true, "bb-default",  TRACK_ALGO_BANGBANG, ALGO_DEFAULT_PARAMS },
  { true, "pid-default", TRACK_ALGO_PID,      ALGO_DEFAULT_PARAMS },
  // 其余槽位由静态零初始化：used=false
};
static int _active = 0;   // 激活条目id

AlgoProfile track_default_params() {
  AlgoProfile p = ALGO_DEFAULT_PARAMS;
  return p;
}

static void entry_set_name(char* dst, const char* name) {
  if (!name) name = "";
  strncpy(dst, name, ALGO_NAME_LEN - 1);
  dst[ALGO_NAME_LEN - 1] = '\0';
}

// 当前生效值 <-> 档案 的搬运。speed存在config_module(_speed)，这里通过config_get/set_speed同步。
static void profile_capture(AlgoProfile* p) {
  p->speed        = config_get_speed();
  p->slew_rate    = _slew_rate;
  p->turn_ratio   = _turn_outer_ratio;
  p->medium_ratio = _medium_ratio;
  p->sharp_ratio  = _sharp_ratio;
  p->xsharp_ratio = _xsharp_ratio;
  p->medium_speed = _medium_speed;
  p->sharp_speed  = _sharp_speed;
  p->hairpin_speed= _hairpin_speed;
  p->min_move_pwm = _min_move_pwm;
  p->pid_kp       = _pid_kp;
  p->pid_ki       = _pid_ki;
  p->pid_kd       = _pid_kd;
}
static void profile_apply(const AlgoProfile* p) {
  config_set_speed(p->speed);
  _slew_rate      = p->slew_rate;
  _turn_outer_ratio = p->turn_ratio;
  _medium_ratio   = p->medium_ratio;
  _sharp_ratio    = p->sharp_ratio;
  _xsharp_ratio   = p->xsharp_ratio;
  _medium_speed   = p->medium_speed;
  _sharp_speed    = p->sharp_speed;
  _hairpin_speed  = p->hairpin_speed;
  _min_move_pwm   = p->min_move_pwm;
  _pid_kp         = p->pid_kp;
  _pid_ki         = p->pid_ki;
  _pid_kd         = p->pid_kd;
}

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

// 正向驱动轮的最小PWM保护：正值且低于地板则抬到地板(防静摩擦堵转停车)；0(停)和负值(反转)原样返回。
// 只给"必须持续前进"的外轮用；内轮的减速/反转是转向动作，不加地板。
static int floor_fwd(int v) {
  if (v > 0 && v < _min_move_pwm) return _min_move_pwm;
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

float track_get_medium_ratio() {
  return _medium_ratio;
}

void track_set_medium_ratio(float ratio) {
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  _medium_ratio = ratio;
}

float track_get_xsharp_ratio() {
  return _xsharp_ratio;
}

void track_set_xsharp_ratio(float ratio) {
  if (ratio < -1.0f) ratio = -1.0f;
  if (ratio > 0.0f) ratio = 0.0f;
  _xsharp_ratio = ratio;
}

// 方案A：各档前进速度系数(0~1)，clamp到合法区间
float track_get_medium_speed() { return _medium_speed; }
void  track_set_medium_speed(float r) { if (r < 0.0f) r = 0.0f; if (r > 1.0f) r = 1.0f; _medium_speed = r; }
float track_get_sharp_speed() { return _sharp_speed; }
void  track_set_sharp_speed(float r) { if (r < 0.0f) r = 0.0f; if (r > 1.0f) r = 1.0f; _sharp_speed = r; }
float track_get_hairpin_speed() { return _hairpin_speed; }
void  track_set_hairpin_speed(float r) { if (r < 0.0f) r = 0.0f; if (r > 1.0f) r = 1.0f; _hairpin_speed = r; }
int   track_get_min_move_pwm() { return _min_move_pwm; }
void  track_set_min_move_pwm(int pwm) { if (pwm < 0) pwm = 0; if (pwm > 255) pwm = 255; _min_move_pwm = pwm; }

int track_get_algo() {
  return _algo;
}

// 旧命令别名：切到默认条目 id0(BANGBANG) / id1(PID)
void track_set_algo(int algo) {
  track_algo_use(algo == TRACK_ALGO_PID ? 1 : 0);
}

int  track_algo_active() { return _active; }
bool track_algo_used(int id) { return (id >= 0 && id < ALGO_MAX && _algos[id].used); }

AlgoEntry track_get_entry(int id) {
  if (id < 0 || id >= ALGO_MAX) id = _active;
  return _algos[id];
}

// 把当前生效值回存进激活条目（config_save/print/algolist前调，保证读到的是最新调参）
void track_capture_active() {
  profile_capture(&_algos[_active].params);
}

// 切换激活条目：存回旧激活、载入新激活参数+控制律，清PID历史。热切换(正在跑也允许)。
bool track_algo_use(int id) {
  if (id < 0 || id >= ALGO_MAX || !_algos[id].used) return false;
  profile_capture(&_algos[_active].params);
  _active = id;
  profile_apply(&_algos[_active].params);
  _algo = _algos[_active].base;
  pid_reset();
  return true;
}

// 新建条目(默认参数)于第一个空槽，返回新id；满了返回-1。不改变激活条目。
int track_algo_new(int base, const char* name) {
  if (base != TRACK_ALGO_BANGBANG && base != TRACK_ALGO_PID) base = TRACK_ALGO_BANGBANG;
  for (int i = 0; i < ALGO_MAX; i++) {
    if (!_algos[i].used) {
      _algos[i].used = true;
      _algos[i].base = base;
      entry_set_name(_algos[i].name, name);
      _algos[i].params = track_default_params();
      return i;
    }
  }
  return -1;
}

// 复制条目N到空槽（便于在现有调参上派生变体），返回新id；失败-1。
int track_algo_copy(int id, const char* name) {
  if (id < 0 || id >= ALGO_MAX || !_algos[id].used) return -1;
  if (id == _active) profile_capture(&_algos[_active].params);  // 先同步激活条目生效值再复制
  for (int i = 0; i < ALGO_MAX; i++) {
    if (!_algos[i].used) {
      _algos[i] = _algos[id];
      entry_set_name(_algos[i].name, name);
      return i;
    }
  }
  return -1;
}

bool track_algo_rename(int id, const char* name) {
  if (id < 0 || id >= ALGO_MAX || !_algos[id].used) return false;
  entry_set_name(_algos[id].name, name);
  return true;
}

// 删除条目：不能删激活条目，也不能删最后一个可用条目。
bool track_algo_del(int id) {
  if (id < 0 || id >= ALGO_MAX || !_algos[id].used) return false;
  if (id == _active) return false;
  int cnt = 0;
  for (int i = 0; i < ALGO_MAX; i++) if (_algos[i].used) cnt++;
  if (cnt <= 1) return false;
  _algos[id].used = false;
  return true;
}

// config加载：整体灌入条目数组并应用激活条目（含兜底：至少一个可用、激活id有效）。
void track_load_entries(const AlgoEntry* arr, int active_id) {
  for (int i = 0; i < ALGO_MAX; i++) _algos[i] = arr[i];
  bool any = false;
  for (int i = 0; i < ALGO_MAX; i++) if (_algos[i].used) { any = true; break; }
  if (!any) {
    _algos[0].used = true;
    _algos[0].base = TRACK_ALGO_BANGBANG;
    entry_set_name(_algos[0].name, "bb-default");
    _algos[0].params = track_default_params();
  }
  if (active_id < 0 || active_id >= ALGO_MAX || !_algos[active_id].used) {
    active_id = 0;
    for (int i = 0; i < ALGO_MAX; i++) if (_algos[i].used) { active_id = i; break; }
  }
  _active = active_id;
  profile_apply(&_algos[_active].params);
  _algo = _algos[_active].base;
  pid_reset();
}

float track_get_pid_kp() { return _pid_kp; }
void  track_set_pid_kp(float kp) { if (kp < 0.0f) kp = 0.0f; _pid_kp = kp; }
float track_get_pid_ki() { return _pid_ki; }
void  track_set_pid_ki(float ki) { if (ki < 0.0f) ki = 0.0f; _pid_ki = ki; }
float track_get_pid_kd() { return _pid_kd; }
void  track_set_pid_kd(float kd) { if (kd < 0.0f) kd = 0.0f; _pid_kd = kd; }

float track_get_slew_rate() { return _slew_rate; }
void  track_set_slew_rate(float rate) { if (rate < 1.0f) rate = 1.0f; _slew_rate = rate; }

int  track_get_log_interval() { return _log_interval_ms; }
void track_set_log_interval(int ms) { if (ms < 1) ms = 1; _log_interval_ms = ms; }

void track_begin() {
  _on = false;
  _last_dir = 0;
  _lost_since = 0;
  _last_mode_code = 0;
  _last_log_ms = 0;
  _last_switch_ms = 0;
  _track_on_ms = 0;
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

  // 日志全部写内存缓冲区（ram_log_module）。track on~off全程不碰flash，只有track off
  // 这一刻才会顺带把内存内容备份进flash——这时电机已经停了，落盘卡多久都不影响巡线响应。
  // 只写内存：命令/按钮各自已经通过reply()把ON/OFF提示发到USB+BT了，这里不重复走out_usb/out_bt。
  char buf[LOG_LINE_MAX];
  if (on) {
    _track_on_ms = millis();  // 真实挂钟时长(t_off-t_on)的起点，跟ram_log内部的采集状态分开记

    ram_log_begin();
    _last_mode_code = 0;   // 哨兵值，保证这一趟第一条事件行一定会记（不管当前是不是STRAIGHT）
    _last_log_ms = 0;
    _last_switch_ms = 0;

    snprintf(buf, sizeof(buf), ">>> TRACK_ON t=%lu\r\n", _track_on_ms);
    ram_log_append_marker(buf);

    config_build_params_line(buf, sizeof(buf));  // 当前生效的全部参数快照，见4.3节
    ram_log_append_marker(buf);
  } else {
    // 先取duration/lines/elapsed再拼TRACK_OFF这一行：duration只统计事件/心跳行，不含
    // TRACK_OFF自己（TRACK_OFF走的是可以动用预留余量的marker通道，见ram_log_append_marker()
    // 注释）。elapsed是track on到track off之间真实经过的挂钟时间，跟duration是两回事——
    // 之前有过用户拿手机计时对不上号的疑惑（duration被10秒窗口卡住了，那套窗口现在已经
    // 去掉了，但duration仍然可能因为缓冲区提前写满而比elapsed短，所以两个数都留着）
    unsigned long now_ms = millis();
    unsigned long elapsed_ms = now_ms - _track_on_ms;
    unsigned long duration_ms = ram_log_duration_ms();
    unsigned long lines = ram_log_line_count();

    snprintf(buf, sizeof(buf), ">>> TRACK_OFF t=%lu elapsed=%lums duration=%lums lines=%lu\r\n",
             now_ms, elapsed_ms, duration_ms, lines);
    ram_log_append_marker(buf);

    // flash里永远只留"最新一趟"：落盘前先清空旧内容，再把这一趟写进去（不是累积多趟）。
    // ram_log_flush_to_file()只负责写，不会清空RAM，"log dump"照常能从内存读到这一趟的内容
    print_file_clear();
    ram_log_flush_to_file();

    char msg[160];
    snprintf(msg, sizeof(msg),
             ">>> Captured %lu lines over %lums (track on~off elapsed %lums), saved to flash, type 'log dump' to view\r\n",
             lines, duration_ms, elapsed_ms);
    Serial.print(msg);   // 直接走Serial，不受print usb on/off开关影响，现场随时能看到
  }
}

bool track_is_on() {
  return _on;
}

void track_update() {
  if (!_on) return;

  // 只读一次原始ADC值，二值判断和PID的模拟量加权都基于这份数据算，避免同一loop周期
  // 对8路传感器重复采样两次
  int vals[SENSOR_COUNT];
  sensor_read(vals);
  bool is_white[SENSOR_COUNT];
  sensor_binary_from(vals, is_white);

  // 8路顺序：CH1,CH2,...,CH8 → index 0~7（见"8路传感器方案.md"第2、3节）
  // 传感器物理反装（180°翻转）：index 0(CH1)现在在最右侧，index 7(CH8)在最左侧
  // 物理左→右：CH8 CH7 CH6 CH5 | CH4 CH3 CH2 CH1
  bool xsharp_l = is_white[7]; // CH8，物理左侧最外，发卡弯
  bool sharp_l  = is_white[6]; // CH7，物理左侧，急转
  bool medium_l = is_white[5]; // CH6，物理左侧，一档中转
  // CH5(is_white[4])/CH4(is_white[3])为中心两路，压线时直行，不单独判档
  bool medium_r = is_white[2]; // CH3，物理右侧，一档中转
  bool sharp_r  = is_white[1]; // CH2，物理右侧，急转
  bool xsharp_r = is_white[0]; // CH1，物理右侧最外，发卡弯

  bool lost = true;
  for (int i = 0; i < SENSOR_COUNT; i++) if (is_white[i]) { lost = false; break; }
  // 十字路口/横杠：真十字会横穿整个赛道，让两侧"最外两路"都压线才算。
  // 要求 CH7/CH8(左最外) 有一个亮 且 CH1/CH2(右最外) 有一个亮 → cross，直行穿过、不触发转向。
  // （旧写法"左半任一 && 右半任一"分界切在CH4/CH5之间，会把居中的CH4+CH5误判成十字，见log解析答疑Q4）
  bool far_left  = is_white[7] || is_white[6];   // CH8 或 CH7
  bool far_right = is_white[0] || is_white[1];   // CH1 或 CH2
  bool cross = far_left && far_right;

  int base = motor_level_to_pwm(config_get_speed());
  // sharp_turn仅供丢线找线(LOST)分支延续方向找线用，基于满速base，不受方案A的弯道降速影响
  int sharp_turn  = (int)(base * _sharp_ratio);

  int pwm_l = base;
  int pwm_r = base;
  char mode_code = '0';  // modeCode编码表见"LOG精简方案.md"3.2节，'0'=STRAIGHT
  bool stop_now = false;  // true时跳过限速直接停车（安全优先，丢线超时停车不该被限速拖慢）

  unsigned long now = millis();

  if (lost) {
    if (_lost_since == 0) _lost_since = now;
    pid_reset();   // 丢线期间不是连续跟踪，清掉积分/微分历史，避免重新压线后带着丢线期间的误差冲一把
    if (now - _lost_since > LOST_TIMEOUT_MS) {
      pwm_l = 0;
      pwm_r = 0;
      mode_code = 'C';
      stop_now = true;
    } else if (_last_dir < 0) {
      pwm_l = sharp_turn;   // 延续上次左转方向找线
      mode_code = 'A';
    } else if (_last_dir > 0) {
      pwm_r = sharp_turn;   // 延续上次右转方向找线
      mode_code = 'B';
    }
  } else if (cross) {
    _lost_since = 0;
    mode_code = '9';   // 十字路口/宽线，直行穿过，不更新_last_dir
  } else if (_algo == TRACK_ALGO_PID) {
    _lost_since = 0;
    // 误差=模拟量加权位置（2026-07-29改用analog版本，见sensor_module.cpp的
    // sensor_position_analog_from()），setpoint=0（居中）；正=线偏右，需要向右修正
    // （左轮加速/右轮减速）。比原来基于is_white[]二值判断的离散阶梯值更接近连续量，
    // 传感器实测是慢慢从~2000变到~600的，不是一下子跳变，模拟量加权更贴近真实物理过程
    float error = sensor_position_analog_from(vals);
    // 上面这行的丢线判定(权重和<0.3)跟外层的lost判定(is_white[]全黑)标准不完全一样，
    // 理论上存在"is_white[]判断没丢线，但analog权重判断丢线"的边界情况，这里兜底成0
    // （不修正，沿用上一次的_pid_hold_output），避免NAN进到后面的积分/微分运算里
    if (isnan(error)) error = 0.0f;

    // PID只按固定周期重算，中间这些loop沿用_pid_hold_output——
    // 传感器位置原来只有9档离散值，跟着loop()裸奔重算等于在1ms尺度上对着一个台阶信号求导，
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
    mode_code = 'P';
  } else {
    _lost_since = 0;
    // 由外到内依次判定，命中最外层就不再看内层（见"8路传感器方案.md"第4节）
    // 方案A（弯道降速，见"log分析/log17分析.md"第七节）：按档位降低前进(外轮)速度——
    // 弯越急外轮降得越多（base_hp<base_shp<base_med<base），进弯前先收速、给机械留转向时间，
    // 避免以直线速度冲进弯冲到最外侧丢线。
    // 两点保护：① 外轮经floor_fwd()保底，绝不低于最小前进PWM，防降速后外轮堵转停车；
    //          ② sharp/发卡的内轮反转仍用满速base算，保住掉头力度，不被降速削弱(否则发卡转不过来)。
    int base_med = (int)(base * _medium_speed);   // MEDIUM档(CH3/CH6)外轮前进速度
    int base_shp = (int)(base * _sharp_speed);    // SHARP档(CH2/CH7)外轮前进速度
    int base_hp  = (int)(base * _hairpin_speed);  // HAIRPIN档(CH1/CH8)外轮前进速度
    if (xsharp_l) {
      pwm_l = (int)(base * _xsharp_ratio);                     // CH8发卡：内轮反转用满速base，保住掉头力度
      pwm_r = floor_fwd((int)(base_hp * _turn_outer_ratio));   // 外轮降速+最小PWM地板防堵转
      mode_code = '7';
      _last_dir = -1;
    } else if (sharp_l) {
      pwm_l = (int)(base * _sharp_ratio);                      // CH7急转：内轮反转用满速base
      pwm_r = floor_fwd((int)(base_shp * _turn_outer_ratio));
      mode_code = '5';
      _last_dir = -1;
    } else if (medium_l) {
      pwm_l = (int)(base_med * _medium_ratio);                 // CH6一档：内轮减速(不反转)
      pwm_r = floor_fwd(base_med);                             // 外轮降到MEDIUM速度+地板保护
      mode_code = '3';
      _last_dir = -1;
    } else if (xsharp_r) {
      pwm_r = (int)(base * _xsharp_ratio);                     // CH1发卡：内轮反转用满速base
      pwm_l = floor_fwd((int)(base_hp * _turn_outer_ratio));
      mode_code = '8';
      _last_dir = 1;
    } else if (sharp_r) {
      pwm_r = (int)(base * _sharp_ratio);                      // CH2急转：内轮反转用满速base
      pwm_l = floor_fwd((int)(base_shp * _turn_outer_ratio));
      mode_code = '6';
      _last_dir = 1;
    } else if (medium_r) {
      pwm_r = (int)(base_med * _medium_ratio);                 // CH3一档：内轮减速(不反转)
      pwm_l = floor_fwd(base_med);                             // 外轮降到MEDIUM速度+地板保护
      mode_code = '4';
      _last_dir = 1;
    }
    // 都不命中（中心两路CH4/CH5压线、全黑、或中间对同时压线）：直行，不更新_last_dir
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

  // 内存日志：事件触发（档位变化）+ 定周期采样（每 _log_interval_ms 补一条），紧凑格式。
  // track on期间【只写内存缓冲区 ram_log_append，不走串口】——高频采样时串口会拖慢loop、还刷屏，
  // 且RAM写入是memcpy极快、不扰动控制。跑完用 log dump 一次性看。（见"LOG高频采样方案.md"）
  bool mode_changed = (mode_code != _last_mode_code);
  bool heartbeat_due = !mode_changed && (now - _last_log_ms >= (unsigned long)_log_interval_ms);
  if (mode_changed || heartbeat_due) {
    // E行的dt=距上一次真正切换的间隔（用_last_switch_ms算，不受中间插入的心跳打断，
    // 判断摆动频率最需要看的就是这个数）；H行的dt=距上一条记录行的间隔（心跳节奏本身）
    unsigned long dt;
    if (mode_changed) {
      dt = (_last_switch_ms == 0) ? 0 : (now - _last_switch_ms);
      _last_switch_ms = now;
    } else {
      dt = (_last_log_ms == 0) ? 0 : (now - _last_log_ms);
    }
    _last_log_ms = now;
    _last_mode_code = mode_code;

    // 8路is_white[]拼成位图：bit7=CH8(index7)...bit0=CH1(index0)，跟"物理左→右"顺序一致
    uint8_t pattern = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) if (is_white[i]) pattern |= (uint8_t)(1 << i);

    char rec[LOG_LINE_MAX];
    char ev = mode_changed ? 'E' : 'H';
    if (_algo == TRACK_ALGO_PID) {
      snprintf(rec, sizeof(rec), "%c%lu %02X %c %d %d %+.2f\r\n",
               ev, dt, pattern, mode_code, pwm_l, pwm_r, _pid_last_error);
    } else {
      snprintf(rec, sizeof(rec), "%c%lu %02X %c %d %d\r\n",
               ev, dt, pattern, mode_code, pwm_l, pwm_r);
    }
    ram_log_append(rec);   // 只写内存，不走串口（out_usb/out_bt 已去掉，避免高频采样拖慢loop）
  }
}
