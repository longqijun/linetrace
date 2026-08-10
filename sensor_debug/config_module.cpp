#include "config_module.h"
#include "sensor_module.h"
#include "track_module.h"
#include "print_module.h"
#include "bt_module.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>

#define CONFIG_FILE   "/config.json"
#define SENSOR_FILE   "/sensor.json"   // 传感器值(white/black/threshold)独立存这里，跟参数分开保存
#define DEFAULT_SPEED 12  // 原1~10档的3，等比换算到1~40档（3*4）

static int _speed = DEFAULT_SPEED;

// 从JsonVariant读一份算法档案；键名与老flat格式一致，缺字段用p里现值兜底。
// 既能读根级老flat键（向后兼容旧config），也能读嵌套的"bb"/"pid"对象。
static void read_profile(JsonVariantConst o, AlgoProfile& p) {
  p.speed         = o["speed"]         | p.speed;
  p.slew_rate     = o["slew_rate"]     | p.slew_rate;
  p.turn_ratio    = o["turn_ratio"]    | p.turn_ratio;
  p.medium_ratio  = o["medium_ratio"]  | p.medium_ratio;
  p.sharp_ratio   = o["sharp_ratio"]   | p.sharp_ratio;
  p.xsharp_ratio  = o["xsharp_ratio"]  | p.xsharp_ratio;
  p.medium_speed  = o["medium_speed"]  | p.medium_speed;
  p.sharp_speed   = o["sharp_speed"]   | p.sharp_speed;
  p.hairpin_speed = o["hairpin_speed"] | p.hairpin_speed;
  p.min_move_pwm  = o["min_move_pwm"]  | p.min_move_pwm;
  p.pid_kp        = o["pid_kp"]        | p.pid_kp;
  p.pid_ki        = o["pid_ki"]        | p.pid_ki;
  p.pid_kd        = o["pid_kd"]        | p.pid_kd;
}
static void write_profile(JsonObject o, const AlgoProfile& p) {
  o["speed"] = p.speed;             o["slew_rate"] = p.slew_rate;
  o["turn_ratio"] = p.turn_ratio;   o["medium_ratio"] = p.medium_ratio;
  o["sharp_ratio"] = p.sharp_ratio; o["xsharp_ratio"] = p.xsharp_ratio;
  o["medium_speed"] = p.medium_speed; o["sharp_speed"] = p.sharp_speed;
  o["hairpin_speed"] = p.hairpin_speed; o["min_move_pwm"] = p.min_move_pwm;
  o["pid_kp"] = p.pid_kp; o["pid_ki"] = p.pid_ki; o["pid_kd"] = p.pid_kd;
}
static void copy_name(char* dst, const char* src) {
  if (!src) src = "";
  strncpy(dst, src, ALGO_NAME_LEN - 1);
  dst[ALGO_NAME_LEN - 1] = '\0';
}

static void cfg_out(const char* s) { Serial.print(s); bt_send(s); }          // config打印:同发Serial+BT
static void build_int_array(char* out, size_t out_size, int (*getter)(int)); // 前置声明(定义在下方)

// 把长度严格等于SENSOR_COUNT的json数组逐路灌进setter；长度不符整个跳过(见下方错位坑注释)
static void apply_sensor_array(JsonArray arr, void (*setter)(int, int)) {
  if (arr.isNull() || arr.size() != SENSOR_COUNT) return;
  int i = 0;
  for (JsonVariant v : arr) { setter(i, v.as<int>()); i++; }
}

// 从/sensor.json载入传感器值(white/black/threshold)，覆盖内存。返回是否成功载入。
static bool load_sensor_file() {
  if (!LittleFS.exists(SENSOR_FILE)) return false;
  File f = LittleFS.open(SENSOR_FILE, "r");
  if (!f) return false;
  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;
  apply_sensor_array(doc["threshold"], sensor_set_threshold);
  apply_sensor_array(doc["white_ref"], sensor_set_white_ref);
  apply_sensor_array(doc["black_ref"], sensor_set_black_ref);
  return true;
}

// 解析/config.json：算法条目 + 共享参数 + 标定参数 + (向后兼容)内嵌的传感器数组
static void load_config_file() {
  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return;

  DynamicJsonDocument doc(6144);   // 堆分配，避免大对象压栈（最多8条目）
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;

  // ---- 载入算法条目（管理单元）----
  AlgoEntry entries[ALGO_MAX];
  for (int i = 0; i < ALGO_MAX; i++) {
    entries[i].used = false;
    entries[i].name[0] = '\0';
    entries[i].base = TRACK_ALGO_BANGBANG;
    entries[i].params = track_default_params();
  }
  int active;
  if (doc.containsKey("algos")) {
    // 新格式：algos 数组，每元素 {id,name,base,...params}
    for (JsonVariant v : doc["algos"].as<JsonArray>()) {
      int id = v["id"] | -1;
      if (id < 0 || id >= ALGO_MAX) continue;
      entries[id].used = true;
      entries[id].base = v["base"] | TRACK_ALGO_BANGBANG;
      copy_name(entries[id].name, v["name"] | "");
      entries[id].params = track_default_params();
      read_profile(v, entries[id].params);
    }
    active = doc["active"] | 0;
  } else {
    // 向后兼容：老的根级扁平键 / 上一版 bb|pid 对象 → 重建 id0(bb)、id1(pid)
    entries[0].used = true; entries[0].base = TRACK_ALGO_BANGBANG; copy_name(entries[0].name, "bb-default");
    read_profile(doc.as<JsonVariant>(), entries[0].params);
    if (doc.containsKey("bb"))  read_profile(doc["bb"], entries[0].params);
    entries[1].used = true; entries[1].base = TRACK_ALGO_PID;      copy_name(entries[1].name, "pid-default");
    read_profile(doc.as<JsonVariant>(), entries[1].params);
    if (doc.containsKey("pid")) read_profile(doc["pid"], entries[1].params);
    active = doc["active"] | (doc["algo"] | 0);
  }
  track_load_entries(entries, active);

  print_set_file(doc["file_log"] | print_file_enabled());
  track_set_log_interval(doc["lograte"] | track_get_log_interval());  // 全局:内存日志采样周期

  // 自动标定参数（旋钮，不是测得的传感器值，归config.json）
  sensor_set_calib_k(doc["calib_k"] | sensor_get_calib_k());
  sensor_set_calib_ratio(doc["calib_ratio"] | sensor_get_calib_ratio());
  sensor_set_calib_sweep_sec(doc["calib_sweep_sec"] | sensor_get_calib_sweep_sec());

  // 向后兼容：旧config.json里内嵌的传感器数组(现已迁到/sensor.json)。apply_sensor_array内部
  // 会校验长度严格等于SENSOR_COUNT才应用——长度不对整个跳过，宁可用默认值也不要按下标错位覆盖
  // （2026-07-29实测踩过：5路[CH2..CH6]被当8路[CH1..CH5]加载，6路阈值全部错位一格）
  apply_sensor_array(doc["threshold"], sensor_set_threshold);
  apply_sensor_array(doc["white_ref"], sensor_set_white_ref);
  apply_sensor_array(doc["black_ref"], sensor_set_black_ref);
}

void config_begin() {
  _speed = DEFAULT_SPEED;
  // 阈值默认值已在sensor_module内置，这里只在文件存在时覆盖
  if (!LittleFS.begin(true)) return;

  if (LittleFS.exists(CONFIG_FILE)) load_config_file();  // 参数 + (兼容)内嵌传感器值

  // 传感器值优先用独立的/sensor.json；不存在则把刚从config.json/默认值载入的值一次性迁移过去，
  // 之后传感器值就归/sensor.json管，config_save()不再写它们（见"传感器阈值自动标定方案.md"8.5节）
  if (!load_sensor_file()) config_save_sensor(SENSOR_SAVE_ALL);
}

int config_get_speed() {
  return _speed;
}

void config_set_speed(int level) {
  _speed = level;
}

// save/print 共用：先把生效值回存进激活条目，再把 active + 所有已启用条目 + 共享标定拼进doc
static void build_config_doc(JsonDocument& doc) {
  track_capture_active();   // 保证落盘/打印的是激活条目最新调参
  doc["active"] = track_algo_active();
  doc["file_log"] = print_file_enabled();
  doc["lograte"] = track_get_log_interval();

  // 自动标定参数（旋钮）。传感器值(white/black/threshold)不在此，由config_save_sensor写/sensor.json
  doc["calib_k"] = sensor_get_calib_k();
  doc["calib_ratio"] = sensor_get_calib_ratio();
  doc["calib_sweep_sec"] = sensor_get_calib_sweep_sec();

  JsonArray algos = doc.createNestedArray("algos");
  for (int id = 0; id < ALGO_MAX; id++) {
    if (!track_algo_used(id)) continue;
    AlgoEntry e = track_get_entry(id);
    JsonObject o = algos.createNestedObject();
    o["id"] = id;
    o["name"] = e.name;   // char[]非常量→ArduinoJson复制字符串，e出作用域也安全
    o["base"] = e.base;
    write_profile(o, e.params);
  }
}

void config_save() {
  DynamicJsonDocument doc(6144);
  build_config_doc(doc);

  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return;
  serializeJson(doc, f);
  f.close();
}

// 保存传感器值到/sensor.json：读现有文件→只覆盖mask选中的数组→写回，未选中的原样保留。
// 这样 savesensor white 不会动 black/threshold，跟 save(参数)也互不干扰(不同文件)。
void config_save_sensor(int mask) {
  DynamicJsonDocument doc(2048);
  if (LittleFS.exists(SENSOR_FILE)) {          // 先读回现有内容，保住不改的项
    File rf = LittleFS.open(SENSOR_FILE, "r");
    if (rf) { deserializeJson(doc, rf); rf.close(); }
  }
  if (mask & SENSOR_SAVE_THRESH) {
    doc.remove("threshold");
    JsonArray a = doc.createNestedArray("threshold");
    for (int i = 0; i < SENSOR_COUNT; i++) a.add(sensor_get_threshold(i));
  }
  if (mask & SENSOR_SAVE_WHITE) {
    doc.remove("white_ref");
    JsonArray a = doc.createNestedArray("white_ref");
    for (int i = 0; i < SENSOR_COUNT; i++) a.add(sensor_get_white_ref(i));
  }
  if (mask & SENSOR_SAVE_BLACK) {
    doc.remove("black_ref");
    JsonArray a = doc.createNestedArray("black_ref");
    for (int i = 0; i < SENSOR_COUNT; i++) a.add(sensor_get_black_ref(i));
  }
  File f = LittleFS.open(SENSOR_FILE, "w");
  if (!f) return;
  serializeJson(doc, f);
  f.close();
}

// config 命令:人类可读的分节分行视图(先共用参数、后算法参数)。落盘的 config_save 仍走 JSON,不受此影响。
void config_print() {
  track_capture_active();   // 同步激活条目的生效值，保证显示最新
  char line[256], arr[112];

  // ---- 共用参数(两算法通用) ----
  cfg_out(">>> config —— 共用参数(两算法通用):\r\n");
  snprintf(line, sizeof(line), "  active=%d  file_log=%s  lograte=%dms\r\n",
           track_algo_active(), print_file_enabled() ? "ON" : "OFF", track_get_log_interval());
  cfg_out(line);
  build_int_array(arr, sizeof(arr), sensor_get_threshold);
  snprintf(line, sizeof(line), "  threshold=%s\r\n", arr); cfg_out(line);
  build_int_array(arr, sizeof(arr), sensor_get_white_ref);
  snprintf(line, sizeof(line), "  white_ref=%s\r\n", arr); cfg_out(line);
  build_int_array(arr, sizeof(arr), sensor_get_black_ref);
  snprintf(line, sizeof(line), "  black_ref=%s\r\n", arr); cfg_out(line);
  snprintf(line, sizeof(line), "  calib: k=%d ratio=%.2f sweep=%ds\r\n",
           sensor_get_calib_k(), sensor_get_calib_ratio(), sensor_get_calib_sweep_sec());
  cfg_out(line);

  // ---- 算法参数(每条一算法, *=当前激活) ----
  cfg_out(">>> config —— 算法参数(*=当前激活):\r\n");
  int act = track_algo_active();
  for (int id = 0; id < ALGO_MAX; id++) {
    if (!track_algo_used(id)) continue;
    AlgoEntry e = track_get_entry(id);
    snprintf(line, sizeof(line), "  %c%d %-15s base=%s  speed=%d slew=%.0f\r\n",
             id == act ? '*' : ' ', id, e.name, e.base == TRACK_ALGO_PID ? "PID" : "BB",
             e.params.speed, e.params.slew_rate);
    cfg_out(line);
    if (e.base == TRACK_ALGO_PID) {
      snprintf(line, sizeof(line), "       pid  kp=%.2f ki=%.2f kd=%.2f\r\n",
               e.params.pid_kp, e.params.pid_ki, e.params.pid_kd);
    } else {
      snprintf(line, sizeof(line),
               "       bb   turn=%.2f medium=%.2f sharp=%.2f xsharp=%.2f medspeed=%.2f shpspeed=%.2f hpspeed=%.2f minpwm=%d\r\n",
               e.params.turn_ratio, e.params.medium_ratio, e.params.sharp_ratio, e.params.xsharp_ratio,
               e.params.medium_speed, e.params.sharp_speed, e.params.hairpin_speed, e.params.min_move_pwm);
    }
    cfg_out(line);
  }
}

// "[v0,v1,...,v7]"，getter(i)取第i路的值，SENSOR_COUNT路
static void build_int_array(char* out, size_t out_size, int (*getter)(int)) {
  size_t pos = 0;
  if (out_size == 0) return;
  out[pos++] = '[';
  for (int i = 0; i < SENSOR_COUNT && pos + 1 < out_size; i++) {
    int n = snprintf(out + pos, out_size - pos, i == 0 ? "%d" : ",%d", getter(i));
    if (n < 0 || pos + (size_t)n >= out_size) break;
    pos += n;
  }
  if (pos + 1 < out_size) out[pos++] = ']';
  out[pos] = '\0';
}

void config_build_params_line(char* buf, size_t buf_size) {
  char thresh[96], white[96], black[96];
  build_int_array(thresh, sizeof(thresh), sensor_get_threshold);
  build_int_array(white, sizeof(white), sensor_get_white_ref);
  build_int_array(black, sizeof(black), sensor_get_black_ref);

  AlgoEntry ae = track_get_entry(track_algo_active());  // 当前激活算法(取id+name)
  snprintf(buf, buf_size,
           ">>> PARAMS algoid=%d name=%s speed=%d algo=%s turn_ratio=%.2f medium_ratio=%.2f sharp_ratio=%.2f xsharp_ratio=%.2f "
           "medspeed=%.2f shpspeed=%.2f hpspeed=%.2f minpwm=%d "
           "slew_rate=%.1f lograte=%d pid_kp=%.2f pid_ki=%.2f pid_kd=%.2f thresh=%s white=%s black=%s\r\n",
           track_algo_active(), ae.name, _speed, track_get_algo() == TRACK_ALGO_PID ? "PID" : "BANGBANG",
           track_get_turn_ratio(), track_get_medium_ratio(), track_get_sharp_ratio(), track_get_xsharp_ratio(),
           track_get_medium_speed(), track_get_sharp_speed(), track_get_hairpin_speed(), track_get_min_move_pwm(),
           track_get_slew_rate(), track_get_log_interval(), track_get_pid_kp(), track_get_pid_ki(), track_get_pid_kd(),
           thresh, white, black);
}
