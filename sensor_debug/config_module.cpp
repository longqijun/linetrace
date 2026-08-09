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

void config_begin() {
  _speed = DEFAULT_SPEED;
  // 阈值默认值已在sensor_module内置，这里只在json存在时覆盖

  if (!LittleFS.begin(true)) return;
  if (!LittleFS.exists(CONFIG_FILE)) return;

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

  // 长度必须严格等于当前SENSOR_COUNT才应用——如果以后SENSOR_COUNT又变了（比如加/减传感器路数），
  // 旧config.json里长度不匹配的threshold数组会按下标错位覆盖到错误的通道上（2026-07-29实测踩过
  // 这个坑：5路方案的[CH2..CH6]数组被当成8路方案的[CH1..CH5]加载，导致6路阈值全部错位一格），
  // 长度不对就整个跳过，宁可用代码默认值也不要错位应用
  JsonArray arr = doc["threshold"];
  if (!arr.isNull() && arr.size() == SENSOR_COUNT) {
    int i = 0;
    for (JsonVariant v : arr) {
      sensor_set_threshold(i, v.as<int>());
      i++;
    }
  }

  // 同上，长度不对就整个跳过，不按下标错位应用
  JsonArray white_arr = doc["white_ref"];
  if (!white_arr.isNull() && white_arr.size() == SENSOR_COUNT) {
    int i = 0;
    for (JsonVariant v : white_arr) {
      sensor_set_white_ref(i, v.as<int>());
      i++;
    }
  }
  JsonArray black_arr = doc["black_ref"];
  if (!black_arr.isNull() && black_arr.size() == SENSOR_COUNT) {
    int i = 0;
    for (JsonVariant v : black_arr) {
      sensor_set_black_ref(i, v.as<int>());
      i++;
    }
  }
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

  JsonArray arr = doc.createNestedArray("threshold");
  for (int i = 0; i < SENSOR_COUNT; i++) arr.add(sensor_get_threshold(i));
  JsonArray white_arr = doc.createNestedArray("white_ref");
  for (int i = 0; i < SENSOR_COUNT; i++) white_arr.add(sensor_get_white_ref(i));
  JsonArray black_arr = doc.createNestedArray("black_ref");
  for (int i = 0; i < SENSOR_COUNT; i++) black_arr.add(sensor_get_black_ref(i));
}

void config_save() {
  DynamicJsonDocument doc(6144);
  build_config_doc(doc);

  File f = LittleFS.open(CONFIG_FILE, "w");
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
  snprintf(line, sizeof(line), "  active=%d  file_log=%s\r\n",
           track_algo_active(), print_file_enabled() ? "ON" : "OFF");
  cfg_out(line);
  build_int_array(arr, sizeof(arr), sensor_get_threshold);
  snprintf(line, sizeof(line), "  threshold=%s\r\n", arr); cfg_out(line);
  build_int_array(arr, sizeof(arr), sensor_get_white_ref);
  snprintf(line, sizeof(line), "  white_ref=%s\r\n", arr); cfg_out(line);
  build_int_array(arr, sizeof(arr), sensor_get_black_ref);
  snprintf(line, sizeof(line), "  black_ref=%s\r\n", arr); cfg_out(line);

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
           "slew_rate=%.1f pid_kp=%.2f pid_ki=%.2f pid_kd=%.2f thresh=%s white=%s black=%s\r\n",
           track_algo_active(), ae.name, _speed, track_get_algo() == TRACK_ALGO_PID ? "PID" : "BANGBANG",
           track_get_turn_ratio(), track_get_medium_ratio(), track_get_sharp_ratio(), track_get_xsharp_ratio(),
           track_get_medium_speed(), track_get_sharp_speed(), track_get_hairpin_speed(), track_get_min_move_pwm(),
           track_get_slew_rate(), track_get_pid_kp(), track_get_pid_ki(), track_get_pid_kd(),
           thresh, white, black);
}
