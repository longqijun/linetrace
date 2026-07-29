#include "config_module.h"
#include "sensor_module.h"
#include "track_module.h"
#include "print_module.h"
#include "bt_module.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#define CONFIG_FILE   "/config.json"
#define DEFAULT_SPEED 12  // 原1~10档的3，等比换算到1~40档（3*4）

static int _speed = DEFAULT_SPEED;

void config_begin() {
  _speed = DEFAULT_SPEED;
  // 阈值默认值已在sensor_module内置，这里只在json存在时覆盖

  if (!LittleFS.begin(true)) return;
  if (!LittleFS.exists(CONFIG_FILE)) return;

  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return;

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;

  _speed = doc["speed"] | DEFAULT_SPEED;
  track_set_turn_ratio(doc["turn_ratio"] | track_get_turn_ratio());
  track_set_medium_ratio(doc["medium_ratio"] | track_get_medium_ratio());
  track_set_sharp_ratio(doc["sharp_ratio"] | track_get_sharp_ratio());
  track_set_xsharp_ratio(doc["xsharp_ratio"] | track_get_xsharp_ratio());
  track_set_algo(doc["algo"] | track_get_algo());
  track_set_pid_kp(doc["pid_kp"] | track_get_pid_kp());
  track_set_pid_ki(doc["pid_ki"] | track_get_pid_ki());
  track_set_pid_kd(doc["pid_kd"] | track_get_pid_kd());
  track_set_slew_rate(doc["slew_rate"] | track_get_slew_rate());
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

void config_save() {
  StaticJsonDocument<1024> doc;
  doc["speed"] = _speed;
  doc["turn_ratio"] = track_get_turn_ratio();
  doc["medium_ratio"] = track_get_medium_ratio();
  doc["sharp_ratio"] = track_get_sharp_ratio();
  doc["xsharp_ratio"] = track_get_xsharp_ratio();
  doc["algo"] = track_get_algo();
  doc["pid_kp"] = track_get_pid_kp();
  doc["pid_ki"] = track_get_pid_ki();
  doc["pid_kd"] = track_get_pid_kd();
  doc["slew_rate"] = track_get_slew_rate();
  doc["file_log"] = print_file_enabled();

  JsonArray arr = doc.createNestedArray("threshold");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    arr.add(sensor_get_threshold(i));
  }
  JsonArray white_arr = doc.createNestedArray("white_ref");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    white_arr.add(sensor_get_white_ref(i));
  }
  JsonArray black_arr = doc.createNestedArray("black_ref");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    black_arr.add(sensor_get_black_ref(i));
  }

  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return;

  serializeJson(doc, f);
  f.close();
}

void config_print() {
  StaticJsonDocument<1024> doc;
  doc["speed"] = _speed;
  doc["turn_ratio"] = track_get_turn_ratio();
  doc["medium_ratio"] = track_get_medium_ratio();
  doc["sharp_ratio"] = track_get_sharp_ratio();
  doc["xsharp_ratio"] = track_get_xsharp_ratio();
  doc["algo"] = track_get_algo();
  doc["pid_kp"] = track_get_pid_kp();
  doc["pid_ki"] = track_get_pid_ki();
  doc["pid_kd"] = track_get_pid_kd();
  doc["slew_rate"] = track_get_slew_rate();
  doc["file_log"] = print_file_enabled();

  JsonArray arr = doc.createNestedArray("threshold");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    arr.add(sensor_get_threshold(i));
  }
  JsonArray white_arr = doc.createNestedArray("white_ref");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    white_arr.add(sensor_get_white_ref(i));
  }
  JsonArray black_arr = doc.createNestedArray("black_ref");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    black_arr.add(sensor_get_black_ref(i));
  }

  char buf[512];
  serializeJson(doc, buf, sizeof(buf));

  Serial.print(buf);
  Serial.print("\r\n");
  bt_send(buf);
  bt_send("\r\n");
}
