#include "config_module.h"
#include "sensor_module.h"
#include "bt_module.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#define CONFIG_FILE   "/config.json"
#define DEFAULT_SPEED 3

static int _speed = DEFAULT_SPEED;

void config_begin() {
  _speed = DEFAULT_SPEED;
  // 阈值默认值已在sensor_module内置，这里只在json存在时覆盖

  if (!LittleFS.begin(true)) return;
  if (!LittleFS.exists(CONFIG_FILE)) return;

  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return;

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;

  _speed = doc["speed"] | DEFAULT_SPEED;

  JsonArray arr = doc["threshold"];
  if (!arr.isNull()) {
    int i = 0;
    for (JsonVariant v : arr) {
      if (i >= SENSOR_COUNT) break;
      sensor_set_threshold(i, v.as<int>());
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
  StaticJsonDocument<256> doc;
  doc["speed"] = _speed;

  JsonArray arr = doc.createNestedArray("threshold");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    arr.add(sensor_get_threshold(i));
  }

  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return;

  serializeJson(doc, f);
  f.close();
}

void config_print() {
  StaticJsonDocument<256> doc;
  doc["speed"] = _speed;

  JsonArray arr = doc.createNestedArray("threshold");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    arr.add(sensor_get_threshold(i));
  }

  char buf[192];
  serializeJson(doc, buf, sizeof(buf));

  Serial.print(buf);
  Serial.print("\r\n");
  bt_send(buf);
  bt_send("\r\n");
}
