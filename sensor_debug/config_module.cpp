#include "config_module.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

#define CONFIG_FILE   "/config.json"
#define DEFAULT_SPEED 3

static int _speed = DEFAULT_SPEED;

void config_begin() {
  _speed = DEFAULT_SPEED;

  if (!LittleFS.begin(true)) return;
  if (!LittleFS.exists(CONFIG_FILE)) return;

  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return;

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;

  _speed = doc["speed"] | DEFAULT_SPEED;
}

int config_get_speed() {
  return _speed;
}

void config_set_speed(int level) {
  _speed = level;
}

void config_save() {
  StaticJsonDocument<128> doc;
  doc["speed"] = _speed;

  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return;

  serializeJson(doc, f);
  f.close();
}
