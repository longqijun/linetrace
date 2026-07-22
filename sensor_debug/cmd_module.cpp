#include "cmd_module.h"
#include "bt_module.h"
#include "motor_module.h"
#include "print_module.h"
#include "config_module.h"
#include "track_module.h"
#include "sensor_module.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// 命令响应：始终输出，不受 print_module 控制
static void reply(const char* msg) {
  Serial.print(msg);
  bt_send(msg);
}

static void show_prompt() {
  reply(">> ");
}

static void handle_command(const char* cmd) {

  // --- print usb/bt/all on|off ---
  if (strcmp(cmd, "print usb on") == 0) {
    print_set_usb(true);
    reply(">>> USB print ON\r\n");
  } else if (strcmp(cmd, "print usb off") == 0) {
    print_set_usb(false);
    reply(">>> USB print OFF\r\n");
  } else if (strcmp(cmd, "print bt on") == 0) {
    print_set_bt(true);
    reply(">>> BT print ON\r\n");
  } else if (strcmp(cmd, "print bt off") == 0) {
    print_set_bt(false);
    reply(">>> BT print OFF\r\n");
  } else if (strcmp(cmd, "print on") == 0) {
    print_set_usb(true);
    print_set_bt(true);
    reply(">>> USB+BT print ON\r\n");
  } else if (strcmp(cmd, "print off") == 0) {
    print_set_usb(false);
    print_set_bt(false);
    reply(">>> USB+BT print OFF\r\n");

  // --- stop ---
  } else if (strcmp(cmd, "stop") == 0) {
    track_set(false);
    motor_stop();
    reply(">>> Motor stopped\r\n");

  // --- track on/off ---
  } else if (strcmp(cmd, "track on") == 0) {
    track_set(true);
    reply(">>> Line tracking ON\r\n");
  } else if (strcmp(cmd, "track off") == 0) {
    track_set(false);
    reply(">>> Line tracking OFF\r\n");

  // --- speed N ---
  } else if (strncmp(cmd, "speed ", 6) == 0) {
    int level = atoi(cmd + 6);
    if (level < 1 || level > 40) {
      reply(">>> Speed range 1~40\r\n");
    } else {
      config_set_speed(level);
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> Speed set to %d (PWM %d)\r\n",
               level, motor_level_to_pwm(level));
      reply(buf);
    }

  // --- turnspeed N (急弯外轮速度比例，百分比0~100) ---
  } else if (strncmp(cmd, "turnspeed ", 10) == 0) {
    int pct = atoi(cmd + 10);
    if (pct < 0 || pct > 100) {
      reply(">>> Turn speed range 0~100\r\n");
    } else {
      track_set_turn_ratio(pct / 100.0f);
      char buf[80];
      snprintf(buf, sizeof(buf), ">>> Turn speed set to %d%% (outer wheel ratio on sharp turn)\r\n", pct);
      reply(buf);
    }

  // --- sharpratio N (急弯内轮反转比例，百分比0~100，内部转成负值) ---
  } else if (strncmp(cmd, "sharpratio ", 11) == 0) {
    int pct = atoi(cmd + 11);
    if (pct < 0 || pct > 100) {
      reply(">>> Sharp ratio range 0~100\r\n");
    } else {
      track_set_sharp_ratio(-pct / 100.0f);
      char buf[80];
      snprintf(buf, sizeof(buf), ">>> Sharp ratio set to -%d%% (inner wheel reverse on sharp turn)\r\n", pct);
      reply(buf);
    }

  // --- algo bangbang/pid ---
  } else if (strcmp(cmd, "algo bangbang") == 0) {
    track_set_algo(TRACK_ALGO_BANGBANG);
    reply(">>> Track algorithm: BANGBANG\r\n");
  } else if (strcmp(cmd, "algo pid") == 0) {
    track_set_algo(TRACK_ALGO_PID);
    reply(">>> Track algorithm: PID\r\n");

  // --- pid kp/ki/kd N (float, PID增益) ---
  } else if (strncmp(cmd, "pid kp ", 7) == 0) {
    float v = atof(cmd + 7);
    if (v < 0.0f) {
      reply(">>> PID Kp must be >= 0\r\n");
    } else {
      track_set_pid_kp(v);
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> PID Kp set to %.2f\r\n", v);
      reply(buf);
    }
  } else if (strncmp(cmd, "pid ki ", 7) == 0) {
    float v = atof(cmd + 7);
    if (v < 0.0f) {
      reply(">>> PID Ki must be >= 0\r\n");
    } else {
      track_set_pid_ki(v);
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> PID Ki set to %.2f\r\n", v);
      reply(buf);
    }
  } else if (strncmp(cmd, "pid kd ", 7) == 0) {
    float v = atof(cmd + 7);
    if (v < 0.0f) {
      reply(">>> PID Kd must be >= 0\r\n");
    } else {
      track_set_pid_kd(v);
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> PID Kd set to %.2f\r\n", v);
      reply(buf);
    }

  // --- save ---
  } else if (strcmp(cmd, "save") == 0) {
    config_save();
    char buf[48];
    snprintf(buf, sizeof(buf), ">>> Config saved (speed=%d)\r\n", config_get_speed());
    reply(buf);

  // --- config (print current config as JSON) ---
  } else if (strcmp(cmd, "config") == 0) {
    config_print();

  // --- threshold CH VALUE ---
  } else if (strncmp(cmd, "threshold ", 10) == 0) {
    int ch, value;
    if (sscanf(cmd + 10, "%d %d", &ch, &value) != 2 || ch < 2 || ch > 6) {
      reply(">>> Usage: threshold CH VALUE (CH=2~6)\r\n");
    } else {
      sensor_set_threshold(ch - 2, value);
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> CH%d threshold set to %d (memory only, use save to persist)\r\n",
               ch, value);
      reply(buf);
    }

  // --- go N ---
  } else if (strncmp(cmd, "go ", 3) == 0) {
    int sec = atoi(cmd + 3);
    if (sec <= 0 || sec > 60) {
      reply(">>> Time range 1~60 sec\r\n");
    } else {
      int pwm = motor_level_to_pwm(config_get_speed());
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> Forward %d sec, speed %d (PWM %d)\r\n",
               sec, config_get_speed(), pwm);
      reply(buf);
      motor_set(pwm, pwm);
      delay((unsigned long)sec * 1000);
      motor_stop();
      reply(">>> Stopped\r\n");
    }

  // --- back N ---
  } else if (strncmp(cmd, "back ", 5) == 0) {
    int sec = atoi(cmd + 5);
    if (sec <= 0 || sec > 60) {
      reply(">>> Time range 1~60 sec\r\n");
    } else {
      int pwm = motor_level_to_pwm(config_get_speed());
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> Backward %d sec, speed %d (PWM %d)\r\n",
               sec, config_get_speed(), pwm);
      reply(buf);
      motor_set(-pwm, -pwm);
      delay((unsigned long)sec * 1000);
      motor_stop();
      reply(">>> Stopped\r\n");
    }

  // --- spin left/right N (原地转圈，测试电机扭矩) ---
  } else if (strncmp(cmd, "spin left ", 10) == 0) {
    int sec = atoi(cmd + 10);
    if (sec <= 0 || sec > 60) {
      reply(">>> Time range 1~60 sec\r\n");
    } else {
      int pwm = motor_level_to_pwm(config_get_speed());
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> Spin left %d sec, speed %d (PWM %d)\r\n",
               sec, config_get_speed(), pwm);
      reply(buf);
      motor_set(-pwm, pwm);
      delay((unsigned long)sec * 1000);
      motor_stop();
      reply(">>> Stopped\r\n");
    }

  } else if (strncmp(cmd, "spin right ", 11) == 0) {
    int sec = atoi(cmd + 11);
    if (sec <= 0 || sec > 60) {
      reply(">>> Time range 1~60 sec\r\n");
    } else {
      int pwm = motor_level_to_pwm(config_get_speed());
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> Spin right %d sec, speed %d (PWM %d)\r\n",
               sec, config_get_speed(), pwm);
      reply(buf);
      motor_set(pwm, -pwm);
      delay((unsigned long)sec * 1000);
      motor_stop();
      reply(">>> Stopped\r\n");
    }

  // --- help ---
  } else if (strcmp(cmd, "help") == 0) {
    reply(">>> Available commands:\r\n");
    reply("    print on/off         USB+BT data stream on/off\r\n");
    reply("    print usb on/off     USB data stream only\r\n");
    reply("    print bt  on/off     BT  data stream only\r\n");
    reply("    go N                 forward N sec (1~60)\r\n");
    reply("    back N               backward N sec (1~60)\r\n");
    reply("    spin left N          spin in place left N sec (1~60)\r\n");
    reply("    spin right N         spin in place right N sec (1~60)\r\n");
    reply("    stop                 stop motor immediately (also cancels track)\r\n");
    reply("    track on/off         start/stop autonomous line tracking\r\n");
    reply("    speed N              speed level (1~40, default 12, until changed)\r\n");
    reply("    turnspeed N          outer wheel ratio on sharp turn (0~100%, default 65)\r\n");
    reply("    sharpratio N         inner wheel reverse ratio on sharp turn (0~100%, default 30)\r\n");
    reply("    algo bangbang/pid    select track algorithm (default bangbang)\r\n");
    reply("    pid kp N             PID proportional gain (float, default 40.0)\r\n");
    reply("    pid ki N             PID integral gain (float, default 0.0)\r\n");
    reply("    pid kd N             PID derivative gain (float, default 5.0)\r\n");
    reply("    save                 save speed+turnspeed+sharpratio+algo+pid gains+thresholds to flash (/config.json)\r\n");
    reply("    config               print current config as JSON\r\n");
    reply("    threshold CH VALUE   set CHx (2~6) threshold, memory only\r\n");
    reply("    help                 show this help\r\n");

  } else {
    reply(">>> Unknown command, type help for command list\r\n");
  }
}

// Serial 行读取（带回显）
static char _serial_buf[64];
static int  _serial_len = 0;

static bool serial_poll_line(char* buf, int maxlen) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (_serial_len == 0) continue;
      Serial.print("\r\n");
      _serial_buf[_serial_len] = '\0';
      strncpy(buf, _serial_buf, maxlen - 1);
      buf[maxlen - 1] = '\0';
      _serial_len = 0;
      return true;
    } else if (c == 0x08 || c == 0x7F) {
      if (_serial_len > 0) {
        _serial_len--;
        Serial.print("\x08 \x08");
      }
    } else if (_serial_len < (int)sizeof(_serial_buf) - 1) {
      _serial_buf[_serial_len++] = c;
      Serial.write(c);
    }
  }
  return false;
}

void cmd_begin() {
  show_prompt();
}

void cmd_poll() {
  char line[64];
  if (bt_poll_line(line, sizeof(line))) {
    handle_command(line);
    show_prompt();
  }
  if (serial_poll_line(line, sizeof(line))) {
    handle_command(line);
    show_prompt();
  }
}
