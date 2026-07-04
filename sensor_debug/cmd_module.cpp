#include "cmd_module.h"
#include "bt_module.h"
#include "motor_module.h"
#include "print_module.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

static int _speed_level = 5;

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
    motor_stop();
    reply(">>> Motor stopped\r\n");

  // --- speed N ---
  } else if (strncmp(cmd, "speed ", 6) == 0) {
    int level = atoi(cmd + 6);
    if (level < 1 || level > 10) {
      reply(">>> Speed range 1~10\r\n");
    } else {
      _speed_level = level;
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> Speed set to %d (PWM %d)\r\n",
               _speed_level, motor_level_to_pwm(_speed_level));
      reply(buf);
    }

  // --- go N ---
  } else if (strncmp(cmd, "go ", 3) == 0) {
    int sec = atoi(cmd + 3);
    if (sec <= 0 || sec > 60) {
      reply(">>> Time range 1~60 sec\r\n");
    } else {
      int pwm = motor_level_to_pwm(_speed_level);
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> Forward %d sec, speed %d (PWM %d)\r\n",
               sec, _speed_level, pwm);
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
      int pwm = motor_level_to_pwm(_speed_level);
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> Backward %d sec, speed %d (PWM %d)\r\n",
               sec, _speed_level, pwm);
      reply(buf);
      motor_set(-pwm, -pwm);
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
    reply("    stop                 stop motor immediately\r\n");
    reply("    speed N              speed level (1~10, default 5)\r\n");
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
  _speed_level = 5;
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
