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

static void handle_command(const char* cmd) {

  // --- print usb/bt/all on|off ---
  if (strcmp(cmd, "print usb on") == 0) {
    print_set_usb(true);
    reply(">>> USB 打印已开启\r\n");
  } else if (strcmp(cmd, "print usb off") == 0) {
    print_set_usb(false);
    reply(">>> USB 打印已关闭\r\n");
  } else if (strcmp(cmd, "print bt on") == 0) {
    print_set_bt(true);
    reply(">>> BT 打印已开启\r\n");
  } else if (strcmp(cmd, "print bt off") == 0) {
    print_set_bt(false);
    reply(">>> BT 打印已关闭\r\n");
  } else if (strcmp(cmd, "print on") == 0) {
    print_set_usb(true);
    print_set_bt(true);
    reply(">>> USB+BT 打印已开启\r\n");
  } else if (strcmp(cmd, "print off") == 0) {
    print_set_usb(false);
    print_set_bt(false);
    reply(">>> USB+BT 打印已关闭\r\n");

  // --- stop ---
  } else if (strcmp(cmd, "stop") == 0) {
    motor_stop();
    reply(">>> 电机已停止\r\n");

  // --- speed N ---
  } else if (strncmp(cmd, "speed ", 6) == 0) {
    int level = atoi(cmd + 6);
    if (level < 1 || level > 10) {
      reply(">>> 速度范围 1~10\r\n");
    } else {
      _speed_level = level;
      char buf[48];
      snprintf(buf, sizeof(buf), ">>> 速度设为 %d (PWM %d)\r\n",
               _speed_level, motor_level_to_pwm(_speed_level));
      reply(buf);
    }

  // --- go N ---
  } else if (strncmp(cmd, "go ", 3) == 0) {
    int sec = atoi(cmd + 3);
    if (sec <= 0 || sec > 60) {
      reply(">>> 时间范围 1~60 秒\r\n");
    } else {
      int pwm = motor_level_to_pwm(_speed_level);
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> 前进 %d 秒，速度 %d (PWM %d)\r\n",
               sec, _speed_level, pwm);
      reply(buf);
      motor_set(pwm, pwm);
      delay((unsigned long)sec * 1000);
      motor_stop();
      reply(">>> 停止\r\n");
    }

  // --- back N ---
  } else if (strncmp(cmd, "back ", 5) == 0) {
    int sec = atoi(cmd + 5);
    if (sec <= 0 || sec > 60) {
      reply(">>> 时间范围 1~60 秒\r\n");
    } else {
      int pwm = motor_level_to_pwm(_speed_level);
      char buf[64];
      snprintf(buf, sizeof(buf), ">>> 后退 %d 秒，速度 %d (PWM %d)\r\n",
               sec, _speed_level, pwm);
      reply(buf);
      motor_set(-pwm, -pwm);
      delay((unsigned long)sec * 1000);
      motor_stop();
      reply(">>> 停止\r\n");
    }

  // --- help ---
  } else if (strcmp(cmd, "help") == 0) {
    reply(">>> 可用命令:\r\n");
    reply("    print on/off         USB+BT 数据流开关\r\n");
    reply("    print usb on/off     仅USB 数据流开关\r\n");
    reply("    print bt  on/off     仅BT  数据流开关\r\n");
    reply("    go N                 前进N秒 (1~60)\r\n");
    reply("    back N               后退N秒 (1~60)\r\n");
    reply("    stop                 立即停止电机\r\n");
    reply("    speed N              速度档位 (1~10，默认5)\r\n");
    reply("    help                 显示此帮助\r\n");

  } else {
    reply(">>> 未知命令，输入 help 查看可用命令\r\n");
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
}

void cmd_poll() {
  char line[64];
  if (bt_poll_line(line, sizeof(line))) {
    handle_command(line);
  }
  if (serial_poll_line(line, sizeof(line))) {
    handle_command(line);
  }
}
