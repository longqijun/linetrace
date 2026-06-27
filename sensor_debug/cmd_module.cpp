#include "cmd_module.h"
#include "bt_module.h"
#include <Arduino.h>
#include <string.h>

static bool _print_enabled = true;

static void handle_command(const char* cmd) {
  if (strcmp(cmd, "print off") == 0) {
    _print_enabled = false;
    bt_send(">>> 连续打印已关闭\r\n");

  } else if (strcmp(cmd, "print on") == 0) {
    _print_enabled = true;
    bt_send(">>> 连续打印已开启\r\n");

  } else if (strcmp(cmd, "help") == 0) {
    bt_send(">>> 可用命令:\r\n");
    bt_send("    print on   开启连续打印\r\n");
    bt_send("    print off  关闭连续打印\r\n");
    bt_send("    help       显示此帮助\r\n");

  } else {
    bt_send(">>> 未知命令，输入 help 查看可用命令\r\n");
  }
}

void cmd_begin() {
  _print_enabled = true;
}

void cmd_poll() {
  char line[64];
  if (bt_poll_line(line, sizeof(line))) {
    handle_command(line);
  }
}

bool cmd_print_enabled() {
  return _print_enabled;
}
