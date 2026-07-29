#include "bt_module.h"
#include "BluetoothSerial.h"

static BluetoothSerial _bt;
static bool _began = false;  // bt_begin()没被调用过时，其余函数直接短路返回，不碰_bt对象
static char _line_buf[64];
static int  _line_len = 0;

void bt_begin(const char* name) {
  _bt.begin(name);
  _began = true;
}

bool bt_connected() {
  if (!_began) return false;
  return _bt.hasClient();
}

void bt_send(const char* msg) {
  if (!_began) return;
  if (_bt.hasClient()) {
    _bt.print(msg);
  }
}

bool bt_poll_line(char* buf, int maxlen) {
  if (!_began) return false;
  while (_bt.available()) {
    char c = (char)_bt.read();

    if (c == '\r' || c == '\n') {
      if (_line_len == 0) continue;       // 忽略空行
      _bt.print("\r\n");                  // 回车换行回显
      _line_buf[_line_len] = '\0';
      strncpy(buf, _line_buf, maxlen - 1);
      buf[maxlen - 1] = '\0';
      _line_len = 0;
      return true;
    } else if (c == 0x08 || c == 0x7F) { // 退格
      if (_line_len > 0) {
        _line_len--;
        _bt.print("\x08 \x08"); // 光标退一格、清除、再退
      }
    } else if (_line_len < (int)sizeof(_line_buf) - 1) {
      _line_buf[_line_len++] = c;
      _bt.write(c);                       // 字符回显
    }
  }
  return false;
}
