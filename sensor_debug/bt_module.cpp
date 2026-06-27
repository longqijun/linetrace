#include "bt_module.h"
#include "BluetoothSerial.h"

static BluetoothSerial _bt;

void bt_begin(const char* name) {
  _bt.begin(name);
}

bool bt_connected() {
  return _bt.hasClient();
}

void bt_send(const char* msg) {
  if (_bt.hasClient()) {
    _bt.print(msg);
  }
}
