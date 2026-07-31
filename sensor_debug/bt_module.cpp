#include "bt_module.h"

// BT彻底停用后的空实现（stub）：不再#include "BluetoothSerial.h"、不再有BluetoothSerial对象，
// 经典BT/Bluedroid host协议栈那部分代码因此不会被链接进固件（配合sensor_debug.ino里
// esp_bt_controller_mem_release()释放controller层内存，两层加起来BT基本不占用flash/RAM）。
// 函数签名保持不变，是为了其他模块（cmd_module.cpp的reply()、print_module.cpp的out_bt()、
// sensor_debug.ino的按键处理）不用跟着改一行代码——都调用这几个函数，只是现在什么也不做。
// 如果以后要恢复BT：把下面的实现换回真正调BluetoothSerial的版本即可（可以查git历史）。

void bt_begin(const char* name) {
  (void)name;
}

bool bt_connected() {
  return false;
}

void bt_send(const char* msg) {
  (void)msg;
}

bool bt_poll_line(char* buf, int maxlen) {
  (void)buf;
  (void)maxlen;
  return false;
}
