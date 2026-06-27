#pragma once

void print_begin();               // 初始化，默认USB和BT均关闭
void print_set_usb(bool en);
void print_set_bt(bool en);
bool print_usb_enabled();
bool print_bt_enabled();

void out(const char* msg);        // 向所有已开启的通道输出
void out_usb(const char* msg);    // 仅USB
void out_bt(const char* msg);     // 仅BT
