#pragma once

// 输出通道：USB / BT / Flash文件（/track.log）三选任意组合开启
// 文件通道给BT不稳定/断连场景用：track_module的调试log正常走out()同时写进文件，
// 事后USB接上后用log_dump命令把文件内容整份打印出来（相当于"离线补一份log"）

void print_begin();               // 初始化，默认USB/BT/文件均关闭
void print_set_usb(bool en);
void print_set_bt(bool en);
void print_set_file(bool en);     // 开关文件通道，开启时会同步一次已有文件大小
bool print_usb_enabled();
bool print_bt_enabled();
bool print_file_enabled();

void out(const char* msg);        // 向所有已开启的通道输出（含文件通道）
void out_usb(const char* msg);    // 仅USB
void out_bt(const char* msg);     // 仅BT
void out_file(const char* msg);   // 仅文件：先写入内存缓冲区，攒够了才落盘，减少flash写入频率

void print_file_flush();          // 把缓冲区里还没落盘的内容立即写入文件
void print_file_dump();           // 把/track.log整份内容打印到Serial（USB下载log用）
void print_file_clear();          // 删除/track.log，缓冲区清空，下次写入从头开始
long print_file_size();           // 当前已落盘+缓冲区里的字节数总和，无文件返回0
