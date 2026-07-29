#pragma once

// 输出通道：USB / BT / Flash文件（/track.log）三选任意组合开启
// 文件通道给BT不稳定/断连场景用：track_module的调试log正常走out()同时写进文件，
// 事后USB接上后用log_dump命令把文件内容整份打印出来（相当于"离线补一份log"）
//
// 文件通道是固定容量的环形缓冲区（写满后绕回开头覆盖最老的记录，不会像之前那样写满就停）：
// 每条记录固定长度、带独立递增的#ID前缀。dump出来的物理行顺序不代表时间顺序——
// 槽位是按 (ID % 总槽位数) 算出来的，绕回之后靠对比#ID大小才能看出真实的时间顺序和绕回点。
// #ID只存在内存里，重启会归零，不代表持久化的绝对序号。

void print_begin();               // 初始化，默认USB/BT/文件均关闭
void print_set_usb(bool en);
void print_set_bt(bool en);
void print_set_file(bool en);     // 开关文件通道；首次开启且文件未预分配好时会同步预分配整个环形文件（可能耗时数秒）
bool print_usb_enabled();
bool print_bt_enabled();
bool print_file_enabled();

void out(const char* msg);        // 向所有已开启的通道输出（含文件通道）
void out_usb(const char* msg);    // 仅USB
void out_bt(const char* msg);     // 仅BT
void out_file(const char* msg);   // 仅文件：一条完整log行当一条记录，先攒到内存缓冲区，攒够了才落盘

void print_file_flush();          // 把缓冲区里还没落盘的记录立即写入文件
void print_file_dump();           // 把/track.log整份内容（含空槽位）打印到Serial（USB下载log用）
void print_file_clear();          // 清空环形缓冲区：#ID归零，物理文件重新预分配为全空白槽位
long print_file_size();           // 环形文件的总容量字节数，未预分配过返回0（不再是"已用字节数"）
unsigned long print_file_next_id(); // 下一条即将写入的记录ID（等于目前为止累计写过的记录条数）
bool print_file_wrapped();        // 是否已经绕回过至少一圈（true=最老的记录已经开始被覆盖）
