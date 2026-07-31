#pragma once

// 单条log行的最大长度（含结尾\r\n和'\0'）。print_module/track_module/config_module/
// ram_log_module共用这一个常量，改行长上限只用改这一处（见"LOG精简方案.md"）
#define LOG_LINE_MAX 320

// 输出通道：USB / BT / Flash文件（/track.log.0 ~ .N-1）三选任意组合开启
// 文件通道给BT不稳定/断连场景用：track_module的调试log正常走out()同时写进文件，
// 事后USB接上后用log_dump命令把文件内容整份打印出来（相当于"离线补一份log"）
//
// 文件通道是多段文件轮转的环形缓冲区（写满不会停，绕回覆盖最老的一段）：
// 固定LOG_SEGMENT_COUNT个文件，每个文件从头到尾只做顺序追加(open "a"写)，写满一段的
// 上限就切到下一段（先删除该段旧内容再重新开始写）。之所以不用"单个大文件+seek()写任意
// 偏移量"的方案，是因为2026-07-29实测发现LittleFS对大文件的随机偏移写非常慢
// （单次seek+write能卡3~4秒，足以让整个loop()冻结、巡线电机指令跟着卡住），
// 分段后每段内部全程顺序追加，彻底避开这个坑。
//
// 每条log行带独立递增的#ID前缀。dump出来是按段号0..N-1固定顺序输出，不代表时间顺序——
// 靠对比每行的#ID大小才能看出真实的时间顺序和绕回点（同之前的约定）。
// #ID只存在内存里，重启会归零，不代表持久化的绝对序号。

void print_begin();               // 初始化，默认USB/BT/文件均关闭
void print_set_usb(bool en);
void print_set_bt(bool en);
void print_set_file(bool en);     // 开关文件通道；开启时只做一次文件大小查询，很快，不会卡顿
bool print_usb_enabled();
bool print_bt_enabled();
bool print_file_enabled();

void out(const char* msg);        // 向所有已开启的通道输出（含文件通道）
void out_usb(const char* msg);    // 仅USB
void out_bt(const char* msg);     // 仅BT
void out_file(const char* msg);   // 仅文件：一条完整log行，先攒到内存缓冲区，攒够了才顺序追加落盘

void print_file_flush();          // 把缓冲区里还没落盘的内容立即顺序追加写入当前段
void print_file_dump();           // 把所有段文件按0..N-1顺序打印到Serial（USB下载log用），开头附带一次格式说明
void print_file_clear();          // 清空所有段文件：#ID归零，当前段号归0
long print_file_size();           // 所有段加起来的总容量字节数（固定值，不代表已用字节数）
unsigned long print_file_next_id(); // 下一条即将写入的记录ID（等于目前为止累计写过的记录条数）
bool print_file_wrapped();        // 是否已经绕完一整圈（true=最老的段已经被覆盖过至少一次）

void print_log_legend();          // 打印事件行/心跳行的格式说明+modeCode编码表（见"LOG精简方案.md"第3节），仅走Serial
