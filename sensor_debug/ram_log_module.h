#pragma once

#include <stddef.h>  // size_t

// track on期间的日志暂存区：track on~off全程只写内存，不碰flash（写flash那几百毫秒的
// 阻塞只会发生在track off这一刻，这时电机已经停了，不影响巡线响应）。
// 不按时间窗口截止——track on之后事件/心跳行一直写，写到整个缓冲区剩余空间低于
// RAM_LOG_RESERVE_BYTES（预留给收尾标记的余量）才不再追加新的事件/心跳行，track本身
// 继续正常运行不受影响。内存里这一趟的内容在track off之后依然原样留着（不清空），
// 直到下一次track on才会被ram_log_begin()清空覆盖——所以track off之后随时可以用
// ram_log_dump()直接从内存看当前这一趟的内容，不用等flash。
// track off时（track_module.cpp）还会调用ram_log_flush_to_file()把这份内容顺带写
// 进flash当备份：调用方会先print_file_clear()清空flash，flash里因此永远只保留
// "最新这一趟"，不会跨多趟track on/off累积——RAM那份不受这次落盘影响，继续可读。
// 这样即使还没来得及敲log dump就断电/重启（RAM会丢），flash里也留了一份最近的备份。
//
// 缓冲区分成多个固定大小的小块（chunk，20KB/份）而不是一次性malloc一大块：ESP32上
// malloc()一次要一整块连续内存经常要不到（总空闲堆够大，但碎片化导致最大连续块小很多，
// 见"LOG精简方案.md"4.2节的实测记录），改成很多份小块之后，每份只需要20KB连续空间，
// 在同样碎片化的堆上命中率高得多，能拿到的总容量也更接近"总空闲堆-安全余量"这个理想值。
// 开机时一份一份分配，直到剩余空闲堆逼近安全余量为止；每份分配后不再释放，运行期间
// 不再malloc/free，不会有堆碎片问题。多份chunk按分配顺序拼起来，dump时对外表现为
// 一份连续的log，感知不到底下是分块存的。

void ram_log_auto_init();              // setup()末尾调用一次：一份一份分配chunk，直到逼近安全余量为止
size_t ram_log_capacity();             // 所有chunk加起来的总字节数（0=一份都没分到，日志功能自动关闭，不影响巡线）

void ram_log_begin();                  // track on时调用：所有chunk清空、以当前时刻作为采集起点
void ram_log_append(const char* line); // 追加一行事件/心跳行（需自带\r\n）；缓冲区剩余空间低于预留余量或写满时静默丢弃，不阻塞、不报错
void ram_log_append_marker(const char* line); // 追加一行TRACK_ON/PARAMS/TRACK_OFF这类标记行；可以动用预留余量，只受chunk硬容量限制
void ram_log_dump();                   // 把当前内存里的内容直接打印到Serial（不经过flash），随时可调用，包括track还没off的时候
unsigned long ram_log_line_count();    // 当前缓冲区里已经追加了多少行
unsigned long ram_log_duration_ms();   // 实际捕捉到的事件/心跳数据一共覆盖了多长时间(ms)，不含TRACK_ON/PARAMS/TRACK_OFF这些标记行（如果缓冲区提前写满，会比track实际运行时间短）

void ram_log_flush_to_file();          // track off时调用：把当前内存内容写进flash当备份，不清空RAM（调用方需要先自己print_file_clear()，见上）
