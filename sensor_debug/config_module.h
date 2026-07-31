#pragma once

#include <stddef.h>  // size_t

// LittleFS + JSON 配置持久化：速度档位 + 急弯外轮速度比例 + 急弯内轮反转比例
// + 巡线算法选择(bangbang/pid) + PID三个增益 + 文件log开关 + 5路传感器阈值
// json中无该字段/文件不存在时，使用默认值（速度12，急弯外轮比例0.65，急弯内轮反转比例-0.3，
// 算法默认bangbang，PID增益Kp40/Ki0/Kd5，文件log默认关闭，阈值为sensor_module内置默认值）

void config_begin();             // 挂载LittleFS，从/config.json加载配置（失败则用默认值）
int  config_get_speed();         // 获取当前速度档位(1~40)
void config_set_speed(int level);// 设置当前速度档位（仅内存，不写入Flash）
void config_save();              // 把当前内存中的速度档位和传感器阈值写入/config.json
void config_print();             // 打印当前配置的JSON内容（Serial+BT）

// 生成一行紧凑的"当前生效参数"快照（PARAMS行，给track_module的内存log用，见"LOG精简方案.md"4.3节）。
// 字段集合要跟config_print()保持一致（改参数记得两处都改，这是本方案在实施前就已知的取舍，
// 见该文档第7节风险），buf至少要有LOG_LINE_MAX(print_module.h)大小
void config_build_params_line(char* buf, size_t buf_size);
