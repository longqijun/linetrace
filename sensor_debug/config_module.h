#pragma once

// LittleFS + JSON 配置持久化：速度档位 + 急弯外轮速度比例 + 急弯内轮反转比例 + 5路传感器阈值
// json中无该字段/文件不存在时，使用默认值（速度12，急弯外轮比例0.65，急弯内轮反转比例-0.3，阈值为sensor_module内置默认值）

void config_begin();             // 挂载LittleFS，从/config.json加载配置（失败则用默认值）
int  config_get_speed();         // 获取当前速度档位(1~40)
void config_set_speed(int level);// 设置当前速度档位（仅内存，不写入Flash）
void config_save();              // 把当前内存中的速度档位和传感器阈值写入/config.json
void config_print();             // 打印当前配置的JSON内容（Serial+BT）
