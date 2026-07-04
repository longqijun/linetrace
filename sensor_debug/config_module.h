#pragma once

// LittleFS + JSON 配置持久化：目前仅保存速度档位
// json中无该字段或文件不存在时，使用默认值3

void config_begin();             // 挂载LittleFS，从/config.json加载配置（失败则用默认值）
int  config_get_speed();         // 获取当前速度档位(1~10)
void config_set_speed(int level);// 设置当前速度档位（仅内存，不写入Flash）
void config_save();              // 把当前内存中的配置写入/config.json
