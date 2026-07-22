#pragma once

// 5路传感器：CH2~CH6，引脚 D33/D34/D35/VP(36)/VN(39)
// 白色=低值，黑色=高值（NPN光电晶体管 active-low 特性）

#define SENSOR_COUNT 5

void sensor_begin();
void sensor_read(int values[SENSOR_COUNT]);           // 原始ADC值
void sensor_binary(bool is_white[SENSOR_COUNT]);      // true=白线, false=黑色
float sensor_position();  // -1.0(最左) ~ 0(居中) ~ +1.0(最右)，NAN=丢线
float sensor_position_from(const bool is_white[SENSOR_COUNT]); // 纯函数版：用调用方已采样的is_white[]算位置，不重新读ADC
int sensor_get_threshold(int index);                  // 获取某路阈值
void sensor_set_threshold(int index, int value);      // 覆盖某路阈值（仅内存，供config_module加载配置用）
