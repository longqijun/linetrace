#pragma once

// 8路传感器：CH1~CH8，引脚 D32/D33/D34/D35/VP(36)/VN(39)/D13/D14
// 白色=低值，黑色=高值（NPN光电晶体管 active-low 特性）
// CH7/CH8是ADC2，BT关闭前会跟BT/WiFi冲突读数出错，现在BT已关闭可以正常使用

#define SENSOR_COUNT 8

void sensor_begin();
void sensor_read(int values[SENSOR_COUNT]);           // 原始ADC值
void sensor_binary(bool is_white[SENSOR_COUNT]);      // true=白线, false=黑色
void sensor_binary_from(const int vals[SENSOR_COUNT], bool is_white[SENSOR_COUNT]); // 纯函数版：用调用方已读的原始值算二值，不重新读ADC
float sensor_position();  // -1.0(最左) ~ 0(居中) ~ +1.0(最右)，NAN=丢线；基于二值判断，离散阶梯值
float sensor_position_from(const bool is_white[SENSOR_COUNT]); // 纯函数版：用调用方已采样的is_white[]算位置，不重新读ADC
int sensor_get_threshold(int index);                  // 获取某路阈值
void sensor_set_threshold(int index, int value);      // 覆盖某路阈值（仅内存，供config_module加载配置用）

// 模拟量加权位置（给PID用，比二值判断更连续，见"PID算法分析.md"模拟量改进方案）：
// 每路按(黑参考值-原始值)/(黑参考值-白参考值)算"在线权重"(0~1,越接近1越压线中心)，
// 再按权重加权平均各路物理位置，不再是9档/17档离散阶梯，而是接近连续的量
float sensor_position_analog();  // 内部会重新读一次ADC
float sensor_position_analog_from(const int vals[SENSOR_COUNT]); // 纯函数版：用调用方已读的原始值算，不重新读ADC
int   sensor_get_white_ref(int index);                 // 获取某路"压线中心"参考值（白线上，ADC低值）
void  sensor_set_white_ref(int index, int value);      // 设置某路白参考值（仅内存，供config_module加载配置用）
int   sensor_get_black_ref(int index);                 // 获取某路"完全压不到线"参考值（黑色背景，ADC高值）
void  sensor_set_black_ref(int index, int value);      // 设置某路黑参考值（仅内存，供config_module加载配置用）
