#pragma once

// 自动巡线：CH4(index2)为主力压线，CH3(index1)/CH5(index3)为左右辅助修正
// 差速转向（不停顿），CH2/CH6暂不使用

void track_begin();          // 初始化，默认关闭
void track_set(bool on);     // 开启/关闭巡线模式（关闭时会停止电机）
bool track_is_on();          // 是否处于巡线模式
void track_update();         // 每次loop调用，仅开启时生效，非阻塞

float track_get_turn_ratio();          // 获取急弯外轮速度比例(0.0~1.0)，占base的百分比
void  track_set_turn_ratio(float ratio); // 设置急弯外轮速度比例（仅内存，供config_module持久化用）

float track_get_sharp_ratio();          // 获取急弯内轮反转比例(-1.0~0.0)，占base的百分比
void  track_set_sharp_ratio(float ratio); // 设置急弯内轮反转比例（仅内存，供config_module持久化用）
