#pragma once

// 自动巡线：CH4(index2)为主力压线，CH3(index1)/CH5(index3)为左右辅助修正
// 差速转向（不停顿），CH2/CH6暂不使用

void track_begin();          // 初始化，默认关闭
void track_set(bool on);     // 开启/关闭巡线模式（关闭时会停止电机）
bool track_is_on();          // 是否处于巡线模式
void track_update();         // 每次loop调用，仅开启时生效，非阻塞
