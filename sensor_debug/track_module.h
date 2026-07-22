#pragma once

// 自动巡线，两种算法并列，运行时可切换（默认BANGBANG）：
//   TRACK_ALGO_BANGBANG：三级差速bang-bang，CH4主力压线，CH3/CH5缓转，CH2/CH6急转
//   TRACK_ALGO_PID：连续加权位置(sensor_position_from)作误差输入的PID闭环差速
// 丢线延续方向找线、十字路口/宽线直行穿过这两个行为，两种算法共用同一套判定，不受算法切换影响

#define TRACK_ALGO_BANGBANG 0
#define TRACK_ALGO_PID       1

void track_begin();          // 初始化，默认关闭
void track_set(bool on);     // 开启/关闭巡线模式（关闭时会停止电机）
bool track_is_on();          // 是否处于巡线模式
void track_update();         // 每次loop调用，仅开启时生效，非阻塞

float track_get_turn_ratio();          // 获取急弯外轮速度比例(0.0~1.0)，占base的百分比
void  track_set_turn_ratio(float ratio); // 设置急弯外轮速度比例（仅内存，供config_module持久化用）

float track_get_sharp_ratio();          // 获取急弯内轮反转比例(-1.0~0.0)，占base的百分比
void  track_set_sharp_ratio(float ratio); // 设置急弯内轮反转比例（仅内存，供config_module持久化用）

int  track_get_algo();       // 获取当前算法（TRACK_ALGO_BANGBANG/TRACK_ALGO_PID）
void track_set_algo(int algo); // 设置算法，非法值回落到BANGBANG（仅内存，供config_module持久化用）

float track_get_pid_kp();          // 获取PID比例增益
void  track_set_pid_kp(float kp);  // 设置PID比例增益（>=0，仅内存，供config_module持久化用）
float track_get_pid_ki();          // 获取PID积分增益
void  track_set_pid_ki(float ki);  // 设置PID积分增益（>=0，仅内存，供config_module持久化用）
float track_get_pid_kd();          // 获取PID微分增益
void  track_set_pid_kd(float kd);  // 设置PID微分增益（>=0，仅内存，供config_module持久化用）
