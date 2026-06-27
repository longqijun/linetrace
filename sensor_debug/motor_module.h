#pragma once

void motor_begin();
void motor_stop();
void motor_brake();
void motor_set(int pwm_l, int pwm_r);  // -255~255，负值=反转
int  motor_level_to_pwm(int level);    // 1~10 → 0~255
