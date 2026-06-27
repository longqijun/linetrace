#include "motor_module.h"
#include <Arduino.h>

#define PIN_EEP  17
#define PIN_IN1  18  // 右电机
#define PIN_IN2  19
#define PIN_IN3  21  // 左电机
#define PIN_IN4  22

// 1~10档对应PWM值（参考monitor_test.txt）
static const int SPEED_TABLE[11] = {0, 25, 51, 76, 102, 127, 153, 178, 204, 229, 255};

int motor_level_to_pwm(int level) {
  if (level < 1)  level = 1;
  if (level > 10) level = 10;
  return SPEED_TABLE[level];
}

void motor_begin() {
  pinMode(PIN_EEP, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  motor_stop();
  digitalWrite(PIN_EEP, HIGH);  // 使能DRV8833
}

void motor_stop() {
  analogWrite(PIN_IN1, 0);
  analogWrite(PIN_IN2, 0);
  analogWrite(PIN_IN3, 0);
  analogWrite(PIN_IN4, 0);
}

void motor_brake() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, HIGH);
}

// pwm_l/pwm_r: -255~255，正=前进，负=后退
void motor_set(int pwm_l, int pwm_r) {
  // 右电机
  if (pwm_r >= 0) {
    analogWrite(PIN_IN1, pwm_r);
    analogWrite(PIN_IN2, 0);
  } else {
    analogWrite(PIN_IN1, 0);
    analogWrite(PIN_IN2, -pwm_r);
  }
  // 左电机
  if (pwm_l >= 0) {
    analogWrite(PIN_IN3, pwm_l);
    analogWrite(PIN_IN4, 0);
  } else {
    analogWrite(PIN_IN3, 0);
    analogWrite(PIN_IN4, -pwm_l);
  }
}
