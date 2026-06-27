#pragma once

void cmd_begin();
void cmd_poll();            // 每次loop调用，处理BT输入
bool cmd_print_enabled();   // 是否允许连续打印传感器数据
