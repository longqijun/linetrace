#pragma once

void bt_begin(const char* name);
bool bt_connected();
void bt_send(const char* msg);
bool bt_poll_line(char* buf, int maxlen);  // 带回显，收到完整行返回true
