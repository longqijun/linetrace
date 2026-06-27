#include "print_module.h"
#include "bt_module.h"
#include <Arduino.h>

static bool _usb = false;
static bool _bt  = false;

void print_begin()          { _usb = false; _bt = false; }
void print_set_usb(bool en) { _usb = en; }
void print_set_bt(bool en)  { _bt  = en; }
bool print_usb_enabled()    { return _usb; }
bool print_bt_enabled()     { return _bt;  }

void out_usb(const char* msg) { if (_usb) Serial.print(msg); }
void out_bt(const char* msg)  { if (_bt)  bt_send(msg); }
void out(const char* msg)     { out_usb(msg); out_bt(msg); }
