#pragma once
#include "Arduino.h"
#define WL_CONNECTED 3
extern bool g_wifi_ok; extern int g_wifi_begin_calls;
struct IPA{ String toString(){return String("1.2.3.4");} };
struct WiFiClient{};
struct WiFiT{ void begin(const char*,const char*){g_wifi_begin_calls++;} int status(){return g_wifi_ok?3:6;} IPA localIP(){return IPA();} void setAutoReconnect(bool){} bool reconnect(){g_wifi_begin_calls++;return true;} };
extern WiFiT WiFi;
