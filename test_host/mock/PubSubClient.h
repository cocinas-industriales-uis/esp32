#pragma once
#include "WiFi.h"
extern bool g_mqtt_ok; extern std::vector<std::string> g_pub; extern int g_conn_attempts; extern unsigned long g_blocked_ms;
struct PubSubClient{ bool c=false; PubSubClient(WiFiClient&){}
 void setServer(const char*,int){} void setKeepAlive(int){}
 bool connected(){return c&&g_mqtt_ok;}
 bool connect(const char*){g_conn_attempts++; if(g_mqtt_ok){c=true;return true;} g_millis+=3000; g_blocked_ms+=3000; return false;}
 int state(){return -2;} void loop(){}
 bool publish(const char*,const char*b){g_pub.push_back(b);return true;} };
