#pragma once
#include "Arduino.h"
extern std::string g_lcd[2]; extern int g_lcd_writes; extern int g_lcd_clears;
struct LiquidCrystal_I2C{ int cur=0,col=0;
 LiquidCrystal_I2C(int,int,int){}
 void init(){} void backlight(){}
 void clear(){g_lcd[0]="";g_lcd[1]="";g_lcd_clears++;}
 void setCursor(int c,int r){col=c;cur=r;}
 void wr(const std::string&t){ if((int)g_lcd[cur].size()<col) g_lcd[cur].resize(col,' '); g_lcd[cur].replace(col,t.size(),t); col+=t.size(); g_lcd_writes++; }
 void print(const char*t){wr(t);} void print(int v){wr(std::to_string(v));} void print(long v){wr(std::to_string(v));}
 void print(String t){wr(t.s);} };
