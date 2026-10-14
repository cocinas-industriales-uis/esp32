#pragma once
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
typedef unsigned long ulong_t;
extern unsigned long g_millis;
extern int g_analog[64]; extern int g_digital[64];
extern int g_pwm[64]; extern int g_dout[64];
extern int g_pinmode[64];
inline unsigned long millis(){return g_millis;}
inline void delay(unsigned long ms){g_millis+=ms;}
inline int analogRead(int p){return g_analog[p];}
inline int digitalRead(int p){return g_digital[p];}
inline void digitalWrite(int p,int v){g_dout[p]=v;}
inline void analogWrite(int p,int v){g_pwm[p]=v;}
inline void pinMode(int p,int m){g_pinmode[p]=m;}
template<class T,class U,class V> auto constrain(T a,U lo,V hi)->T{return a<(T)lo?(T)lo:(a>(T)hi?(T)hi:a);}
inline long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}
template<class T> T max(T a,T b){return a>b?a:b;}
template<class T> T min(T a,T b){return a<b?a:b;}
struct String{ std::string s; String(const char*c=""):s(c){} String(const std::string&c):s(c){} String operator+(const String&o)const{return String(s+o.s);} };
inline String operator+(const char*a,const String&b){return String(std::string(a)+b.s);}
struct SerialT{
  void begin(int){} void print(const char*){} void print(int){} void print(float,int=2){} void print(double,int=2){}
  void println(const char*){} void println(int){} void println(String){} void println(){}
  void printf(const char*,...){}
};
extern SerialT Serial;
