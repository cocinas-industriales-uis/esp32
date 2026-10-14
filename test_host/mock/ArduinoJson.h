#pragma once
#include "Arduino.h"
#include <type_traits>
struct JV{ double d=0; bool isb=false; 
 template<class T> JV& operator=(T v){ if constexpr(std::is_same<T,bool>::value){isb=true;d=v;} else {isb=false;d=(double)v;} return *this;}
 template<class T> T as() const {return (T)d;} };
template<int N> struct StaticJsonDocument{ std::vector<std::pair<std::string,JV>> kv;
 JV& operator[](const char*k){ for(auto&p:kv) if(p.first==k) return p.second; kv.push_back({k,JV()}); return kv.back().second; } };
template<int N> size_t serializeJson(StaticJsonDocument<N>&doc,char*buf){ std::string s="{"; bool f=true; for(auto&p:doc.kv){ if(!f)s+=","; f=false; char t[64]; if(p.second.isb) snprintf(t,64,"%s",p.second.d?"true":"false"); else snprintf(t,64,"%.10g",p.second.d); s+="\""+p.first+"\":"+t;} s+="}"; strcpy(buf,s.c_str()); return s.size(); }
