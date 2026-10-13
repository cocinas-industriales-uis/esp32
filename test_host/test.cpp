// Arnés de pruebas en PC: compila el .ino REAL con stubs de la API Arduino/ESP32.
#include "Arduino.h"
#include <random>
#include <algorithm>
unsigned long g_millis=0; int g_analog[64]={0}; int g_digital[64]={0}; int g_pwm[64]={0}; int g_dout[64]={0}; int g_pinmode[64]={0};
SerialT Serial; std::string g_lcd[2]; int g_lcd_writes=0; int g_lcd_clears=0;
bool g_wifi_ok=true; int g_wifi_begin_calls=0; bool g_mqtt_ok=true; std::vector<std::string> g_pub; int g_conn_attempts=0; unsigned long g_blocked_ms=0;
#include "WiFi.h"
WiFiT WiFi;
#include FW_PATH

static int rawTemp(double T){ double R=10000.0*exp(3950.0*(1.0/(T+273.15)-1.0/298.15)); return (int)(4095.0*10000.0/(10000.0+R)+0.5); }
static void inputs(double T,int gas,int pres,bool llama){ g_analog[PIN_TEMP]=rawTemp(T); g_analog[PIN_GAS]=gas; g_analog[PIN_PRESION]=pres; g_digital[PIN_LLAMA]=llama?0:1; }
static void run(unsigned long ms){ unsigned long e=g_millis+ms; while(g_millis<e) loop(); }
static int fails=0;
static int opens=0;
static void checkOpen(const char*n,bool ok,const char*det=""){ printf("  [%s] %s %s\n",ok?"PASS":"ABIERTO",n,det); if(!ok)opens++; }
static void check(const char*n,bool ok,const char*det=""){ printf("  [%s] %s %s\n",ok?"PASS":"FAIL",n,det); if(!ok)fails++; }
static double jget(const std::string&j,const char*k){ std::string key=std::string("\"")+k+"\":"; size_t p=j.find(key); if(p==std::string::npos) return -12345; p+=key.size(); if(j.compare(p,4,"true")==0) return 1; if(j.compare(p,5,"false")==0) return 0; return atof(j.c_str()+p); }
static void boot(){ setup(); }

int main(int argc,char**argv){
  std::string t=argc>1?argv[1]:""; char buf[200];
  if(t=="T01_normal25"){ inputs(25,100,500,false); boot(); run(1000); sprintf(buf,"prio=%d v1=%d v2=%d",prioActual,g_pwm[VENT1_PIN],g_pwm[VENT2_PIN]);
    check("P7 con 25C: ventilacion proporcional",prioActual==7&&g_pwm[VENT1_PIN]>=80&&g_pwm[VENT1_PIN]==g_pwm[VENT2_PIN],buf); }
  else if(t=="T02_frio15"){ inputs(15,100,500,false); boot(); run(1000); check("Normal(9) con 15C, ventiladores 0",prioActual==9&&g_pwm[VENT1_PIN]==0); }
  else if(t=="T03_gas_critico"){ inputs(25,1700,500,false); boot(); run(500); sprintf(buf,"prio=%d",prioActual);
    check("P1: 3 vent 255, valvula=1, aspersor=0",prioActual==1&&g_pwm[VENT1_PIN]==255&&g_pwm[VENT2_PIN]==255&&g_pwm[VENT3_PIN]==255&&g_dout[LED_VALVULA]==1&&g_dout[LED_ASPERSOR]==0,buf); }
  else if(t=="T04_gas_llama"){ inputs(25,1700,500,true); boot(); run(500); sprintf(buf,"prio=%d",prioActual);
    check("P0: vent 0, valvula=1, aspersor=1",prioActual==0&&g_pwm[VENT1_PIN]==0&&g_pwm[VENT2_PIN]==0&&g_pwm[VENT3_PIN]==0&&g_dout[LED_VALVULA]==1&&g_dout[LED_ASPERSOR]==1,buf); }
  else if(t=="T05_llama95"){ inputs(95,100,500,true); boot(); run(300); sprintf(buf,"prio=%d",prioActual); check("P2 inmediato (llama+95C)",prioActual==2&&g_dout[LED_ASPERSOR]==1&&g_pwm[VENT2_PIN]==0,buf); }
  else if(t=="T06_llama70"){ inputs(70,100,500,true); boot(); run(5000); int a=prioActual; bool conf=jget(g_pub.back(),"confirmando")==1; int pp=(int)jget(g_pub.back(),"prioridad");
    sprintf(buf,"t=5s prioActual=%d pub.prioridad=%d confirmando=%d",a,pp,conf); check("antes de 10 s: actuadores en Normal, publica P3 confirmando",a!=3&&pp==3&&conf,buf);
    run(6000); sprintf(buf,"t=11s prioActual=%d",prioActual); check("tras 10 s: P3 confirmado",prioActual==3&&g_pwm[VENT2_PIN]==255&&g_pwm[VENT1_PIN]==0,buf); }
  else if(t=="T07_llama25"){ inputs(25,100,500,true); boot(); run(11000); sprintf(buf,"prio=%d",prioActual); check("P8 tras 10 s (llama sola)",prioActual==8,buf); }
  else if(t=="T08_gas_prealerta"){ inputs(25,1000,500,false); boot(); run(500); sprintf(buf,"prio=%d v1=%d",prioActual,g_pwm[VENT1_PIN]); check("P4 (gas 800-1640), vent 80..200",prioActual==4&&g_pwm[VENT1_PIN]>=80&&g_pwm[VENT1_PIN]<=200,buf); }
  else if(t=="T09_pres_gas"){ inputs(25,1000,3000,false); boot(); run(500); check("P5 presion alta + gas alerta",prioActual==5&&g_pwm[VENT2_PIN]==255&&g_pwm[VENT3_PIN]==255); }
  else if(t=="T10_pres"){ inputs(25,100,3000,false); boot(); run(500); sprintf(buf,"prio=%d v1=%d v2=%d",prioActual,g_pwm[VENT1_PIN],g_pwm[VENT2_PIN]); check("P6 presion alta: v2>v1",prioActual==6&&g_pwm[VENT2_PIN]>g_pwm[VENT1_PIN],buf); }
  else if(t=="T11_llama_gas_prealerta"){ inputs(65,1000,500,true); boot(); bool buz=false; unsigned long e=g_millis+5000; while(g_millis<e){ loop(); if(g_dout[BUZZER_PIN]) buz=true; }
    sprintf(buf,"t=5s prioActual=%d valvula=%d aspersor=%d buzzer_activo=%d",prioActual,g_dout[LED_VALVULA],g_dout[LED_ASPERSOR],buz);
    check("llama(T>=60)+gas>=10%LEL: valvula cerrada y buzzer antes de los 10 s",g_dout[LED_VALVULA]==1&&buz,buf);
    check("...y el aspersor/corte de inyeccion siguen esperando la confirmacion",g_dout[LED_ASPERSOR]==0&&prioActual!=3,buf);
    run(6000); sprintf(buf,"t=11s prioActual=%d",prioActual); check("tras 10 s: P3 completo",prioActual==3&&g_dout[LED_ASPERSOR]==1&&g_pwm[VENT1_PIN]==0,buf); }
  else if(t=="T19_ntc_averiado_con_llama"){ inputs(25,100,500,true); boot(); g_analog[PIN_TEMP]=0; g_pub.clear(); run(300); sprintf(buf,"prio=%d valvula=%d aspersor=%d v1=%d v2=%d",prioActual,g_dout[LED_VALVULA],g_dout[LED_ASPERSOR],g_pwm[VENT1_PIN],g_pwm[VENT2_PIN]);
    check("NTC averiado + llama: P3 inmediato",prioActual==3&&g_dout[LED_VALVULA]==1&&g_dout[LED_ASPERSOR]==1&&g_pwm[VENT2_PIN]==255&&g_pwm[VENT1_PIN]==0,buf);
    run(5000); check("se publica prioridad 3 con sensor_ok=0",!g_pub.empty()&&jget(g_pub.back(),"prioridad")==3&&jget(g_pub.back(),"sensor_ok")==0); }
  else if(t=="T20_ntc_averiado_sin_llama"){ inputs(25,100,500,false); boot(); g_analog[PIN_TEMP]=0; run(1000); sprintf(buf,"prio=%d",prioActual); check("NTC averiado SIN llama: no se dispara P3 (solo Normal)",prioActual==9&&g_dout[LED_VALVULA]==0,buf); }
  else if(t=="T12_sensor_recupera"){ inputs(25,100,500,true); boot(); run(1000); g_analog[PIN_TEMP]=0; g_pub.clear(); run(6000); std::string e=g_pub.empty()?"{}":g_pub.back(); double tE=jget(e,"temperatura");
    inputs(25,100,500,true); int p3=0; unsigned long end=g_millis+1000; while(g_millis<end){ loop(); if(prioActual==3||prioActual==2) p3++; }
    sprintf(buf,"durante falla publica temperatura=%.1f ; tras recuperar, vueltas con P3/P2 espurio=%d",tE,p3);
    check("tras falla de sensor no se dispara P3 espurio por delta_t",p3==0,buf);
    check("durante la falla el payload permite distinguir sensor invalido (campo sensor_ok=0)",jget(e,"sensor_ok")==0,""); }
  else if(t=="T13_mqtt_caido"){ g_mqtt_ok=false; inputs(95,100,500,true); boot(); unsigned long mx=0; for(int i=0;i<2000&&g_millis<60000;i++){ unsigned long a=g_millis; loop(); mx=std::max(mx,g_millis-a);} 
    sprintf(buf,"intentos de conexion en 60 s=%d (bloqueado total %lu ms)",g_conn_attempts,g_blocked_ms);
    check("con broker caido: <=1 intento cada 10 s (sin reintentos en rafaga)",g_conn_attempts<=7,buf);
    sprintf(buf,"max vuelta de loop=%lu ms (supone 3 s de timeout TCP por intento)",mx);
    checkOpen("[pendiente hardware/FreeRTOS] con broker caido, ninguna vuelta de loop dura >200 ms",mx<=200,buf); }
  else if(t=="T14_wifi_caido_arranque"){ g_wifi_ok=false; inputs(95,100,500,true); unsigned long a=g_millis; boot(); unsigned long d=g_millis-a; sprintf(buf,"setup() tarda %lu ms",d); check("setup() con WiFi caido tarda <1 s (no retrasa la proteccion local)",d<1000,buf); }
  else if(t=="T15_lcd_parpadeo"){ inputs(25,100,500,false); boot(); g_lcd_clears=0; run(10000); sprintf(buf,"%d lcd.clear() en 10 s",g_lcd_clears); check("LCD sin clear() en cada vuelta (<=1/s)",g_lcd_clears<=10,buf); }
  else if(t=="T16_ruido_deltaT"){ std::mt19937 g2(42); std::normal_distribution<double> n(0,10); inputs(25,100,500,true); boot(); int p3=0; for(int i=0;i<1200;i++){ g_analog[PIN_TEMP]=rawTemp(25)+(int)n(g2); loop(); if(prioActual==3) p3++; }
    sprintf(buf,"ruido sigma=10 cuentas ADC (supuesto): %d de 1200 vueltas en P3 por deltaT con T=25C",p3); check("ruido ADC razonable no dispara P3 por deltaT",p3==0,buf); }
  else if(t=="T17_pines"){ int out[]={VENT1_PIN,VENT2_PIN,VENT3_PIN,LED_ASPERSOR,LED_VALVULA,BUZZER_PIN}; int in[]={PIN_TEMP,PIN_GAS,PIN_PRESION,PIN_LLAMA}; std::vector<int> all; for(int p:out)all.push_back(p); for(int p:in)all.push_back(p);
    std::sort(all.begin(),all.end()); bool uniq=std::adjacent_find(all.begin(),all.end())==all.end(); check("pines sin duplicados",uniq);
    check("buzzer no usa SDA(21) ni SCL(22) del LCD I2C",BUZZER_PIN!=21&&BUZZER_PIN!=22,""); 
    bool adc2=false; for(int p:{PIN_TEMP,PIN_GAS,PIN_PRESION}) if(p==0||p==2||p==4||p==12||p==13||p==14||p==15||p==25||p==26||p==27) adc2=true; check("sensores analogicos en ADC1 (compatible con WiFi)",!adc2); }
  else if(t=="T18_payload_cadencia"){ inputs(25,100,500,false); boot(); run(20000); size_t n7=g_pub.size(); inputs(25,1700,500,false); g_pub.clear(); run(5000); size_t n1=g_pub.size();
    const char*keys[]={"temperatura","gas_raw","gas_pct","presion_raw","presion_pct","llama","delta_t","prioridad","v1_pct","v2_pct","v3_pct","valvula","aspersor","confirmando","ts"}; bool all=true; for(auto k:keys) if(jget(g_pub.back(),k)==-12345) all=false;
    sprintf(buf,"P7: %zu msgs en 20 s ; P1: %zu msgs en 5 s",n7,n1); check("payload con las 15 claves del contrato",all);
    check("cadencia: P7~5 s (3-5 msgs/20s) y P1~0.5 s (>=9 msgs/5s)",n7>=3&&n7<=6&&n1>=9,buf); }
  else { printf("uso: t <escenario>\n"); return 2; }
  printf("  resumen: fallos=%d abiertos=%d\n",fails,opens); return fails?1:0;
}
