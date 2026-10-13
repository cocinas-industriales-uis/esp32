// ================================================================
// Sistema IoT de Seguridad — Cocinas Industriales
// Universidad Industrial de Santander (UIS) — 2026
// Autor: Cesar Daniel Ávila Barbosa
//
// Hardware: ESP32 DevKit
// Sensores: NTC (A0), MQ-2 (A1), Potenciómetro presión (A2),
//           KY-026 llama (GPIO14)
// Actuadores: L293D × 2, LEDs, Buzzer, LCD 16×2 I2C
// Comunicación: WiFi + MQTT (broker Mosquitto)
// ================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

// ================================================================
// CONFIGURACIÓN WiFi + MQTT
// ================================================================
const char* WIFI_SSID      = "TU_RED_WIFI";
const char* WIFI_PASSWORD  = "TU_CLAVE_WIFI";
const char* MQTT_BROKER    = "192.168.1.X";   // IP de tu PC con Mosquitto
const int   MQTT_PORT      = 1883;
const char* MQTT_TOPIC     = "cocinas/cocina_01";
const char* MQTT_CLIENT_ID = "esp32_cocina_01";

// ================================================================
// PINES ESP32
// ================================================================
#define PIN_TEMP       34   // ADC — NTC temperatura (solo lectura)
#define PIN_GAS        35   // ADC — MQ-2 gas GLP (solo lectura)
#define PIN_PRESION    32   // ADC — Potenciómetro presión (solo lectura)
#define PIN_LLAMA      14   // Digital — KY-026 (LOW = llama detectada). No usar GPIO 6-11 (flash)

#define VENT1_PIN      25   // PWM — Ventilador inyección
#define VENT2_PIN      26   // PWM — Ventilador extracción
#define VENT3_PIN      27   // PWM — Ventilador emergencia gas

#define LED_ASPERSOR   18   // LED azul — aspersores
#define LED_VALVULA    19   // LED rojo — válvula gas
#define BUZZER_PIN     23   // Buzzer activo (NO usar 21/22: son SDA/SCL del LCD I2C)

// LCD I2C en pines por defecto del ESP32: SDA=21, SCL=22

// ================================================================
// UMBRALES — ajustar tras calibración con gas real
// ================================================================

// Temperatura (°C)
#define TEMP_INICIO_VENT    20.0f   // Arranca ventilación proporcional
#define TEMP_MAX_VENT       60.0f   // Velocidad máxima proporcional
#define TEMP_P3_MOD         60.0f   // Incendio moderado (con llama)
#define TEMP_P2_SEV         90.0f   // Incendio severo (con llama)

// Gas MQ-2 (raw ADC 0–4095 en ESP32)
// Correlación aproximada GLP: raw 800≈10%LEL · raw 1640≈20%LEL · raw 3075≈40%LEL
#define GAS_LEL_LOW      800    // ~10% LEL — prealerta P4
#define GAS_LEL_HIGH    1640    // ~20% LEL — emergencia P1

// Presión (raw ADC 0–4095)
#define PRES_ALTA       2786    // ~68% — activa P6

// ΔT/s — velocidad de cambio de temperatura
#define DELTA_T_UMBRAL   2.0f   // °C/s — escala a P3 con llama activa

// Confirmación (ms). P0, P1 y P2 actúan SIN espera; P3 también cuando ΔT/s es alto.
// Solo P3 por temperatura y P8 esperan CONFIRM_MS de condición sostenida.
#define CONFIRM_MS      10000   // 10 segundos

// Antirrebote de llama: el KY-026 parpadea; se mantiene "llama" este tiempo
// tras la última lectura positiva para no reiniciar el temporizador de confirmación.
#define LLAMA_HOLD_MS    1500

// ================================================================
// INTERVALOS DE ENVÍO MQTT
// ================================================================
#define INTERVALO_EMERGENCIA    500    // P0–P3: 0.5s
#define INTERVALO_PREALERTA    1000    // P4–P5: 1s
#define INTERVALO_NORMAL       5000    // P6–P8, Normal: 5s

// ================================================================
// NTC — Steinhart-Hart
// ================================================================
#define NTC_NOMINAL     10000.0f   // 10kΩ a 25°C
#define TEMP_NOMINAL       25.0f   // °C
#define B_COEFF          3950.0f   // coeficiente B del NTC
#define SERIE_RESISTOR  10000.0f   // resistencia en serie del divisor

// ================================================================
// HISTORIAL PARA ΔT/s
// ================================================================
#define HIST_N 10
float tempHistorial[HIST_N];
int   histIdx = 0;
unsigned long ultimaLectura = 0;

// ================================================================
// ESTADO DEL SISTEMA
// ================================================================
int   prioActual        = 9;   // 9 = Normal
int   prioConfirmando   = -1;
unsigned long tsConfirmInicio = 0;

unsigned long ultimoEnvio   = 0;
unsigned long ultimaLlamaTs = 0;   // última vez que el KY-026 leyó llama
int           prioEnvioPrev = 9;   // prioridad usada en el ciclo anterior para decidir el envío
unsigned long timerBuzzer   = 0;
bool          buzzerState   = false;

// ================================================================
// OBJETOS
// ================================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient        wifiClient;
PubSubClient      mqttClient(wifiClient);

// LCD: solo se reescribe una línea cuando cambia su contenido (evita lcd.clear()
// cada 50 ms, que provocaba parpadeo).
char lcdBuf[2][17] = {"", ""};
void lcdLinea(int fila, const char* txt) {
  char l[17];
  snprintf(l, sizeof(l), "%-16s", txt);
  if (strcmp(l, lcdBuf[fila]) != 0) {
    lcd.setCursor(0, fila);
    lcd.print(l);
    strcpy(lcdBuf[fila], l);
  }
}

// ================================================================
// FUNCIONES AUXILIARES
// ================================================================

// Convierte raw ADC ESP32 (0–4095) a temperatura °C usando Steinhart-Hart
float leerTemperatura() {
  int raw = analogRead(PIN_TEMP);
  if (raw <= 0 || raw >= 4095) return -999.0f;

  float voltaje    = raw * (3.3f / 4095.0f);
  float resistencia = SERIE_RESISTOR * (3.3f / voltaje - 1.0f);

  float steinhart = resistencia / NTC_NOMINAL;
  steinhart = log(steinhart);
  steinhart /= B_COEFF;
  steinhart += 1.0f / (TEMP_NOMINAL + 273.15f);
  steinhart  = 1.0f / steinhart;
  steinhart -= 273.15f;
  return steinhart;
}

// Calcula ΔT/s con historial circular
float calcularDeltaT(float tempActual) {
  unsigned long ahora = millis();
  float dt = (ahora - ultimaLectura) / 1000.0f;
  ultimaLectura = ahora;

  tempHistorial[histIdx % HIST_N] = tempActual;
  histIdx++;

  float oldest = tempHistorial[histIdx % HIST_N];
  float newest = tempActual;
  float ventana = HIST_N * max(dt, 0.05f);

  return (newest - oldest) / ventana;
}

// Buzzer no bloqueante
void actualizarBuzzer(bool activo, int intervalo) {
  if (!activo) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
    return;
  }
  if (millis() - timerBuzzer >= (unsigned long)intervalo) {
    timerBuzzer = millis();
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
  }
}

// PWM en ESP32 (0–255 → 0–255, igual que Arduino)
void setPWM(int pin, int valor) {
  analogWrite(pin, constrain(valor, 0, 255));
}

// Reintentos de red: NO bloqueantes. La protección local (sensores → actuadores)
// debe funcionar aunque no haya WiFi ni broker.
#define REINTENTO_WIFI_MS   10000
#define REINTENTO_MQTT_MS   10000
unsigned long ultimoIntentoWiFi = 0;
unsigned long ultimoIntentoMQTT = 0;
bool          primerIntentoMQTT = true;   // el primer intento no espera REINTENTO_MQTT_MS

// Conexión WiFi: solo lanza el intento; el loop() sigue funcionando sin red.
void conectarWiFi() {
  Serial.println("Conectando WiFi (en segundo plano)...");
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  ultimoIntentoWiFi = millis();
}

// Reintenta WiFi cada REINTENTO_WIFI_MS si está caído
void mantenerWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - ultimoIntentoWiFi < REINTENTO_WIFI_MS) return;
  ultimoIntentoWiFi = millis();
  Serial.println("WiFi caído — reintentando");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// Reconexión MQTT: un único intento cada REINTENTO_MQTT_MS, sin delay().
// OJO: connect() puede bloquear hasta el timeout TCP del cliente WiFi (~3 s por
// intento) — pendiente validar en hardware / mover red a una tarea FreeRTOS.
void reconectarMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!primerIntentoMQTT && millis() - ultimoIntentoMQTT < REINTENTO_MQTT_MS) return;
  primerIntentoMQTT = false;
  ultimoIntentoMQTT = millis();
  Serial.print("Conectando MQTT...");
  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    Serial.println(" OK");
  } else {
    Serial.print(" fallo (rc="); Serial.print(mqttClient.state()); Serial.println(")");
  }
}

// Devuelve el intervalo de envío según prioridad
unsigned long getIntervalo(int prio) {
  if (prio <= 3) return INTERVALO_EMERGENCIA;   // P0–P3: 0.5s
  if (prio <= 5) return INTERVALO_PREALERTA;    // P4–P5: 1s
  return INTERVALO_NORMAL;                      // P6–P8, Normal: 5s
}

// Envía payload JSON por MQTT
void enviarMQTT(float temp, int gasRaw, int presRaw, bool llama,
                float deltaT, int prio,
                int v1, int v2, int v3,
                bool valvula, bool aspersor, bool confirmando,
                bool sensorOk) {

  if (!mqttClient.connected()) return;

  StaticJsonDocument<352> doc;
  doc["temperatura"]  = round(temp * 10) / 10.0;
  doc["gas_raw"]      = gasRaw;
  doc["gas_pct"]      = map(gasRaw, 0, 4095, 0, 100);
  doc["presion_raw"]  = presRaw;
  doc["presion_pct"]  = map(presRaw, 0, 4095, 0, 100);
  doc["llama"]        = llama;
  doc["delta_t"]      = round(deltaT * 100) / 100.0;
  doc["prioridad"]    = prio;
  doc["v1_pct"]       = map(v1, 0, 255, 0, 100);
  doc["v2_pct"]       = map(v2, 0, 255, 0, 100);
  doc["v3_pct"]       = map(v3, 0, 255, 0, 100);
  doc["valvula"]      = valvula;
  doc["aspersor"]     = aspersor;
  doc["confirmando"]  = confirmando;   // true = condición detectada, aún sin confirmar
  doc["sensor_ok"]    = sensorOk;      // false = NTC fuera de rango; "temperatura" (0) NO es una medida válida
  doc["ts"]           = millis();

  char buffer[352];
  serializeJson(doc, buffer);
  mqttClient.publish(MQTT_TOPIC, buffer);

  Serial.print("MQTT → prio:");
  Serial.print(prio);
  Serial.print(" T:");
  Serial.print(temp, 1);
  Serial.print(" G:");
  Serial.print(doc["gas_pct"].as<int>());
  Serial.println("%");
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  // Pines de salida
  pinMode(VENT1_PIN,    OUTPUT);
  pinMode(VENT2_PIN,    OUTPUT);
  pinMode(VENT3_PIN,    OUTPUT);
  pinMode(LED_VALVULA,  OUTPUT);
  pinMode(LED_ASPERSOR, OUTPUT);
  pinMode(BUZZER_PIN,   OUTPUT);
  pinMode(PIN_LLAMA,    INPUT_PULLUP);  // KY-026: LOW = llama

  // Apagar todo al inicio
  setPWM(VENT1_PIN, 0); setPWM(VENT2_PIN, 0); setPWM(VENT3_PIN, 0);
  digitalWrite(LED_VALVULA,  LOW);
  digitalWrite(LED_ASPERSOR, LOW);
  digitalWrite(BUZZER_PIN,   LOW);

  // Inicializar historial de temperatura
  float tempInicial = leerTemperatura();
  if (tempInicial < -50.0f || tempInicial > 300.0f) tempInicial = 25.0f;   // lectura inválida
  for (int i = 0; i < HIST_N; i++) tempHistorial[i] = tempInicial;
  ultimaLectura = millis();

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("SISTEMA COCINA");
  lcd.setCursor(0, 1); lcd.print("Iniciando...");

  // WiFi + MQTT (no bloqueante: la protección local arranca de inmediato)
  conectarWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setKeepAlive(30);

  delay(500);   // solo para ver el mensaje de arranque
  lcd.clear();
  lcdBuf[0][0] = lcdBuf[1][0] = '\0';
}

// ================================================================
// LOOP PRINCIPAL
// ================================================================
void loop() {
  // Mantener red (sin bloquear la protección local)
  mantenerWiFi();
  if (!mqttClient.connected()) reconectarMQTT();
  mqttClient.loop();

  // ── 1. LEER SENSORES ──────────────────────────────────────────
  float temperatura = leerTemperatura();
  int   gasRaw      = analogRead(PIN_GAS);
  int   presRaw     = analogRead(PIN_PRESION);
  bool  llamaRaw    = (digitalRead(PIN_LLAMA) == LOW);  // KY-026 activo bajo
  if (llamaRaw) ultimaLlamaTs = millis();
  bool  llama       = llamaRaw ||
                      (ultimaLlamaTs != 0 && (millis() - ultimaLlamaTs) < LLAMA_HOLD_MS);

  // Con el NTC fuera de rango NO se alimenta el historial de ΔT/s (evita un
  // pico espurio al recuperarse el sensor) y se marca sensor_ok=false en el payload.
  bool sensorError  = (temperatura < -50.0f || temperatura > 300.0f);
  float deltaT      = 0.0f;
  if (sensorError) {
    temperatura   = 0.0f;
    ultimaLectura = millis();
  } else {
    deltaT = calcularDeltaT(temperatura);
  }

  // Flags de condición
  bool gasAlerta    = (gasRaw >= GAS_LEL_LOW);
  bool gasEmergencia = (gasRaw >= GAS_LEL_HIGH);
  bool presAlta     = (presRaw >= PRES_ALTA);
  bool dtRapido     = (deltaT >= DELTA_T_UMBRAL);

  // ── 2. CALCULAR PRIORIDAD ─────────────────────────────────────
  int prioTarget = 9;  // 9 = Normal

  if (llama && gasEmergencia)                         prioTarget = 0;  // P0
  else if (gasEmergencia && !llama)                   prioTarget = 1;  // P1
  else if (llama && temperatura > TEMP_P2_SEV)        prioTarget = 2;  // P2
  // Decisión de diseño: con llama y NTC averiado (temperatura no confiable) se asume P3, sin espera
  else if (llama && (temperatura >= TEMP_P3_MOD || dtRapido || sensorError)) prioTarget = 3; // P3
  else if (presAlta && gasAlerta)                     prioTarget = 5;  // P5
  else if (gasAlerta && !gasEmergencia)               prioTarget = 4;  // P4
  else if (llama && temperatura < TEMP_P3_MOD && !dtRapido) prioTarget = 8; // P8
  else if (presAlta && !gasAlerta)                    prioTarget = 6;  // P6
  else if (!sensorError && temperatura > TEMP_INICIO_VENT) prioTarget = 7; // P7
  else                                                prioTarget = 9;  // Normal

  // ── 3. TEMPORIZADOR DE CONFIRMACIÓN ───────────────────────────
  // Sin espera: P0, P1, P2 (llama + >90°C) y P3 con ΔT/s alto o NTC averiado.
  // Con espera de CONFIRM_MS: P3 por temperatura y P8.
  bool sinEspera  = (prioTarget == 2) || (prioTarget == 3 && (dtRapido || sensorError));
  bool confirmado = true;

  if (!sinEspera && (prioTarget == 3 || prioTarget == 8)) {
    if (prioTarget != prioConfirmando) {
      // Nueva condición — iniciar temporizador
      prioConfirmando = prioTarget;
      tsConfirmInicio = millis();
      confirmado      = false;
    } else {
      confirmado = (millis() - tsConfirmInicio >= CONFIRM_MS);
    }
  } else {
    prioConfirmando = -1;
  }

  // Mientras no está confirmada, los actuadores siguen en Normal (9).
  // El ENVÍO MQTT, en cambio, usa la prioridad detectada (ver sección 7).
  prioActual = confirmado ? prioTarget : 9;
  bool confirmando = !confirmado;

  // ── 4. LÓGICA DE ACTUADORES ───────────────────────────────────
  int  v1 = 0, v2 = 0, v3 = 0;
  bool valvula  = false;
  bool aspersor = false;
  bool buzzer   = false;
  int  buzzerMs = 300;

  switch (prioActual) {

    case 0:  // P0 — CRÍTICA: Gas + Llama → TODO OFF
      v1 = 0; v2 = 0; v3 = 0;
      valvula = true; aspersor = true;
      buzzer = true; buzzerMs = 100;
      break;

    case 1:  // P1 — EMERGENCIA GAS → Ventilación total
      v1 = 255; v2 = 255; v3 = 255;
      valvula = true; aspersor = false;
      buzzer = true; buzzerMs = 150;
      break;

    case 2:  // P2 — INCENDIO SEVERO → Todo OFF + aspersores
      v1 = 0; v2 = 0; v3 = 0;
      valvula = true; aspersor = true;
      buzzer = true; buzzerMs = 150;
      break;

    case 3:  // P3 — INCENDIO MODERADO → Solo extracción
      v1 = 0; v2 = 255; v3 = 0;
      valvula = true; aspersor = true;
      buzzer = true; buzzerMs = 250;
      break;

    case 4:  // P4 — PREALERTA GAS → Ventilación acelerada
      {
        int boost = map(gasRaw, GAS_LEL_LOW, GAS_LEL_HIGH, 100, 200);
        v1 = constrain(boost, 80, 200);
        v2 = constrain(boost, 80, 200);
        v3 = 0;
        buzzer = true; buzzerMs = 600;
      }
      break;

    case 5:  // P5 — PRESIÓN + GAS → Extracción máxima
      v1 = 0; v2 = 255; v3 = 255;
      buzzer = true; buzzerMs = 400;
      break;

    case 6:  // P6 — PRESIÓN ALTA → Diferencial
      {
        int base = !sensorError && temperatura > TEMP_INICIO_VENT
                   ? map((int)temperatura, (int)TEMP_INICIO_VENT, (int)TEMP_MAX_VENT, 80, 200)
                   : 60;
        base  = constrain(base, 60, 200);
        v1 = constrain((int)(base * 0.65f), 0, 200);
        v2 = constrain((int)(base * 1.35f), 0, 255);
        v3 = 0;
      }
      break;

    case 7:  // P7 — TEMPERATURA ALTA → Proporcional
      if (!sensorError && temperatura > TEMP_INICIO_VENT) {
        int pwm = map((int)temperatura, (int)TEMP_INICIO_VENT, (int)TEMP_MAX_VENT, 80, 255);
        v1 = constrain(pwm, 80, 255);
        v2 = v1;
      }
      break;

    case 8:  // P8 — MONITOREO LLAMA → Normal sin alarma
      if (!sensorError && temperatura > TEMP_INICIO_VENT) {
        int pwm = map((int)temperatura, (int)TEMP_INICIO_VENT, (int)TEMP_MAX_VENT, 80, 180);
        v1 = constrain(pwm, 60, 180);
        v2 = v1;
      }
      break;

    default:  // Normal
      if (!sensorError && temperatura > TEMP_INICIO_VENT) {
        int pwm = map((int)temperatura, (int)TEMP_INICIO_VENT, (int)TEMP_MAX_VENT, 80, 255);
        v1 = constrain(pwm, 80, 255);
        v2 = v1;
      }
      break;
  }

  // Decisión de diseño: llama + gas en prealerta (≥ GAS_LEL_LOW) mientras P3 aún se confirma
  // → válvula cerrada y buzzer YA; aspersor y corte de inyección esperan la confirmación.
  if (!confirmado && prioTarget == 3 && gasAlerta) {
    valvula  = true;
    buzzer   = true;
    buzzerMs = 250;
  }

  // ── 5. APLICAR ACTUADORES ─────────────────────────────────────
  setPWM(VENT1_PIN, v1);
  setPWM(VENT2_PIN, v2);
  setPWM(VENT3_PIN, v3);
  digitalWrite(LED_VALVULA,  valvula  ? HIGH : LOW);
  digitalWrite(LED_ASPERSOR, aspersor ? HIGH : LOW);
  actualizarBuzzer(buzzer, buzzerMs);

  // ── 6. LCD ────────────────────────────────────────────────────
  char l0[40], l1[40], tTxt[8];
  int  gasPct  = map(gasRaw, 0, 4095, 0, 100);
  int  presPct = map(presRaw, 0, 4095, 0, 100);
  if (sensorError) snprintf(tTxt, sizeof(tTxt), "ERR");
  else             snprintf(tTxt, sizeof(tTxt), "%dC", (int)temperatura);
  switch (prioActual) {
    case 0:
      snprintf(l0, sizeof(l0), "!!GAS + LLAMA!!");
      snprintf(l1, sizeof(l1), "EVACUAR AHORA!!");
      break;
    case 1:
      snprintf(l0, sizeof(l0), "!!FUGA DE GAS!!");
      snprintf(l1, sizeof(l1), "G:%d%% VLV:OFF", gasPct);
      break;
    case 2:
      snprintf(l0, sizeof(l0), "!!INCEND.SEV!!");
      snprintf(l1, sizeof(l1), "T:%s ASP:ON", tTxt);
      break;
    case 3:
      snprintf(l0, sizeof(l0), "!!INCENDIO!!");
      snprintf(l1, sizeof(l1), "T:%s V2:100%%", tTxt);
      break;
    case 4:
      snprintf(l0, sizeof(l0), "GAS ALERTA");
      snprintf(l1, sizeof(l1), "G:%d%% VENT+", gasPct);
      break;
    case 5:
      snprintf(l0, sizeof(l0), "PRES+GAS ALRT");
      snprintf(l1, sizeof(l1), "P:%d%% G:%d%%", presPct, gasPct);
      break;
    case 6:
      snprintf(l0, sizeof(l0), "Pres:%d%% ALTA", presPct);
      snprintf(l1, sizeof(l1), "V1:%d%% V2:%d%%", map(v1,0,255,0,100), map(v2,0,255,0,100));
      break;
    case 8:
      snprintf(l0, sizeof(l0), "Llama detectada");
      snprintf(l1, sizeof(l1), "T:%s MONIT.", tTxt);
      break;
    default: {
      // Alterna entre dos vistas cada 2s
      bool vista1 = ((millis() / 2000) % 2 == 0);
      if (vista1) {
        snprintf(l0, sizeof(l0), "T:%s G:%d%%", tTxt, gasPct);
        snprintf(l1, sizeof(l1), "V1:%d%% V2:%d%%", map(v1,0,255,0,100), map(v2,0,255,0,100));
      } else {
        snprintf(l0, sizeof(l0), "Pres:%d%%", presPct);
        snprintf(l1, sizeof(l1), "V3:%d%%%s", map(v3,0,255,0,100), llama ? " LLAMA" : "");
      }
      break;
    }
  }
  lcdLinea(0, l0);
  lcdLinea(1, l1);

  // ── 7. ENVÍO MQTT ADAPTATIVO ──────────────────────────────────
  // El intervalo depende de lo que se DETECTA (prioTarget), no de lo confirmado:
  // llama + temperatura alta debe reportarse a 0.5 s desde el primer instante.
  unsigned long ahora     = millis();
  int           prioEnvio = confirmado ? prioActual : prioTarget;
  unsigned long intervalo = getIntervalo(prioEnvio);

  // Al escalar a P0–P3 se envía de inmediato, sin esperar el intervalo anterior
  bool escalada = (prioEnvio <= 3 && prioEnvioPrev > 3);
  prioEnvioPrev = prioEnvio;

  if (escalada || (ahora - ultimoEnvio >= intervalo)) {
    ultimoEnvio = ahora;
    enviarMQTT(temperatura, gasRaw, presRaw, llama, deltaT, prioEnvio,
               v1, v2, v3, valvula, aspersor, confirmando, !sensorError);
  }

  // ── 8. DEBUG SERIAL ───────────────────────────────────────────
  Serial.printf("P%d | T:%.1f G:%d%% P:%d%% L:%d ΔT:%.2f | V1:%d V2:%d V3:%d | int:%lums\n",
    prioActual,
    temperatura,
    map(gasRaw,0,4095,0,100),
    map(presRaw,0,4095,0,100),
    llama,
    deltaT,
    map(v1,0,255,0,100),
    map(v2,0,255,0,100),
    map(v3,0,255,0,100),
    intervalo
  );

  // Loop rápido — actuadores y buzzer necesitan ~50ms de resolución
  delay(50);
}
