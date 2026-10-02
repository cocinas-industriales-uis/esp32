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
#define BUZZER_PIN     21   // Buzzer activo

// LCD I2C en pines por defecto del ESP32: SDA=21, SCL=22
// OJO: si usas buzzer en pin 21, reasigna buzzer a pin 23

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

// Conexión WiFi
void conectarWiFi() {
  Serial.print("Conectando WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 30) {
    delay(500); Serial.print("."); intentos++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi OK — IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi FALLO — modo offline");
  }
}

// Reconexión MQTT
void reconectarMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  int intentos = 0;
  while (!mqttClient.connected() && intentos < 3) {
    Serial.print("Conectando MQTT...");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println(" OK");
    } else {
      Serial.print(" fallo (rc="); Serial.print(mqttClient.state()); Serial.println(")");
      delay(500);
    }
    intentos++;
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
                bool valvula, bool aspersor, bool confirmando) {

  if (!mqttClient.connected()) return;

  StaticJsonDocument<320> doc;
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
  doc["ts"]           = millis();

  char buffer[320];
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
  if (tempInicial < -900) tempInicial = 25.0f;
  for (int i = 0; i < HIST_N; i++) tempHistorial[i] = tempInicial;
  ultimaLectura = millis();

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("SISTEMA COCINA");
  lcd.setCursor(0, 1); lcd.print("Iniciando...");

  // WiFi + MQTT
  conectarWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setKeepAlive(30);

  delay(2000);
  lcd.clear();
}

// ================================================================
// LOOP PRINCIPAL
// ================================================================
void loop() {
  // Mantener MQTT vivo
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
  float deltaT      = calcularDeltaT(temperatura);

  bool sensorError  = (temperatura < -50.0f || temperatura > 300.0f);
  if (sensorError) temperatura = 0.0f;

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
  else if (llama && (temperatura >= TEMP_P3_MOD || dtRapido)) prioTarget = 3; // P3
  else if (presAlta && gasAlerta)                     prioTarget = 5;  // P5
  else if (gasAlerta && !gasEmergencia)               prioTarget = 4;  // P4
  else if (llama && temperatura < TEMP_P3_MOD && !dtRapido) prioTarget = 8; // P8
  else if (presAlta && !gasAlerta)                    prioTarget = 6;  // P6
  else if (!sensorError && temperatura > TEMP_INICIO_VENT) prioTarget = 7; // P7
  else                                                prioTarget = 9;  // Normal

  // ── 3. TEMPORIZADOR DE CONFIRMACIÓN ───────────────────────────
  // Sin espera: P0, P1, P2 (llama + >90°C) y P3 con ΔT/s alto.
  // Con espera de CONFIRM_MS: P3 por temperatura y P8.
  bool sinEspera  = (prioTarget == 2) || (prioTarget == 3 && dtRapido);
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

  // ── 5. APLICAR ACTUADORES ─────────────────────────────────────
  setPWM(VENT1_PIN, v1);
  setPWM(VENT2_PIN, v2);
  setPWM(VENT3_PIN, v3);
  digitalWrite(LED_VALVULA,  valvula  ? HIGH : LOW);
  digitalWrite(LED_ASPERSOR, aspersor ? HIGH : LOW);
  actualizarBuzzer(buzzer, buzzerMs);

  // ── 6. LCD ────────────────────────────────────────────────────
  lcd.clear();
  switch (prioActual) {
    case 0:
      lcd.setCursor(0,0); lcd.print("!!GAS + LLAMA!!");
      lcd.setCursor(0,1); lcd.print("EVACUAR AHORA!!");
      break;
    case 1:
      lcd.setCursor(0,0); lcd.print("!!FUGA DE GAS!!");
      lcd.setCursor(0,1);
      lcd.print("G:"); lcd.print(map(gasRaw,0,4095,0,100));
      lcd.print("% VLV:OFF");
      break;
    case 2:
      lcd.setCursor(0,0); lcd.print("!!INCEND.SEV!! ");
      lcd.setCursor(0,1);
      lcd.print("T:"); lcd.print((int)temperatura); lcd.print("C ASP:ON");
      break;
    case 3:
      lcd.setCursor(0,0); lcd.print("!!INCENDIO!!   ");
      lcd.setCursor(0,1);
      lcd.print("T:"); lcd.print((int)temperatura); lcd.print("C V2:100%");
      break;
    case 4:
      lcd.setCursor(0,0); lcd.print("GAS ALERTA     ");
      lcd.setCursor(0,1);
      lcd.print("G:"); lcd.print(map(gasRaw,0,4095,0,100)); lcd.print("% VENT+");
      break;
    case 5:
      lcd.setCursor(0,0); lcd.print("PRES+GAS ALRT  ");
      lcd.setCursor(0,1);
      lcd.print("P:"); lcd.print(map(presRaw,0,4095,0,100));
      lcd.print("% G:"); lcd.print(map(gasRaw,0,4095,0,100)); lcd.print("%");
      break;
    case 6:
      lcd.setCursor(0,0);
      lcd.print("Pres:"); lcd.print(map(presRaw,0,4095,0,100)); lcd.print("% ALTA  ");
      lcd.setCursor(0,1);
      lcd.print("V1:"); lcd.print(map(v1,0,255,0,100));
      lcd.print("% V2:"); lcd.print(map(v2,0,255,0,100)); lcd.print("%");
      break;
    case 8:
      lcd.setCursor(0,0); lcd.print("Llama detectada");
      lcd.setCursor(0,1);
      lcd.print("T:"); lcd.print((int)temperatura); lcd.print("C MONIT.");
      break;
    default: {
      // Alterna entre dos vistas cada 2s
      bool vista1 = ((millis() / 2000) % 2 == 0);
      if (vista1) {
        lcd.setCursor(0,0);
        lcd.print("T:"); lcd.print((int)temperatura);
        lcd.print("C G:"); lcd.print(map(gasRaw,0,4095,0,100)); lcd.print("%");
        lcd.setCursor(0,1);
        lcd.print("V1:"); lcd.print(map(v1,0,255,0,100));
        lcd.print("% V2:"); lcd.print(map(v2,0,255,0,100)); lcd.print("%");
      } else {
        lcd.setCursor(0,0);
        lcd.print("Pres:"); lcd.print(map(presRaw,0,4095,0,100)); lcd.print("%");
        lcd.setCursor(0,1);
        lcd.print("V3:"); lcd.print(map(v3,0,255,0,100)); lcd.print("%");
        if (llama) lcd.print(" LLAMA");
      }
      break;
    }
  }

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
               v1, v2, v3, valvula, aspersor, confirmando);
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
