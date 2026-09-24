#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =============================================
// PINES — sin conflictos con steppers (8-13)
// =============================================
#define PIN_TEMP     A0
#define PIN_GAS      A1
#define PIN_PRESION  A2
#define PIN_PIR      A3

#define VENT1_PIN    3    // PWM — Ventilador inyeccion
#define VENT2_PIN    5    // PWM — Ventilador extraccion
#define VENT3_PIN    6    // PWM — Ventilador emergencia gas

#define LED_ASPERSOR 2    // LED azul
#define LED_VALVULA  4    // LED rojo
#define BUZZER_PIN   7    // Buzzer

// =============================================
// UMBRALES
// =============================================
#define TEMP_INICIO_VENT   20.0
#define TEMP_MAX_VENT      60.0
#define TEMP_INCENDIO      70.0

#define GAS_ALERTA         500
#define GAS_PELIGRO        750

#define PRES_ALTA          700

// =============================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool buzzerState = false;
unsigned long timerBuzzer = 0;

// =============================================
// Wokwi NTC: mapeo lineal directo
// 0 raw = -55C,  1023 raw = 155C
// =============================================
float leerTemperatura() {
  int raw = analogRead(PIN_TEMP);
  return map(raw, 0, 1023, -55, 155);
}

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

// =============================================
void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  pinMode(VENT1_PIN,    OUTPUT);
  pinMode(VENT2_PIN,    OUTPUT);
  pinMode(VENT3_PIN,    OUTPUT);
  pinMode(LED_VALVULA,  OUTPUT);
  pinMode(LED_ASPERSOR, OUTPUT);
  pinMode(BUZZER_PIN,   OUTPUT);
  pinMode(PIN_PIR,      INPUT);

  analogWrite(VENT1_PIN, 0);
  analogWrite(VENT2_PIN, 0);
  analogWrite(VENT3_PIN, 0);
  digitalWrite(LED_VALVULA,  LOW);
  digitalWrite(LED_ASPERSOR, LOW);
  digitalWrite(BUZZER_PIN,   LOW);

  lcd.setCursor(0, 0); lcd.print("SISTEMA COCINA");
  lcd.setCursor(0, 1); lcd.print("Iniciando...");
  delay(2000);
  lcd.clear();
}

// =============================================
void loop() {
  float temperatura = leerTemperatura();
  int   gasRaw      = analogRead(PIN_GAS);
  int   presionRaw  = analogRead(PIN_PRESION);
  bool  movimiento  = digitalRead(PIN_PIR);

  int gasPct     = map(gasRaw,      0, 1023, 0, 100);
  int presionPct = map(presionRaw,  0, 1023, 0, 100);

  bool sensorError     = (temperatura < -50 || temperatura > 155);
  bool emergenciaGas   = (gasRaw >= GAS_PELIGRO);
  bool emergenciaFuego = (!sensorError && temperatura >= TEMP_INCENDIO);

  int  pwmVent1        = 0;
  int  pwmVent2        = 0;
  int  pwmVent3        = 0;
  bool cerrarValvula   = false;
  bool activarAspersor = false;
  bool activarBuzzer   = false;
  int  intervaloBuzzer = 300;

  // ==========================================
  // PRIORIDAD 1: GAS CRITICO
  // ==========================================
  if (emergenciaGas) {
    pwmVent1        = 255;
    pwmVent2        = 255;
    pwmVent3        = 255;
    cerrarValvula   = true;
    activarBuzzer   = true;
    intervaloBuzzer = 150;   // pitido rapido

  // ==========================================
  // PRIORIDAD 2: INCENDIO
  // ==========================================
  } else if (emergenciaFuego) {
    pwmVent1        = 255;
    pwmVent2        = 255;
    pwmVent3        = 0;
    activarAspersor = true;
    cerrarValvula   = true;
    activarBuzzer   = true;
    intervaloBuzzer = 250;   // pitido medio

  // ==========================================
  // OPERACION NORMAL
  // ==========================================
  } else {

    // Vent1 — proporcional a temperatura
    if (!sensorError && temperatura > TEMP_INICIO_VENT) {
      pwmVent1 = (int)map((long)temperatura,
                          (long)TEMP_INICIO_VENT,
                          (long)TEMP_MAX_VENT,
                          80, 255);
      pwmVent1 = constrain(pwmVent1, 80, 255);
    }

    // Vent2 — mismo flujo que Vent1 de base
    pwmVent2 = pwmVent1;

    // Vent2 — refuerzo si presion alta
    if (presionRaw >= PRES_ALTA) {
      int refuerzo = map(presionRaw, PRES_ALTA, 1023, 30, 100);
      pwmVent2 = constrain(pwmVent2 + refuerzo, 0, 255);
    }

    // Alerta de gas no critica — acelera Vent1 y Vent2
    if (gasRaw >= GAS_ALERTA) {
      int boost = map(gasRaw, GAS_ALERTA, GAS_PELIGRO, 50, 180);
      pwmVent1 = constrain(pwmVent1 + boost, 0, 255);
      pwmVent2 = constrain(pwmVent2 + boost, 0, 255);
    }

    // Vent3 apagado en operacion normal
    pwmVent3 = 0;
  }

  // ==========================================
  // APLICAR ACTUADORES
  // ==========================================
  analogWrite(VENT1_PIN, pwmVent1);
  analogWrite(VENT2_PIN, pwmVent2);
  analogWrite(VENT3_PIN, pwmVent3);
  digitalWrite(LED_VALVULA,  cerrarValvula   ? HIGH : LOW);
  digitalWrite(LED_ASPERSOR, activarAspersor ? HIGH : LOW);
  actualizarBuzzer(activarBuzzer, intervaloBuzzer);

  // ==========================================
  // LCD
  // ==========================================
  lcd.clear();

  if (emergenciaGas) {
    lcd.setCursor(0, 0);
    lcd.print("!!FUGA DE GAS!!");
    lcd.setCursor(0, 1);
    lcd.print("G:");
    lcd.print(gasPct);
    lcd.print("% VLV:OFF");

  } else if (emergenciaFuego) {
    lcd.setCursor(0, 0);
    lcd.print("!! INCENDIO !!");
    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print((int)temperatura);
    lcd.print("C ASP:ON");

  } else {
    // Alterna cada 2 segundos entre dos vistas
    if ((millis() / 2000) % 2 == 0) {
      // Vista 1: temperatura y gas
      lcd.setCursor(0, 0);
      lcd.print("T:");
      if (sensorError) {
        lcd.print("ERR");
      } else {
        lcd.print((int)temperatura);
        lcd.print("C");
      }
      lcd.print(" G:");
      lcd.print(gasPct);
      lcd.print("%");

      lcd.setCursor(0, 1);
      lcd.print("V1:");
      lcd.print(map(pwmVent1, 0, 255, 0, 100));
      lcd.print("% V2:");
      lcd.print(map(pwmVent2, 0, 255, 0, 100));
      lcd.print("%");

    } else {
      // Vista 2: presion y estado
      lcd.setCursor(0, 0);
      lcd.print("Pres:");
      lcd.print(presionPct);
      lcd.print("%");
      if (presionRaw >= PRES_ALTA) lcd.print(" ALTA");

      lcd.setCursor(0, 1);
      lcd.print("V3:");
      lcd.print(map(pwmVent3, 0, 255, 0, 100));
      lcd.print("%");
      if (gasRaw >= GAS_ALERTA && !emergenciaGas) lcd.print(" G-ALRT");
      if (movimiento) lcd.print(" MOV");
    }
  }

  // ==========================================
  // DEBUG SERIAL
  // ==========================================
  Serial.print("T:");   Serial.print(temperatura, 1);
  Serial.print(" G:");  Serial.print(gasPct);
  Serial.print("% P:"); Serial.print(presionPct);
  Serial.print("% V1:"); Serial.print(pwmVent1);
  Serial.print(" V2:");  Serial.print(pwmVent2);
  Serial.print(" V3:");  Serial.println(pwmVent3);

  delay(500);
}