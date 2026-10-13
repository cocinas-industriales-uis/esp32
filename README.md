# ESP32 — Firmware de seguridad para cocinas industriales

Firmware del nodo físico (ESP32 DevKit, Arduino core ESP32 **3.x**, por `analogWrite()`). Lee 4 sensores, aplica una
lógica de prioridades P0–P8 (Normal = 9), actúa de forma autónoma sobre 3 ventiladores, válvula, aspersor y buzzer,
muestra el estado en un LCD 16×2 I2C y publica JSON por MQTT. **La protección local no depende de la red.**

> Estado de validación: lógica probada en PC con stubs (`test_host/`); **no compilado con el toolchain real del ESP32 ni
> probado en hardware** (ver "Pendientes").

## Librerías
`LiquidCrystal_I2C`, `PubSubClient`, `ArduinoJson` (v6, `StaticJsonDocument`), `WiFi` (incluida en el core ESP32).
Antes de cargar edite en el `.ino`: `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_BROKER`, `MQTT_TOPIC`, `MQTT_CLIENT_ID`.
El `.ino` debe estar en una carpeta con su mismo nombre para el Arduino IDE.

## Pines (ESP32)
| Función | Pin | Notas |
|---|---|---|
| NTC temperatura | GPIO34 | ADC1, solo entrada |
| Gas MQ-2 | GPIO35 | ADC1, solo entrada |
| Presión (potenciómetro) | GPIO32 | ADC1 |
| Llama KY-026 | GPIO14 | `INPUT_PULLUP`, LOW = llama |
| Vent1 inyección / Vent2 extracción / Vent3 emergencia | GPIO25 / 26 / 27 | PWM |
| LED aspersor / LED válvula | GPIO18 / 19 | |
| Buzzer | **GPIO23** | antes GPIO21, que es SDA del LCD (corregido; si ya cableó el buzzer a 21, muévalo) |
| LCD I2C 0x27 | SDA 21 / SCL 22 | |

## Lógica de prioridades (la del código)
| P | Condición | Respuesta | Confirmación |
|---|---|---|---|
| 0 | llama + gas ≥ 1640 | vent 0/0/0, válvula cerrada, aspersor ON | inmediata |
| 1 | gas ≥ 1640 (~20 % LEL), sin llama | vent 255/255/255, válvula cerrada | inmediata |
| 2 | llama + T > 90 °C | vent 0, aspersor ON, válvula cerrada | inmediata |
| 3 | llama + (T ≥ 60 °C o ΔT ≥ 2 °C/s o NTC averiado) | solo extracción, aspersor, válvula | ΔT alto o NTC averiado: inmediata; por T: 10 s |
| 4 | gas 800–1640 (~10–20 % LEL) | ventilación 80–200 | inmediata |
| 5 | presión ≥ 2786 y gas ≥ 800 | extracción máxima | inmediata |
| 6 | presión ≥ 2786 | diferencial (V2 > V1) | inmediata |
| 7 | T > 20 °C | PWM proporcional 80–255 | inmediata |
| 8 | solo llama | monitoreo sin alarma | 10 s |
| 9 | resto | normal | — |

**Decisiones de diseño (usuario, 2026-10-12):** (a) si mientras P3 se confirma hay gas ≥ 800, la válvula se cierra y el buzzer suena de inmediato (aspersor y corte de inyección esperan la confirmación); (b) llama + NTC averiado se trata como P3 sin esperar.

Mientras P3/P8 por temperatura se confirman, los actuadores siguen en "normal" pero el **MQTT ya publica la prioridad
detectada** con `confirmando=true`. Umbrales gas/presión en cuentas ADC de 12 bits (0–4095), por calibrar con gas real.

## Payload MQTT (`cocinas/cocina_01`)
`temperatura, gas_raw, gas_pct, presion_raw, presion_pct, llama, delta_t, prioridad, v1_pct, v2_pct, v3_pct, valvula,
aspersor, confirmando, sensor_ok, ts`. `valvula=true` significa válvula **cerrada**. `sensor_ok=false` indica NTC fuera
de rango: `temperatura` vale 0 y no es una medida. Cadencia: P0–P3 cada 0,5 s (y envío inmediato al escalar), P4–P5 cada
1 s, resto cada 5 s.

## Comportamiento sin red
WiFi se inicia sin bloquear; se reintenta cada 10 s. MQTT: un intento cada 10 s. El loop de control corre cada ~50 ms.

## Pruebas en PC
```bash
cd test_host && ./run.sh        # compila el .ino real contra stubs de la API Arduino y ejecuta 20 escenarios
```
Los stubs no modelan el ADC real, el PWM ni los tiempos de WiFi/TCP; los escenarios marcados `ABIERTO` son pendientes.

## Pendientes conocidos
1. `mqttClient.connect()` puede bloquear ~3 s por intento con broker caído; evaluar tarea FreeRTOS para la red.
2. Compilar con el core ESP32 3.x y calibrar NTC (Steinhart-Hart, B=3950), MQ-2 (raw↔%LEL) y ruido de ΔT en hardware.
3. `wokwi_diagram.json` es de Arduino Uno y no corresponde a este firmware.
