# 🔧 ESP32 — Firmware de Seguridad para Cocinas Industriales

<div align="center">

![Plataforma](https://img.shields.io/badge/plataforma-Arduino%20Uno%20%2F%20ESP32-00979D?style=flat-square)
![Lenguaje](https://img.shields.io/badge/lenguaje-C%2B%2B%20Arduino-blue?style=flat-square)
![Simulación](https://img.shields.io/badge/simulación-Wokwi-orange?style=flat-square)
![Estado](https://img.shields.io/badge/estado-funcional-brightgreen?style=flat-square)

Firmware del nodo físico del sistema IoT de monitoreo de seguridad.<br>
Parte del proyecto de grado — Universidad Industrial de Santander (UIS).

[Organización](https://github.com/cocinas-industriales-uis) ·
[Servidor](https://github.com/cocinas-industriales-uis/servidor) ·
[Página web](https://github.com/cocinas-industriales-uis/pagina-web)

</div>

---

## ¿Qué hace este firmware?

Lee tres sensores en tiempo real, aplica una lógica de prioridades de
seguridad y actúa sobre tres ventiladores, una válvula, aspersores y
un buzzer de forma autónoma. Además muestra el estado del sistema en
un LCD 16×2 y envía los datos al servidor Django cada 5 segundos.

---

## Hardware requerido

| Componente | Cantidad | Pin(es) |
|---|:---:|---|
| Arduino Uno | 1 | — |
| Sensor NTC temperatura | 1 | A0 |
| Sensor gas GLP (MQ-6) | 1 | A1 |
| Sensor presión | 1 | A2 |
| Sensor PIR movimiento | 1 | A3 |
| Driver L293D | 2 | — |
| Motor DC (ventiladores) | 3 | PWM: 3, 5, 6 |
| LED rojo — válvula | 1 | 4 |
| LED azul — aspersor | 1 | 2 |
| Buzzer | 1 | 7 |
| LCD 16×2 I2C | 1 | A4 (SDA), A5 (SCL) |
| Resistencias 220Ω | 2 | — |

> El diagrama completo del circuito está en `wokwi_diagram.json`.
> Ábrelo en [wokwi.com](https://wokwi.com) para simular sin hardware físico.

---

## Lógica de prioridades

El sistema toma decisiones en cascada. Una prioridad superior bloquea
completamente a las inferiores.

```
P1 — Gas ≥ 73%          EMERGENCIA TOTAL
     Vent1=100% Vent2=100% Vent3=100%
     Válvula=CERRADA  Buzzer=rápido (150ms)
     → Explosión de gas: riesgo más inmediato e irreversible

P2b — Temp > 90°C       INCENDIO SEVERO
     Vent1=OFF Vent2=OFF Vent3=OFF
     Aspersor=ON  Válvula=CERRADA  Buzzer=rápido
     → Flujo de aire propaga llamas; se corta todo

P2a — Temp 70–90°C      INCENDIO MODERADO
     Vent1=OFF  Vent2=100%  Vent3=OFF
     Aspersor=ON  Válvula=CERRADA  Buzzer=lento (250ms)
     → Se corta inyección, solo extracción para evacuar humo

P3 — Presión ≥ 68%      PRESIÓN ALTA
     Vent2 += refuerzo proporcional
     → Presión alta es peligrosa para llamas y fugas de gas

P4 — Temp 20–60°C       OPERACIÓN NORMAL
     Vent1 = PWM proporcional (80–255)
     Vent2 = Vent1 + ajuste de presión
     → Control suave sin alarma activa
```

---

## Mapa de pines

```
A0  → NTC temperatura        (0 raw = -55°C, 1023 raw = 155°C)
A1  → Sensor gas GLP
A2  → Sensor presión
A3  → PIR movimiento

 2  → LED aspersor (azul)
 3  → Vent1 inyección    (PWM)
 4  → LED válvula (rojo)
 5  → Vent2 extracción   (PWM)
 6  → Vent3 emergencia   (PWM)
 7  → Buzzer

A4  → LCD SDA (I2C)
A5  → LCD SCL (I2C)

Pines 8–13 reservados para drivers A4988 de steppers
```

---

## Umbrales configurables

Todos los umbrales están definidos como `#define` al inicio del archivo
`sceht.ino` para facilitar su ajuste:

```cpp
#define TEMP_INICIO_VENT   20.0   // °C — arranca ventilación
#define TEMP_MAX_VENT      60.0   // °C — velocidad máxima
#define TEMP_INCENDIO      70.0   // °C — activa aspersores

#define GAS_ALERTA         500    // raw (0–1023) — acelera vents
#define GAS_PELIGRO        750    // raw — emergencia total (~73%)

#define PRES_ALTA          700    // raw — refuerza extracción (~68%)
```

---

## LCD — mensajes en pantalla

El display alterna entre dos vistas cada 2 segundos en operación normal,
y cambia a mensajes de emergencia cuando se activa una prioridad:

| Estado | Línea 1 | Línea 2 |
|---|---|---|
| Normal (vista 1) | `T:25C G:10%` | `V1:45% V2:45%` |
| Normal (vista 2) | `Pres:30%` | `V3:0%` |
| Gas alerta | `Pres:30%` | `V3:0% G-ALRT` |
| Incendio moderado | `!!INCENDIO!!` | `T:75C V2:100%` |
| Incendio severo | `!!INCENDIO SEV!!` | `T:95C ASP:ON` |
| Fuga de gas | `!!FUGA DE GAS!!` | `G:80% VLV:OFF` |

---

## Simulación en Wokwi

No necesitas hardware físico para probar el firmware.

1. Ve a [wokwi.com](https://wokwi.com) y crea un nuevo proyecto Arduino
2. Copia el contenido de `sceht.ino` en el editor
3. Importa `wokwi_diagram.json` desde el menú de diagrama
4. Presiona ▶ para simular

Mueve los potenciómetros de Gas y Presión, y el sensor NTC de temperatura
para ver cómo reacciona el sistema en cada escenario.

---

## Nota sobre la conversión de temperatura en Wokwi

El sensor `wokwi-ntc-temperature-sensor` en Wokwi no simula la resistencia
real del NTC — entrega un valor analógico ya mapeado. Por eso el firmware
usa mapeo lineal en lugar de la fórmula Steinhart-Hart:

```cpp
// Wokwi NTC: 0 raw = -55°C, 1023 raw = 155°C
float leerTemperatura() {
  int raw = analogRead(PIN_TEMP);
  return map(raw, 0, 1023, -55, 155);
}
```

En el prototipo físico real con ESP32 y sensor NTC real, esta función
deberá reemplazarse por la fórmula Steinhart-Hart con los coeficientes
del sensor específico.

---

## Archivos

| Archivo | Descripción |
|---|---|
| `sceht.ino` | Firmware principal — lógica completa del sistema |
| `wokwi_diagram.json` | Diagrama del circuito para simulación en Wokwi |

---

## Parte del sistema completo

```
[ESP32 / Arduino Uno]  →  HTTP POST cada 5s  →  [Django Backend]
        ↑                                               ↓
  sensores físicos                            [React Dashboard]
  + actuadores                                [App Android]
```

**Repositorios relacionados:**
- [servidor](https://github.com/cocinas-industriales-uis/servidor) — Django REST API
- [pagina-web](https://github.com/cocinas-industriales-uis/pagina-web) — Dashboard React
- [app-movil](https://github.com/cocinas-industriales-uis/app-movil) — App Android
- [mqtt](https://github.com/cocinas-industriales-uis/mqtt) — Broker MQTT (pendiente)

---

<div align="center">
  <sub>Proyecto de grado · UIS · 2025–2026 · Cesar Daniel Ávila Barbosa</sub>
</div>
