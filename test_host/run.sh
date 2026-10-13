#!/bin/bash
# uso: ./run.sh [ruta .ino]   (por defecto ../cocina_esp32_1_.ino)
S=$(cd "$(dirname "$0")" && pwd); FW=${1:-$S/../cocina_esp32_1_.ino}; B=$(mktemp -d)
g++ -std=c++17 -w -I$S/mock -DFW_PATH="\"$FW\"" -x c++ $S/test.cpp -o $B/t_bin 2>$B/build.log || { echo "COMPILACION FALLA"; head -30 $B/build.log; exit 2; }
echo "compilacion OK (con stubs)"; tot=0; bad=0
for n in T01_normal25 T02_frio15 T03_gas_critico T04_gas_llama T05_llama95 T06_llama70 T07_llama25 T08_gas_prealerta T09_pres_gas T10_pres T11_llama_gas_prealerta T12_sensor_recupera T13_mqtt_caido T14_wifi_caido_arranque T15_lcd_parpadeo T16_ruido_deltaT T17_pines T18_payload_cadencia T19_ntc_averiado_con_llama T20_ntc_averiado_sin_llama; do echo "$n"; timeout 60 $B/t_bin $n; r=$?; tot=$((tot+1)); [ $r -ne 0 ] && bad=$((bad+1)); done
echo "escenarios: $tot  con fallos: $bad"
