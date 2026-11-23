# Autonomous Terrain-Aware Seed Dispersal Payload for UAVs

ELEC4912 Graduation Project – Işık University  
Ömer Yusuf Tuvaş · Supervisor: Assoc. Prof. Dr. Ramazan Köprü

An ESP32-CAM classifies the ground under the drone (soil / grass-vegetation / asphalt-concrete)
with rule-based HSV thresholds and opens a servo-driven slide gate to release barley only over soil.

## Contents

| Folder | Runs on | Description |
|---|---|---|
| `1_esik_belirleme_python/` | PC, Python 3 | Threshold selection and offline test on 15 photos (`veri/`) |
| `2_esp32cam/zemin_otomatik_tohum/` | ESP32-CAM (Arduino IDE, AI Thinker) | Final firmware: classification and automatic release |
| `3_test_kodlari/arpa_dozaj_testi/` | Arduino Nano | Barley dosage test: opens the gate for a time chosen over the serial port (200–1500 ms) |

## Threshold selection (PC)


## Final firmware

`2_esp32cam/zemin_otomatik_tohum`: classification task (~6–7 decisions/s), release after two
consecutive soil decisions, gate open 600–1500 ms according to the soil ratio, gate kept closed on
camera fault. Set `OPTOKUPLOR_VAR = true` when the PC817 stage is fitted.  
Wi-Fi AP `ZeminTest` → http://192.168.4.1 (live view, manual gate buttons, latency statistics).
