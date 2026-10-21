# Modul 3 — Kendali ESP32 melalui Blynk IoT

## Berkas
- `src/blynk_arduino.ino` — firmware kendali motor melalui Blynk.
- `laporan/laporan_modul_3.md` — draft laporan riset.

## Library
- `Blynk` dari Library Manager Arduino IDE.
- `WiFi.h` dan `Arduino.h` disediakan oleh paket board ESP32.

## Konfigurasi wajib
Ganti placeholder berikut sebelum firmware digunakan:
- `BLYNK_TEMPLATE_ID`
- `BLYNK_TEMPLATE_NAME`
- `BLYNK_AUTH_TOKEN`
- `wifiSsid`
- `wifiPassword`

Jangan unggah token atau kata sandi pribadi ke repository publik. Jika pernah terlanjur terunggah, rotasi token/kredensial tersebut.

## Datastream / virtual pin
| Virtual pin | Tipe yang disarankan | Rentang | Fungsi |
|---|---|---:|---|
| V0 | Integer | 0–255 | Slider PWM |
| V1 | Integer | 0–1 | Switch ON/OFF |
| V2 | Integer | 0–1 | Tombol naikkan PWM, mode momentary |
| V3 | Integer | 0–1 | Tombol turunkan PWM, mode momentary |
| V4 | Integer | 0–1 | Tombol STOP, mode momentary |

Pastikan widget memakai virtual pin yang sesuai. Kendali hanya diterima saat Wi-Fi dan Blynk terhubung. Saat koneksi terputus, program memberi perintah PWM nol.
