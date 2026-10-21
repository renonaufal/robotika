# Modul 1 — Basic Arduino / ESP32

## Tujuan
Menyusun program dasar untuk mengendalikan LED indikator dan perintah motor DC menggunakan input serial.

## Berkas
- `src/basic_arduino.ino` — program utama.
- `laporan/laporan_modul_1.md` — draft laporan riset.

## Asumsi pin
GPIO 2 untuk LED, GPIO 27 dan 26 untuk input driver motor, serta GPIO 12 untuk PWM/enable. Susunan ini mengikuti contoh repository referensi dan harus dicocokkan dengan perangkat kelas.

## Langkah konfigurasi
1. Buka `basic_arduino.ino` di Arduino IDE.
2. Pilih board ESP32 yang sesuai dan port yang benar jika perangkat tersedia.
3. Pastikan ESP32 Arduino Core 3.x digunakan karena program memakai `ledcAttach(pin, frequency, resolution)`.
4. Upload firmware bila perangkat tersedia.
5. Buka Serial Monitor pada 115200 baud dan pilih newline sesuai kebutuhan.
6. Kirim `1` untuk memberi perintah ON dan `0` untuk OFF.

## Interpretasi
Pesan serial menyatakan perintah diterima, bukan bukti bahwa motor fisik benar-benar berputar. Kondisi motor aktual dipengaruhi driver, catu daya, sambungan, dan perangkat keras.
