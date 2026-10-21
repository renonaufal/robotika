# Modul 2 — Kendali ESP32 dengan Python

## Berkas
- `src/python_arduino.ino` — firmware ESP32 yang menerima perintah serial per baris.
- `src/motor_control.py` — aplikasi CLI Python untuk mengirim perintah melalui USB serial.
- `requirements.txt` — dependensi Python.
- `laporan/laporan_modul_2.md` — draft laporan riset.

## Instalasi perangkat lunak
```bash
python -m pip install -r requirements.txt
```

## Cara menjalankan Python
```bash
python src/motor_control.py --port COM3
```
Ganti `COM3` dengan port aktual. Pada Linux/macOS gunakan nama perangkat yang sesuai. Program firmware harus memakai baud rate 115200.

## Protokol perintah
- `0`: PWM 0 / berhenti.
- `1`: aktifkan duty default 180, atau pertahankan duty saat ini jika sudah aktif.
- `+` dan `-`: naik/turunkan duty sebesar 15.
- `PWM:0..255`: tetapkan duty secara langsung.
- Jika tidak menerima perintah selama 2,5 detik saat duty aktif, firmware memberi perintah berhenti sebagai mekanisme timeout.

Perintah serial dan pesan status tidak membuktikan putaran motor aktual; itu hanya menunjukkan perintah program yang diproses.
