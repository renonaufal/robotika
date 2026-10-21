# Modul 4 — OTA Firmware Update dengan ArduinoOTA

## Berkas
- `src/ota_arduino.ino` — firmware ESP32 untuk menyediakan layanan OTA melalui access point.
- `laporan/laporan_modul_4.md` — draft laporan riset.

## Konsep
Firmware membuat access point Wi-Fi bernama `iMCLab-OTA`. Laptop terhubung ke access point itu, lalu firmware dapat dikirim melalui network port Arduino IDE jika board package dan jaringan mendukung discovery/OTA. Contoh ini memakai `ArduinoOTA`, bukan Blynk sebagai kanal pembaruan firmware.

## Keamanan
Sebelum digunakan, ganti `AP_PASSWORD` dan `OTA_PASSWORD` dengan nilai kuat. Jangan menggunakan password contoh untuk deployment nyata. Jangan unggah password ke repository publik.

## Alur penggunaan saat perangkat tersedia
1. Pilih board ESP32 dan port serial awal di Arduino IDE.
2. Unggah firmware awal melalui USB.
3. Sambungkan laptop ke access point `iMCLab-OTA` dengan password yang telah dikonfigurasi.
4. Pastikan network port ESP32 muncul pada Arduino IDE; jika tidak, periksa jaringan dan firewall.
5. Pilih network port dan unggah firmware berikutnya.
6. Amati pesan status OTA pada Serial Monitor jika tersedia.

Catatan: firmware OTA harus memiliki dukungan partisi yang sesuai. Perubahan LED_ON hanya contoh perubahan program antara dua versi firmware, bukan hasil pengujian.
