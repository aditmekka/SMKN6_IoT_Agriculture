AQUA (Automatic Quality Utility for Agriculture)

AQUA adalah sistem penyiram tanaman otomatis berbasis mikrokontroler ESP32 yang bekerja dengan sensor AHT20 untuk memantau kondisi lingkungan seperti kelembapan udara dan suhu.
Sistem ini dirancang agar tanaman mendapatkan penyiraman air secara otomatis dan efisien berdasarkan data sensor yang terbaca secara real-time.

Ketika kelembapan udara terdeteksi rendah (menandakan kondisi kering), ESP32 akan memproses data tersebut dan mengaktifkan mekanisme penyiraman otomatis melalui sistem mekanik berupa relay dan pompa air mini. Setelah kelembapan kembali ke tingkat ideal, sistem akan otomatis **mematikan pompa, sehingga penyiraman tidak berlebihan dan penggunaan air menjadi lebih hemat.