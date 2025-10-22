<?php

namespace App\Http\Controllers;

use Illuminate\Http\Request;

class SensorDataController extends Controller
{
    public function index()
    {
        // Dummy data buat preview tampilan
        $sensorData = collect([
            (object)[
                'created_at' => now(),
                'suhu' => 28,
                'kelembapan' => 78,
                'emisi_karbon' => 412,
                'kelembapan_tanah' => 65,
                'status_hujan' => 'Tidak Hujan',
                'status_pompa' => 'Aktif',
            ],
            (object)[
                'created_at' => now()->subMinutes(10),
                'suhu' => 29,
                'kelembapan' => 80,
                'emisi_karbon' => 420,
                'kelembapan_tanah' => 63,
                'status_hujan' => 'Hujan',
                'status_pompa' => 'Nonaktif',
            ],
            (object)[
                'created_at' => now()->subMinutes(20),
                'suhu' => 27,
                'kelembapan' => 77,
                'emisi_karbon' => 415,
                'kelembapan_tanah' => 66,
                'status_hujan' => 'Tidak Hujan',
                'status_pompa' => 'Aktif',
            ],
        ]);

        return view('dataSensor', compact('sensorData'));
    }
}
