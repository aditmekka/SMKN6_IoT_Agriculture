@extends('layouts.app')

@section('content')
<div 
    x-data="{ show: false }" 
    x-init="setTimeout(() => show = true, 150)"
    class="py-6 px-8 min-h-[85vh] flex flex-col items-center"
>
    <div 
        class="w-full max-w-6xl bg-white/10 backdrop-blur-lg rounded-[1.5rem] shadow-[0_1px_5px_rgba(0,0,0,0.1)] p-8 border border-white/10 transition-all duration-700 ease-out"
        :class="show ? 'opacity-100 translate-y-0 scale-100' : 'opacity-0 translate-y-3 scale-[0.98]'"
    >
        <h1 class="text-3xl font-semibold text-[#004aad] mb-8 text-center drop-shadow-sm">Device Control</h1>

        <div class="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-6">
            @php
                $devices = [
                    ['name' => 'Pompa Air', 'status' => 'Aktif', 'color' => 'green', 'button' => 'Matikan', 'buttonColor' => 'red'],
                    ['name' => 'Kipas Pendingin', 'status' => 'Nonaktif', 'color' => 'red', 'button' => 'Nyalakan', 'buttonColor' => 'green'],
                    ['name' => 'Lampu Greenhouse', 'status' => 'Aktif', 'color' => 'green', 'button' => 'Matikan', 'buttonColor' => 'red'],
                    ['name' => 'Sensor Kelembapan Tanah', 'status' => 'Aktif', 'color' => 'green', 'button' => 'Matikan', 'buttonColor' => 'red'],
                    ['name' => 'Sensor Hujan', 'status' => 'Aktif', 'color' => 'green', 'button' => 'Matikan', 'buttonColor' => 'red'],
                    ['name' => 'Sensor Emisi Karbon', 'status' => 'Nonaktif', 'color' => 'red', 'button' => 'Nyalakan', 'buttonColor' => 'green'],
                ];
            @endphp

            @foreach($devices as $index => $device)
            <div 
                x-data="{ visible: false }"
                x-init="setTimeout(() => visible = true, {{ $index * 120 + 200 }})"
                class="bg-white/60 rounded-2xl shadow-sm p-5 text-center backdrop-blur-md border border-white/20 transform transition-all duration-700 ease-out"
                :class="visible ? 'opacity-100 translate-y-0 scale-100' : 'opacity-80 translate-y-3 scale-[0.98]'"
            >
                <h2 class="text-lg font-semibold mb-2 text-gray-800">{{ $device['name'] }}</h2>
                <p class="text-gray-600 mb-3">Status: 
                    <span class="font-semibold text-{{ $device['color'] }}-600">{{ $device['status'] }}</span>
                </p>
                <button class="bg-{{ $device['buttonColor'] }}-500 hover:bg-{{ $device['buttonColor'] }}-600 text-white px-5 py-2 rounded-xl transition duration-300">
                    {{ $device['button'] }}
                </button>
            </div>
            @endforeach
        </div>
    </div>
</div>
@endsection
