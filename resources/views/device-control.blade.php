@extends('layouts.app')

@section('content')
<div class="py-6 px-8 min-h-[85vh] flex flex-col items-center">
    <div class="w-full max-w-6xl bg-white/1 backdrop-blur-lg rounded-[1.5rem] shadow-[0_1px_5px_rgba(0,0,0,0.1)] p-2 border border-white/10">
        <h1 class="text-3xl font-semibold text-[#004aad] mb-8 text-center drop-shadow-sm">Device Control</h1>

        <div class="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-6">
            <!-- Pompa Air -->
            <div class="bg-white/60 rounded-2xl shadow-sm p-5 text-center transition transform hover:scale-[1.02] hover:shadow-md backdrop-blur-md border border-white/20">
                <h2 class="text-lg font-semibold mb-2 text-gray-800">Pompa Air</h2>
                <p class="text-gray-600 mb-3">Status: <span class="font-semibold text-green-600">Aktif</span></p>
                <button class="bg-red-500 hover:bg-red-600 text-white px-5 py-2 rounded-xl transition duration-300">
                    Matikan
                </button>
            </div>

            <!-- Kipas Pendingin -->
            <div class="bg-white/60 rounded-2xl shadow-sm p-5 text-center transition transform hover:scale-[1.02] hover:shadow-md backdrop-blur-md border border-white/20">
                <h2 class="text-lg font-semibold mb-2 text-gray-800">Kipas Pendingin</h2>
                <p class="text-gray-600 mb-3">Status: <span class="font-semibold text-red-600">Nonaktif</span></p>
                <button class="bg-green-500 hover:bg-green-600 text-white px-5 py-2 rounded-xl transition duration-300">
                    Nyalakan
                </button>
            </div>

            <!-- Lampu Greenhouse -->
            <div class="bg-white/60 rounded-2xl shadow-sm p-5 text-center transition transform hover:scale-[1.02] hover:shadow-md backdrop-blur-md border border-white/20">
                <h2 class="text-lg font-semibold mb-2 text-gray-800">Lampu Greenhouse</h2>
                <p class="text-gray-600 mb-3">Status: <span class="font-semibold text-green-600">Aktif</span></p>
                <button class="bg-red-500 hover:bg-red-600 text-white px-5 py-2 rounded-xl transition duration-300">
                    Matikan
                </button>
            </div>

            <!-- Sensor Tanah -->
            <div class="bg-white/60 rounded-2xl shadow-sm p-5 text-center transition transform hover:scale-[1.02] hover:shadow-md backdrop-blur-md border border-white/20">
                <h2 class="text-lg font-semibold mb-2 text-gray-800">Sensor Kelembapan Tanah</h2>
                <p class="text-gray-600 mb-3">Status: <span class="font-semibold text-green-600">Aktif</span></p>
                <button class="bg-red-500 hover:bg-red-600 text-white px-5 py-2 rounded-xl transition duration-300">
                    Matikan
                </button>
            </div>

            <!-- Sensor Hujan -->
            <div class="bg-white/60 rounded-2xl shadow-sm p-5 text-center transition transform hover:scale-[1.02] hover:shadow-md backdrop-blur-md border border-white/20">
                <h2 class="text-lg font-semibold mb-2 text-gray-800">Sensor Hujan</h2>
                <p class="text-gray-600 mb-3">Status: <span class="font-semibold text-green-600">Aktif</span></p>
                <button class="bg-red-500 hover:bg-red-600 text-white px-5 py-2 rounded-xl transition duration-300">
                    Matikan
                </button>
            </div>

            <!-- Emisi Karbon -->
            <div class="bg-white/60 rounded-2xl shadow-sm p-5 text-center transition transform hover:scale-[1.02] hover:shadow-md backdrop-blur-md border border-white/20">
                <h2 class="text-lg font-semibold mb-2 text-gray-800">Sensor Emisi Karbon</h2>
                <p class="text-gray-600 mb-3">Status: <span class="font-semibold text-red-600">Nonaktif</span></p>
                <button class="bg-green-500 hover:bg-green-600 text-white px-5 py-2 rounded-xl transition duration-300">
                    Nyalakan
                </button>
            </div>
        </div>
    </div>
</div>
@endsection
