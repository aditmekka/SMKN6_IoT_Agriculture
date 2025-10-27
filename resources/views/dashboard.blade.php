@extends('layouts.app')

@section('content')
<div 
    x-data="{ showCards: false }" 
    x-init="setTimeout(() => showCards = true, 300)"
    class="p-8 text-gray-800 text-center overflow-hidden"
    x-cloak
>
    <!-- Header Animasi Ngetik -->
    <h1 class="text-3xl font-bold text-blue-800 mb-10">
        <span class="typewriter">Welcome, {{ Auth::user()->name }}.</span>
    </h1>

    <!-- Grid Dashboard -->
    <div 
        class="grid grid-cols-1 md:grid-cols-3 gap-8 justify-items-center"
        x-show="showCards"
    >
        <!-- Card 1 -->
        <div class="glass-card fade-in delay-1 w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-temperature-high text-3xl text-red-500 mb-2"></i>
            <h2 class="font-semibold">Suhu</h2>
            <p class="text-2xl font-bold">28°C</p>
            <p class="text-sm text-gray-600">Normal</p>
        </div>

        <!-- Card 2 -->
        <div class="glass-card fade-in delay-2 w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-droplet text-3xl text-blue-500 mb-2"></i>
            <h2 class="font-semibold">Kelembapan</h2>
            <p class="text-2xl font-bold">78%</p>
            <p class="text-sm text-gray-600">Optimal</p>
        </div>

        <!-- Card 3 -->
        <div class="glass-card fade-in delay-3 w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-cloud text-3xl text-gray-500 mb-2"></i>
            <h2 class="font-semibold">Emisi Karbon</h2>
            <p class="text-2xl font-bold">412 ppm</p>
            <p class="text-sm text-gray-600">Aman</p>
        </div>

        <!-- Card 4 -->
        <div class="glass-card fade-in delay-4 w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-seedling text-3xl text-green-600 mb-2"></i>
            <h2 class="font-semibold">Kelembapan Tanah</h2>
            <p class="text-2xl font-bold">65%</p>
            <p class="text-sm text-gray-600">Stabil</p>
        </div>

        <!-- Card 5 -->
        <div class="glass-card fade-in delay-5 w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-cloud-rain text-3xl text-indigo-500 mb-2"></i>
            <h2 class="font-semibold">Status Hujan</h2>
            <p class="text-2xl font-bold">Tidak Hujan</p>
            <p class="text-sm text-gray-600">Langit Cerah</p>
        </div>

        <!-- Card 6 -->
        <div class="glass-card fade-in delay-6 w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-water text-3xl text-blue-600 mb-2"></i>
            <h2 class="font-semibold">Status Pompa</h2>
            <p class="text-2xl font-bold text-green-600">Aktif</p>
            <p class="text-sm text-gray-600">Beroperasi</p>
        </div>
    </div>
</div>

<!-- Alpine.js -->
<script src="https://cdn.jsdelivr.net/npm/alpinejs@3.x.x/dist/cdn.min.js" defer></script>

<!-- FontAwesome -->
<script src="https://kit.fontawesome.com/a2d9e6d9bb.js" crossorigin="anonymous"></script>

<style>
[x-cloak] { display: none !important; }

/* ✨ Efek kaca */
.glass-card {
    background: rgba(255, 255, 255, 0.7);
    backdrop-filter: blur(10px);
    border-radius: 1rem;
    box-shadow: 0 8px 20px rgba(0, 0, 0, 0.15);
    transition: all 0.3s ease;
}
.glass-card:hover {
    transform: scale(1.05);
    box-shadow: 0 10px 25px rgba(0, 0, 0, 0.2);
}

/* 🎹 Efek mesin ketik fix (garis biru berhenti tepat di akhir teks) */
.typewriter {
  overflow: hidden;
  white-space: nowrap;
  border-right: 3px solid #2563eb;
  width: 0;
  display: inline-block;
  animation: typing 1.8s steps(30, end) forwards, blink 0.7s step-end infinite alternate;
}

@keyframes typing {
  0% { width: 0; }
  95% { border-right: 3px solid #2563eb; }
  100% { width: calc(100% - 0.35ch); border-right: none; }
}

@keyframes blink {
  from { border-color: transparent; }
  to { border-color: #2563eb; }
}

/* 🌫️ Efek Fade-in halus */
.fade-in {
    opacity: 0;
    transform: translateY(10px);
    animation: fadeIn 0.8s ease-out forwards;
}
@keyframes fadeIn {
    to {
        opacity: 1;
        transform: translateY(0);
    }
}

/* Delay muncul per kartu */
.delay-1 { animation-delay: 0.3s; }
.delay-2 { animation-delay: 0.45s; }
.delay-3 { animation-delay: 0.6s; }
.delay-4 { animation-delay: 0.75s; }
.delay-5 { animation-delay: 0.9s; }
.delay-6 { animation-delay: 1.05s; }
</style>
@endsection
