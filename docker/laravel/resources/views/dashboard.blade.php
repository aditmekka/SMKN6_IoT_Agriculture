@extends('layouts.app')

@section('content')
<div class="p-8 text-gray-800 text-center">
    <!-- Header -->
    <h1 class="text-3xl font-bold text-blue-800 mb-10">
        Welcome, {{ Auth::user()->name }}.
    </h1>

    <!-- Grid Dashboard -->
    <div class="grid grid-cols-1 md:grid-cols-3 gap-8 justify-items-center">
        <!-- Card 1 -->
        <div class="glass-card w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-temperature-high text-3xl text-red-500 mb-2"></i>
            <h2 class="font-semibold">Suhu</h2>
            <p class="text-2xl font-bold">28°C</p>
            <p class="text-sm text-gray-600">Normal</p>
        </div>

        <!-- Card 2 -->
        <div class="glass-card w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-droplet text-3xl text-blue-500 mb-2"></i>
            <h2 class="font-semibold">Kelembapan</h2>
            <p class="text-2xl font-bold">78%</p>
            <p class="text-sm text-gray-600">Optimal</p>
        </div>

        <!-- Card 3 -->
        <div class="glass-card w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-cloud text-3xl text-gray-500 mb-2"></i>
            <h2 class="font-semibold">Emisi Karbon</h2>
            <p class="text-2xl font-bold">412 ppm</p>
            <p class="text-sm text-gray-600">Aman</p>
        </div>

        <!-- Card 4 -->
        <div class="glass-card w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-seedling text-3xl text-green-600 mb-2"></i>
            <h2 class="font-semibold">Kelembapan Tanah</h2>
            <p class="text-2xl font-bold">65%</p>
            <p class="text-sm text-gray-600">Stabil</p>
        </div>

        <!-- Card 5 -->
        <div class="glass-card w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-cloud-rain text-3xl text-indigo-500 mb-2"></i>
            <h2 class="font-semibold">Status Hujan</h2>
            <p class="text-2xl font-bold">Tidak Hujan</p>
            <p class="text-sm text-gray-600">Langit Cerah</p>
        </div>

        <!-- Card 6 -->
        <div class="glass-card w-64 h-40 flex flex-col justify-center items-center">
            <i class="fa-solid fa-water text-3xl text-blue-600 mb-2"></i>
            <h2 class="font-semibold">Status Pompa</h2>
            <p class="text-2xl font-bold text-green-600">Aktif</p>
            <p class="text-sm text-gray-600">Beroperasi</p>
        </div>
    </div>
</div>

<style>
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
</style>

<!-- FontAwesome -->
<script src="https://kit.fontawesome.com/a2d9e6d9bb.js" crossorigin="anonymous"></script>
@endsection
