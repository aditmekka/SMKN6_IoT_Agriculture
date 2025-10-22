@extends('layouts.app')

@section('content')
<div class="flex flex-col items-center justify-center min-h-screen bg-gradient-to-b from-blue-50 to-white text-center p-8">
    <h1 class="text-4xl font-bold text-blue-700 mb-4 animate-fade-in">
        Tentang Sistem 🌊
    </h1>
    <p class="text-gray-700 text-lg max-w-2xl mb-6">
        <strong>AQUA IoT Monitoring System v1.0</strong><br>
        Dikembangkan oleh <span class="text-blue-600 font-semibold">Naufal Nadhif</span>.<br>
        Sistem ini dirancang untuk memantau suhu, kelembapan, emisi karbon, dan status pompa secara real-time.
    </p>
    <a href="{{ route('settings') }}" 
       class="bg-blue-600 text-white px-5 py-2 rounded-lg shadow hover:bg-blue-700 transition">
        Kembali ke Settings
    </a>
</div>
@endsection
