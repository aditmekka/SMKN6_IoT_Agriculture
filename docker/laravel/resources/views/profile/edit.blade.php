@extends('layouts.app')

@section('content')
<div class="text-center relative overflow-hidden flex flex-col items-center justify-center min-h-[calc(100vh-100px)]">
    <h1 class="text-2xl font-bold text-blue-700 mb-8 flex justify-center items-center gap-2">
        Keamanan 🔒
    </h1>

    <div class="bg-white/80 backdrop-blur-md rounded-2xl shadow-lg p-8 w-full max-w-4xl flex flex-col md:flex-row gap-6 justify-center items-start transition-all duration-500 hover:shadow-2xl">

        {{-- Kiri: Ubah Password --}}
        <div class="bg-white rounded-xl shadow-md p-5 flex-1">
            @include('profile.partials.update-password-form')
        </div>

        {{-- Divider garis vertikal --}}
        <div class="hidden md:block w-px bg-gray-300"></div>

        {{-- Kanan: Hapus Akun --}}
        <div class="bg-white rounded-xl shadow-md p-5 flex-1">
            <h2 class="text-lg font-semibold text-gray-800 mb-4 flex items-center gap-2">
                <span>⚠️</span> Hapus Akun
            </h2>
            @include('profile.partials.delete-user-form')
        </div>
    </div>

    <div class="mt-8">
        <a href="{{ route('settings') }}"
           class="bg-gray-600 hover:bg-gray-700 text-white px-6 py-2 rounded-lg transition shadow-md hover:shadow-lg text-sm">
           Kembali ke Settings
        </a>
    </div>
</div>
@endsection
