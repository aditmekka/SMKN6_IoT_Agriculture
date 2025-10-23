@extends('layouts.app')

@section('content')
<div 
    x-data="{ open: false, systemOpen: false }" 
    class="text-center relative overflow-hidden transition-opacity duration-500 ease-in-out page-enter" 
    x-cloak
>

    <h1 class="text-3xl font-bold text-blue-700 mb-10 flex justify-center items-center gap-2">
        Settings
    </h1>

    <!-- Layer blur -->
    <div x-cloak x-show="open || systemOpen"
         x-transition:enter="transition-opacity duration-300 ease-out"
         x-transition:enter-start="opacity-0"
         x-transition:enter-end="opacity-100"
         x-transition:leave="transition-opacity duration-300 ease-in"
         x-transition:leave-start="opacity-100"
         x-transition:leave-end="opacity-0"
         class="absolute inset-0 bg-white/30 backdrop-blur-sm z-0 rounded-2xl">
    </div>

    <!-- Grid utama -->
    <div class="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-8 justify-center transition-all duration-500 relative z-20"
         :class="(open || systemOpen) ? 'scale-90 blur-sm opacity-40 pointer-events-none' : 'scale-100 blur-0 opacity-100'">

        <!-- Profil Pengguna -->
        <div class="bg-white/80 rounded-2xl shadow-lg p-6">
            <h3 class="text-lg font-semibold">Profil Pengguna</h3>
            <p class="text-gray-600 text-sm mb-4">Kelola nama dan informasi akun</p>
            <button class="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg transition">Edit</button>
        </div>

        <!-- Keamanan -->
        <div class="bg-white/80 rounded-2xl shadow-lg p-6">
            <h3 class="text-lg font-semibold">Keamanan</h3>
            <p class="text-gray-600 text-sm mb-4">Ubah kata sandi akun Anda</p>
            <a 
                href="{{ route('profile.edit') }}"
                @click.prevent="
                    document.body.classList.add('page-leave');
                    setTimeout(() => window.location.href='{{ route('profile.edit') }}', 450);
                "
                class="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg transition"
            >
                Update
            </a>
        </div>

        <!-- Tema Tampilan -->
        <div class="bg-white/80 rounded-2xl shadow-lg p-6">
            <h3 class="text-lg font-semibold">Tema Tampilan</h3>
            <p class="text-gray-600 text-sm mb-4">Sesuaikan mode terang atau gelap</p>
            <button class="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg transition">Sesuaikan</button>
        </div>

        <!-- Bahasa -->
        <div class="relative">
            <div x-show="!open"
                 x-transition:enter="transition transform duration-400 ease-out"
                 x-transition:enter-start="scale-90 opacity-0"
                 x-transition:enter-end="scale-100 opacity-100"
                 x-transition:leave="transition transform duration-400 ease-in"
                 x-transition:leave-start="scale-100 opacity-100"
                 x-transition:leave-end="scale-90 opacity-0"
                 class="bg-white/80 rounded-2xl shadow-lg p-6 relative z-10">
                <h3 class="text-lg font-semibold">Bahasa</h3>
                <p class="text-gray-600 text-sm mb-4">Atur bahasa antarmuka</p>
                <button @click="open = true"
                        class="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg transition">
                    Pilih
                </button>
            </div>
        </div>

        <!-- Notifikasi -->
        <div class="bg-white/80 rounded-2xl shadow-lg p-6">
            <h3 class="text-lg font-semibold">Notifikasi</h3>
            <p class="text-gray-600 text-sm mb-4">Kelola pemberitahuan sistem</p>
            <button class="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg transition">Kelola</button>
        </div>

        <!-- Tentang Sistem -->
        <div class="relative">
            <div x-show="!systemOpen"
                 x-transition:enter="transition transform duration-400 ease-out"
                 x-transition:enter-start="scale-90 opacity-0"
                 x-transition:enter-end="scale-100 opacity-100"
                 x-transition:leave="transition transform duration-400 ease-in"
                 x-transition:leave-start="scale-100 opacity-100"
                 x-transition:leave-end="scale-90 opacity-0"
                 class="bg-white/80 rounded-2xl shadow-lg p-6 relative z-10">
                <h3 class="text-lg font-semibold">Tentang Sistem</h3>
                <p class="text-gray-600 text-sm mb-4">Lihat versi & pengembang</p>
                <button @click="systemOpen = true"
                        class="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg transition">
                    Detail
                </button>
            </div>
        </div>
    </div>
</div>

<script src="https://cdn.jsdelivr.net/npm/alpinejs@3.x.x/dist/cdn.min.js" defer></script>
@endsection
