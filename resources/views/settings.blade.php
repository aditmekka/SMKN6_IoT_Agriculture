@extends('layouts.app')

@section('content')
<div 
    x-data="{ open: false, systemOpen: false }" 
    class="text-center relative overflow-hidden transition-opacity duration-500 ease-in-out" 
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
            <a href="{{ route('profile.edit') }}"
                 class="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg transition">
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

    <!-- Card Bahasa -->
    <div x-cloak 
         x-show="open"
         x-transition:enter="transition-all transform duration-500 ease-out delay-[20ms]"
         x-transition:enter-start="opacity-0 scale-95 translate-y-5 blur-[2px]"
         x-transition:enter-end="opacity-100 scale-100 translate-y-0 blur-0"
         x-transition:leave="transition-all transform duration-400 ease-in"
         x-transition:leave-start="opacity-100 scale-100 translate-y-0 blur-0"
         x-transition:leave-end="opacity-0 scale-95 translate-y-5 blur-[2px]"
         class="absolute inset-0 flex items-center justify-center z-50">

        <div class="bg-white/95 rounded-3xl p-8 shadow-2xl w-full max-w-md transform transition-all animate-[float_3s_ease-in-out_infinite]">
            <h3 class="text-2xl font-semibold mb-6 text-gray-800">Pilih Bahasa</h3>

            <div class="grid grid-cols-3 gap-4 mb-8">
                <button class="bg-blue-400 hover:bg-blue-500 text-white py-2 rounded-lg transition shadow-md hover:shadow-lg transform hover:scale-105 flex justify-center items-center gap-2">
                    <span class="fi fi-id"></span> Indonesia
                </button>
                <button class="bg-blue-400 hover:bg-blue-500 text-white py-2 rounded-lg transition shadow-md hover:shadow-lg transform hover:scale-105 flex justify-center items-center gap-2">
                    <span class="fi fi-gb"></span> English
                </button>
                <button class="bg-blue-400 hover:bg-blue-500 text-white py-2 rounded-lg transition shadow-md hover:shadow-lg transform hover:scale-105 flex justify-center items-center gap-2 font-[Cairo]">
                    <span class="fi fi-sa"></span> العربية
                </button>
                <button class="bg-blue-400 hover:bg-blue-500 text-white py-2 rounded-lg transition shadow-md hover:shadow-lg transform hover:scale-105 flex justify-center items-center gap-2">
                    <span class="fi fi-fr"></span> Français
                </button>
                <button class="bg-blue-400 hover:bg-blue-500 text-white py-2 rounded-lg transition shadow-md hover:shadow-lg transform hover:scale-105 flex justify-center items-center gap-2">
                    <span class="fi fi-es"></span> Español
                </button>
                <button class="bg-blue-400 hover:bg-blue-500 text-white py-2 rounded-lg transition shadow-md hover:shadow-lg transform hover:scale-105 flex justify-center items-center gap-2">
                    <span class="fi fi-pt"></span> Português
                </button>
                <button class="bg-blue-400 hover:bg-blue-500 text-white py-2 rounded-lg transition shadow-md hover:shadow-lg transform hover:scale-105 flex justify-center items-center gap-2">
                    <span class="fi fi-cn"></span> 中文
                </button>
                <button class="bg-blue-400 hover:bg-blue-500 text-white py-2 rounded-lg transition shadow-md hover:shadow-lg transform hover:scale-105 flex justify-center items-center gap-2">
                    <span class="fi fi-jp"></span> 日本語
                </button>
                <button class="bg-blue-400 hover:bg-blue-500 text-white py-2 rounded-lg transition shadow-md hover:shadow-lg transform hover:scale-105 flex justify-center items-center gap-2">
                    <span class="fi fi-kr"></span> 한국어
                </button>
            </div>

            <button @click="open = false"
                    class="bg-gray-500 hover:bg-gray-600 text-white px-6 py-2 rounded-lg transition shadow-md hover:shadow-lg">
                Kembali
            </button>
        </div>
    </div>

    <!-- Card Tentang Sistem -->
    <div x-cloak 
        x-show="systemOpen"
        x-transition:enter="transition-all transform duration-500 ease-out delay-[20ms]"
        x-transition:enter-start="opacity-0 scale-95 translate-y-5 blur-[2px]"
        x-transition:enter-end="opacity-100 scale-100 translate-y-0 blur-0"
        x-transition:leave="transition-all transform duration-400 ease-in"
        x-transition:leave-start="opacity-100 scale-100 translate-y-0 blur-0"
        x-transition:leave-end="opacity-0 scale-95 translate-y-5 blur-[2px]"
        class="absolute inset-0 flex items-center justify-center z-50">

        <div class="bg-white/95 rounded-3xl p-10 shadow-2xl w-full max-w-xl transform transition-all animate-[float_3s_ease-in-out_infinite]">
            <h3 class="text-2xl font-semibold mb-4 text-gray-800">Tentang Sistem</h3>
            <p class="text-gray-600 mb-4">AQUA (Automatic Quality Utility for Agriculture)</p>

            <ul class="text-left text-gray-700 list-disc list-inside mb-6 leading-relaxed">
                <li><span class="font-semibold">Versi Sistem:</span> 1.0</li>
                <li><span class="font-semibold">Pengembang:</span> Universitas Pendidikan Indonesia dan SMKN 6 Bandung</li>
            </ul>

            <button @click="systemOpen = false"
                    class="bg-blue-600 hover:bg-blue-700 text-white px-6 py-2 rounded-lg transition shadow-md hover:shadow-lg">
                Tutup
            </button>
        </div>
    </div>
</div>

<!-- Alpine.js -->
<script src="https://cdn.jsdelivr.net/npm/alpinejs@3.x.x/dist/cdn.min.js" defer></script>

<!-- Efek mengambang + smooth -->
<style>
[x-cloak] { display: none !important; }

[x-show="open"], [x-show="systemOpen"] > div {
  will-change: transform, opacity;
  backface-visibility: hidden;
  transform: translateZ(0);
}

@keyframes float {
  0%, 100% { transform: translateY(0); }
  50% { transform: translateY(-8px); }
}

.fi {
  width: 20px;
  height: 15px;
  border-radius: 2px;
  box-shadow: 0 0 2px rgba(0,0,0,0.2);
}
</style>
@endsection
