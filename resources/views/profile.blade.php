<x-app-layout>
    <x-slot name="header">
        <h2 class="font-semibold text-xl text-gray-800 dark:text-gray-200 leading-tight">
            {{ __('Profile') }}
        </h2>
    </x-slot>

    <div 
        x-data="{ show: false }"
        x-init="setTimeout(() => show = true, 50)"
        x-show="show"
        x-transition:enter="transition-all transform duration-700 ease-out"
        x-transition:enter-start="opacity-0 translate-y-6 blur-[6px] scale-95"
        x-transition:enter-end="opacity-100 translate-y-0 blur-0 scale-100"
        x-transition:leave="transition-all transform duration-500 ease-in"
        x-transition:leave-start="opacity-100 translate-y-0 blur-0 scale-100"
        x-transition:leave-end="opacity-0 translate-y-5 blur-[6px] scale-95"
        class="py-12 relative overflow-hidden"
    >

        <!-- Background gradient lembut -->
        <div class="absolute inset-0 bg-gradient-to-br from-blue-50 via-white to-blue-100 dark:from-gray-900 dark:via-gray-800 dark:to-gray-900 opacity-60 blur-3xl"></div>

        <div class="max-w-7xl mx-auto sm:px-6 lg:px-8 space-y-6 relative z-10 animate-float">

            <!-- Ubah Profil -->
            <div class="p-4 sm:p-8 bg-white/80 dark:bg-gray-800/80 shadow-xl backdrop-blur-md sm:rounded-2xl transition-all duration-500 hover:scale-[1.01]">
                <div class="max-w-xl">
                    <livewire:profile.update-profile-information-form />
                </div>
            </div>

            <!-- Ubah Kata Sandi -->
            <div class="p-4 sm:p-8 bg-white/80 dark:bg-gray-800/80 shadow-xl backdrop-blur-md sm:rounded-2xl transition-all duration-500 hover:scale-[1.01]">
                <div class="max-w-xl">
                    <livewire:profile.update-password-form />
                </div>
            </div>

            <!-- Hapus Akun -->
            <div class="p-4 sm:p-8 bg-white/80 dark:bg-gray-800/80 shadow-xl backdrop-blur-md sm:rounded-2xl transition-all duration-500 hover:scale-[1.01]">
                <div class="max-w-xl">
                    <livewire:profile.delete-user-form />
                </div>
            </div>

            <!-- Tombol Kembali -->
            <div class="flex justify-center pt-6">
                <a 
                    href="{{ route('settings') }}"
                    @click.prevent="
                        show = false;
                        setTimeout(() => window.location.href='{{ route('settings') }}', 500);
                    "
                    class="bg-gray-700 hover:bg-gray-800 text-white px-6 py-2 rounded-xl transition-all duration-300 shadow-md hover:shadow-lg hover:scale-105"
                >
                    ← Kembali ke Settings
                </a>
            </div>
        </div>
    </div>

    <style>
        /* Efek mengambang */
        @keyframes float {
            0%, 100% { transform: translateY(0); }
            50% { transform: translateY(-8px); }
        }

        .animate-float {
            animation: float 4s ease-in-out infinite;
        }

        /* Mencegah flicker */
        [x-cloak] { display: none !important; }

        /* Optimasi rendering smooth */
        [x-show="show"] {
            will-change: transform, opacity, filter;
            transform: translateZ(0);
            backface-visibility: hidden;
        }
    </style>

    <script src="https://cdn.jsdelivr.net/npm/alpinejs@3.x.x/dist/cdn.min.js" defer></script>
</x-app-layout>
