@extends('layouts.app')

@section('content')
<div class="text-center relative overflow-hidden flex flex-col items-center justify-center min-h-[calc(100vh-120px)]">
    {{-- Judul sedikit dikecilkan --}}
    <h1 class="text-2xl font-bold text-blue-700 mb-8 flex justify-center items-center gap-2">
        Keamanan 🔒
    </h1>

    {{-- Canvas utama, transparan --}}
    <div class="bg-white/25 backdrop-blur-md rounded-2xl shadow-lg p-8 w-full max-w-5xl flex flex-col gap-6 justify-center items-center transition-all duration-500 hover:shadow-2xl">

        {{-- Dua card sejajar --}}
        <div class="flex flex-col md:flex-row gap-6 w-full justify-center">
            
            {{-- Kiri: Ubah Password --}}
            <div class="flex-1 text-left">
                <div class="bg-white/65 rounded-xl shadow p-6 border border-blue-50">
                    <h2 class="text-lg font-semibold text-blue-700 mb-3 flex items-center gap-2">
                        <span>🔑</span> Ubah Kata Sandi
                    </h2>
                    <p class="text-sm text-gray-600 mb-4">
                        Pastikan akun Anda menggunakan kata sandi yang kuat dan sulit ditebak.
                    </p>

                    <form method="post" action="{{ route('password.update') }}" class="space-y-3 text-sm">
                        @csrf
                        @method('put')

                        <div>
                            <x-input-label for="update_password_current_password" :value="__('Kata Sandi Sekarang')" />
                            <x-text-input id="update_password_current_password" name="current_password" type="password" class="mt-1 block w-full text-sm" autocomplete="current-password" />
                            <x-input-error :messages="$errors->updatePassword->get('current_password')" class="mt-1 text-xs" />
                        </div>

                        <div>
                            <x-input-label for="update_password_password" :value="__('Kata Sandi Baru')" />
                            <x-text-input id="update_password_password" name="password" type="password" class="mt-1 block w-full text-sm" autocomplete="new-password" />
                            <x-input-error :messages="$errors->updatePassword->get('password')" class="mt-1 text-xs" />
                        </div>

                        <div>
                            <x-input-label for="update_password_password_confirmation" :value="__('Konfirmasi Kata Sandi Baru')" />
                            <x-text-input id="update_password_password_confirmation" name="password_confirmation" type="password" class="mt-1 block w-full text-sm" autocomplete="new-password" />
                            <x-input-error :messages="$errors->updatePassword->get('password_confirmation')" class="mt-1 text-xs" />
                        </div>

                        <div class="pt-2">
                            <x-primary-button class="text-sm px-4 py-2 bg-blue-700 hover:bg-blue-800">
                                {{ __('Simpan Perubahan') }}
                            </x-primary-button>
                        </div>

                        @if (session('status') === 'password-updated')
                            <p x-data="{ show: true }" x-show="show" x-transition x-init="setTimeout(() => show = false, 2000)" class="text-xs text-gray-500 mt-2">
                                {{ __('Kata sandi berhasil diperbarui.') }}
                            </p>
                        @endif
                    </form>
                </div>
            </div>

                        {{-- Kanan: Hapus Akun --}}
            <div class="flex-1 text-left flex flex-col items-start">
                <div class="bg-white/65 rounded-xl shadow p-6 border border-red-50 w-full">
                    <h2 class="text-lg font-semibold text-red-600 mb-3 flex items-center gap-2">
                        <span>⚠️</span> Hapus Akun
                    </h2>
                    <p class="text-sm text-gray-600 mb-4">
                        Sekali akun Anda dihapus, semua data dan sumber daya akan hilang permanen. 
                        Pastikan Anda telah menyimpan informasi penting sebelum melanjutkan proses ini.
                    </p>

                    {{-- Form hapus akun --}}
                    @include('profile.partials.delete-user-form')
                </div>

                {{-- Tombol Kembali ke Settings (digeser sedikit ke bawah & tengah) --}}
                <div class="mt-8 flex justify-center w-full">
                    <a href="{{ route('settings') }}"
                    class="bg-gray-700 hover:bg-gray-800 text-white px-6 py-2.5 rounded-lg transition shadow-md hover:shadow-lg text-sm">
                    Kembali ke Settings
                    </a>
                </div>
            </div>
        </div>
    </div>
</div>
@endsection
