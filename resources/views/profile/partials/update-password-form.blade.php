<section class="bg-white/80 backdrop-blur-md rounded-2xl shadow-lg p-8 max-w-lg mx-auto mt-6 transition-all duration-500 hover:shadow-2xl">
    <header class="mb-6 text-center">
        <h2 class="text-2xl font-bold text-blue-700 mb-2">
            🔒 Ubah Kata Sandi
        </h2>
        <p class="text-sm text-gray-600">
            Pastikan akun Anda menggunakan kata sandi yang kuat dan sulit ditebak.
        </p>
    </header>

    <form method="post" action="{{ route('password.update') }}" class="space-y-6">
        @csrf
        @method('put')

        <!-- Password Sekarang -->
        <div class="text-left">
            <label for="update_password_current_password" class="block font-semibold text-gray-700 mb-1">
                Kata Sandi Sekarang
            </label>
            <x-text-input id="update_password_current_password" name="current_password" type="password"
                class="block w-full rounded-lg border-gray-300 focus:ring-2 focus:ring-blue-400 focus:border-blue-400 transition"
                autocomplete="current-password" />
            <x-input-error :messages="$errors->updatePassword->get('current_password')" class="mt-2 text-red-600 text-sm" />
        </div>

        <!-- Password Baru -->
        <div class="text-left">
            <label for="update_password_password" class="block font-semibold text-gray-700 mb-1">
                Kata Sandi Baru
            </label>
            <x-text-input id="update_password_password" name="password" type="password"
                class="block w-full rounded-lg border-gray-300 focus:ring-2 focus:ring-blue-400 focus:border-blue-400 transition"
                autocomplete="new-password" />
            <x-input-error :messages="$errors->updatePassword->get('password')" class="mt-2 text-red-600 text-sm" />
        </div>

        <!-- Konfirmasi Password -->
        <div class="text-left">
            <label for="update_password_password_confirmation" class="block font-semibold text-gray-700 mb-1">
                Konfirmasi Kata Sandi Baru
            </label>
            <x-text-input id="update_password_password_confirmation" name="password_confirmation" type="password"
                class="block w-full rounded-lg border-gray-300 focus:ring-2 focus:ring-blue-400 focus:border-blue-400 transition"
                autocomplete="new-password" />
            <x-input-error :messages="$errors->updatePassword->get('password_confirmation')" class="mt-2 text-red-600 text-sm" />
        </div>

        <!-- Tombol -->
        <div class="flex justify-center items-center gap-4 pt-4">
            <button type="submit"
                class="bg-blue-600 hover:bg-blue-700 text-white font-semibold px-6 py-2 rounded-lg shadow-md transition transform hover:scale-105">
                Simpan Perubahan
            </button>

            @if (session('status') === 'password-updated')
                <p x-data="{ show: true }" x-show="show" x-transition
                   x-init="setTimeout(() => show = false, 2000)"
                   class="text-sm text-green-600 font-medium">
                    Kata sandi berhasil diperbarui ✅
                </p>
            @endif
        </div>
    </form>
</section>
