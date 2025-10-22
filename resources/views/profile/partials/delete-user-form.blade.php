<section class="space-y-6">
    {{-- Tombol utama --}}
    <x-danger-button
        x-data=""
        x-on:click.prevent="$dispatch('open-modal', 'confirm-user-deletion')"
        class="bg-red-600 hover:bg-red-700 text-white font-semibold px-4 py-2 rounded-md shadow-md transition"
    >
        {{ __('Hapus Akun') }}
    </x-danger-button>

    {{-- Modal konfirmasi --}}
    <x-modal name="confirm-user-deletion" :show="$errors->userDeletion->isNotEmpty()" focusable>
        <form method="post" action="{{ route('profile.destroy') }}" class="p-6">
            @csrf
            @method('delete')

            <h2 class="text-lg font-semibold text-gray-800 mb-3">
                {{ __('Apakah Anda yakin ingin menghapus akun ini?') }}
            </h2>

            <p class="text-sm text-gray-600 mb-5">
                {{ __('Tindakan ini bersifat permanen dan tidak dapat dibatalkan. Semua data Anda akan dihapus dari sistem.') }}
            </p>

            {{-- Password konfirmasi --}}
            <div class="mb-4">
                <x-input-label for="password" value="{{ __('Masukkan Kata Sandi untuk Konfirmasi') }}" class="sr-only" />
                <x-text-input
                    id="password"
                    name="password"
                    type="password"
                    class="mt-1 block w-full text-sm"
                    placeholder="{{ __('Kata sandi Anda...') }}"
                />
                <x-input-error :messages="$errors->userDeletion->get('password')" class="mt-2 text-xs text-red-500" />
            </div>

            {{-- Tombol aksi --}}
            <div class="flex justify-end gap-3">
                <x-secondary-button x-on:click="$dispatch('close')">
                    {{ __('Batal') }}
                </x-secondary-button>

                <x-danger-button class="bg-red-600 hover:bg-red-700 text-white">
                    {{ __('Hapus Akun') }}
                </x-danger-button>
            </div>
        </form>
    </x-modal>
</section>
