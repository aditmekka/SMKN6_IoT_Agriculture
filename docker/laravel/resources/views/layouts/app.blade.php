<!DOCTYPE html>
<html lang="{{ str_replace('_', '-', app()->getLocale()) }}">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta name="csrf-token" content="{{ csrf_token() }}">
    <title>{{ config('app.name', 'Laravel') }}</title>

    <!-- Fonts -->
    <link rel="preconnect" href="https://fonts.bunny.net">
    <link href="https://fonts.bunny.net/css?family=figtree:400,500,600&display=swap" rel="stylesheet" />

    <!-- Scripts -->
    @vite(['resources/css/app.css', 'resources/js/app.js'])

    <style>
        /* Background */
        .main-bg {
            background-image: url('{{ asset("assets/images/Background.png") }}');
            background-size: cover;
            background-position: center;
            background-repeat: no-repeat;
            min-height: 100vh;
        }

        /* Sidebar */
        aside {
            background-color: rgba(31, 41, 55, 0.95);
            backdrop-filter: blur(6px);
            display: flex;
            flex-direction: column;
            justify-content: space-between;
        }

        /* Kontainer isi (biar konten tetap kebaca) 
        .content-card {
            background-color: rgba(255, 255, 255, 0.85);
            border-radius: 1rem;
            padding: 2rem;
            box-shadow: 0 4px 10px rgba(0,0,0,0.1);
        }
        */

        /* Bagian bawah sidebar (user info) */
        .user-section {
            background-color: #2563eb; /* biru */
            text-align: center;
            padding: 1rem;
            font-weight: 500;
            border-top: 1px solid rgba(255, 255, 255, 0.2);
        }
    </style>
</head>
<body class="font-sans antialiased">
    <div class="min-h-screen flex">
        <!-- Sidebar -->
        <aside class="w-64 text-gray-200">
            <div>
                <div class="p-6 text-2xl font-bold border-b border-gray-700">
                    AQUA
                </div>
                <nav class="mt-4">
                    <a href="{{ route('dashboard') }}"
                       class="block px-6 py-3 hover:bg-gray-700/60 {{ request()->routeIs('dashboard') ? 'bg-gray-700/80' : '' }}">
                       Dashboard
                    </a>
                    <a href="#"
                       class="block px-6 py-3 hover:bg-gray-700/60">
                       Sensor Data
                    </a>
                    <a href="#"
                       class="block px-6 py-3 hover:bg-gray-700/60">
                       Device Control
                    </a>
                    <a href="{{ route('settings') }}"
                        class="block px-6 py-3 hover:bg-gray-700/60 {{ request()->routeIs('settings') ? 'bg-gray-700/80' : '' }}">
                        Settings
                    </a>
                </nav>
            </div>

                <!-- Bagian bawah: nama user + tombol logout -->
                    <div class="user-section flex flex-col items-center space-y-2">
                        <div>👤 {{ Auth::user()->name }}</div>

                        <form method="POST" action="{{ route('logout') }}">
                            @csrf
                            <button type="submit"
                                class="text-sm bg-blue-500 hover:bg-blue-600 text-white px-3 py-1 rounded transition duration-200">
                                Logout
                            </button>
                        </form>
                    </div>
        </aside>

                    <!-- Main content -->
            <div class="flex-1 flex flex-col main-bg">
                <main class="flex-1 p-6">
                    @yield('content')
                </main>
            </div>
    </div>
</body>
</html>
