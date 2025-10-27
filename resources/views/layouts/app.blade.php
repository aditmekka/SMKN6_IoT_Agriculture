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

        /* Bagian bawah sidebar (user info) */
        .user-section {
            background-color: #2563eb; /* biru */
            text-align: center;
            padding: 1rem;
            font-weight: 500;
            border-top: 1px solid rgba(255, 255, 255, 0.2);
        }

        /* ====== Animasi Fade-In Smooth ====== */
        @keyframes fadeInUp {
            0% {
                opacity: 0;
                transform: translateY(15px);
            }
            100% {
                opacity: 1;
                transform: translateY(0);
            }
        }

        .fade-in-up {
            animation: fadeInUp 0.6s ease-out both;
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
                    <!-- Dashboard -->
                    <a href="{{ route('dashboard') }}"
                       class="block px-6 py-3 rounded hover:bg-gray-700/60 transition 
                       {{ request()->routeIs('dashboard') ? 'bg-gray-700/80 text-white' : 'text-gray-300' }}">
                        Dashboard
                    </a>

                    <!-- Sensor Data -->
                    <a href="{{ route('sensor.data') }}"
                       class="block px-6 py-3 rounded hover:bg-gray-700/60 transition 
                       {{ request()->routeIs('sensor.data') ? 'bg-gray-700/80 text-white' : 'text-gray-300' }}">
                        Sensor Data
                    </a>

                    <!-- Device Control -->
                    <a href="{{ route('device-control') }}"
                       class="block px-6 py-3 rounded hover:bg-gray-700/60 transition 
                       {{ request()->routeIs('device-control') ? 'bg-gray-700/80 text-white' : 'text-gray-300' }}">
                        Device Control
                    </a>

                    <!-- Settings -->
                    <a href="{{ route('settings') }}"
                       class="block px-6 py-3 rounded hover:bg-gray-700/60 transition 
                       {{ request()->routeIs('settings') ? 'bg-gray-700/80 text-white' : 'text-gray-300' }}">
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
