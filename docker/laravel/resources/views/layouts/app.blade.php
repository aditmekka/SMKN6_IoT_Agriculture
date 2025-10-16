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
</head>
<body class="font-sans antialiased bg-gray-100 dark:bg-gray-900">
    <div class="min-h-screen flex">

        <!-- Sidebar -->
        <aside class="w-64 bg-gray-800 text-gray-200">
            <div class="p-6 text-2xl font-bold border-b border-gray-700">
                AQUA
            </div>
            <nav class="mt-4">
                <a href="{{ route('dashboard') }}"
                   class="block px-6 py-3 hover:bg-gray-700 {{ request()->routeIs('dashboard') ? 'bg-gray-700' : '' }}">
                   Dashboard
                </a>
                <a href="#"
                   class="block px-6 py-3 hover:bg-gray-700">
                   Sensor Data
                </a>
                <a href="#"
                   class="block px-6 py-3 hover:bg-gray-700">
                   Device Control
                </a>
                <a href="#"
                   class="block px-6 py-3 hover:bg-gray-700">
                   Settings
                </a>
            </nav>
        </aside>

        <!-- Main Content -->
        <div class="flex-1 flex flex-col">
            @include('layouts.navigation')

            <!-- Page Header -->
            @isset($header)
                <header class="bg-white dark:bg-gray-800 shadow">
                    <div class="max-w-7xl mx-auto py-6 px-4 sm:px-6 lg:px-8">
                        {{ $header }}
                    </div>
                </header>
            @endisset

            <!-- Page Content -->
            <main class="flex-1 p-6">
                {{ $slot }}
            </main>
        </div>
    </div>
</body>
</html>
