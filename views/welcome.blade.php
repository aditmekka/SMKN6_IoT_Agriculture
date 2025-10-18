<!DOCTYPE html>
<html lang="{{ str_replace('_', '-', app()->getLocale()) }}">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>{{ config('app.name', 'Laravel') }}</title>

    <!-- Fonts -->
    <link rel="preconnect" href="https://fonts.bunny.net">
    <link href="https://fonts.bunny.net/css?family=figtree:400,600&display=swap" rel="stylesheet" />

    <style>
        body {
            font-family: 'Figtree', sans-serif;
            background-color: #f9fafb;
            display: flex;
            align-items: center;
            justify-content: center;
            height: 100vh;
            margin: 0;
        }

        .container {
            text-align: center;
        }

        .btn {
            display: inline-block;
            padding: 12px 28px;
            margin: 10px;
            font-weight: 600;
            text-decoration: none;
            border-radius: 8px;
            transition: 0.3s;
        }

        .login {
            background-color: #2563eb;
            color: white;
        }

        .login:hover {
            background-color: #1e40af;
        }

        .register {
            background-color: #10b981;
            color: white;
        }

        .register:hover {
            background-color: #047857;
        }
    </style>
</head>
<body>
    <div class="container">
        @if (Route::has('login'))
            @auth
                <a href="{{ url('/dashboard') }}" class="btn login">Dashboard</a>
            @else
                <a href="{{ route('login') }}" class="btn login">Log in</a>
                @if (Route::has('register'))
                    <a href="{{ route('register') }}" class="btn register">Register</a>
                @endif
            @endauth
        @endif
    </div>
</body>
</html>
