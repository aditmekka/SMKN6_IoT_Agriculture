<!DOCTYPE html>
<html lang="{{ str_replace('_', '-', app()->getLocale()) }}">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>{{ config('app.name', 'AQUA') }}</title>

    <!-- Fonts -->
    <link rel="preconnect" href="https://fonts.bunny.net">
    <link href="https://fonts.bunny.net/css?family=figtree:400,600&display=swap" rel="stylesheet" />

    <style>
        body {
            margin: 0;
            height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            font-family: 'Figtree', sans-serif;
            background: linear-gradient(180deg, #e0f2fe 0%, #bae6fd 50%, #93c5fd 100%);
            overflow: hidden;
        }

        /* Background waves */
        .ocean {
        position: absolute;
        bottom: 0;
        left: 0;
        width: 100%;
        height: 220px;
        overflow: hidden;
        line-height: 0;
        z-index: 0;
        }

        .wave-svg {
        position: relative;
        display: block;
        width: 300%; /* Lebar 3x layar biar looping mulus */
        height: 220px;
        }
        .wave {
            position: absolute;
            bottom: 0;
            left: 0;
            width: 200%;
            height: 180px;
            background: rgba(37, 99, 235, 0.3);
            border-radius: 100%;
            transform: translateX(-25%);
            animation: wave 10s infinite linear;
        }

        .wave:nth-child(2) {
            background: rgba(37, 99, 235, 0.2);
            animation-duration: 14s;
        }

        .wave:nth-child(3) {
            background: rgba(37, 99, 235, 0.15);
            animation-duration: 18s;
        }

        @keyframes wave {
            0% { transform: translateX(-25%); }
            100% { transform: translateX(-75%); }
        }

        .container {
            position: relative;
            text-align: center;
            z-index: 10;
            background: rgba(255, 255, 255, 0.85);
            padding: 50px 60px;
            border-radius: 20px;
            box-shadow: 0 8px 20px rgba(0, 0, 0, 0.1);
            backdrop-filter: blur(6px);
        }

        .title {
            font-size: 2.5rem;
            font-weight: 700;
            color: #1e3a8a;
            margin-bottom: 10px;
        }

        .subtitle {
            font-size: 1.1rem;
            color: #2563eb;
            margin-bottom: 35px;
        }

        .btn {
            display: inline-block;
            padding: 12px 32px;
            margin: 0 10px;
            font-weight: 600;
            border-radius: 8px;
            text-decoration: none;
            transition: all 0.3s ease;
        }

        .login {
            background-color: #2563eb;
            color: white;
        }

        .login:hover {
            background-color: #1e40af;
            transform: translateY(-3px);
        }

        .register {
            background-color: #10b981;
            color: white;
        }

        .register:hover {
            background-color: #047857;
            transform: translateY(-3px);
        }

        footer {
            position: absolute;
            bottom: 20px;
            text-align: center;
            width: 100%;
            color: #1e3a8a;
            font-size: 0.9rem;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="title">AQUA</div>
        <div class="subtitle">Automatic Quality Utility for Agriculture</div>

        @if (Route::has('login'))
            @auth
                <a href="{{ url('/dashboard') }}" class="btn login">Dashboard</a>
            @else
                <a href="{{ route('login') }}" class="btn login">Log In</a>
                @if (Route::has('register'))
                    <a href="{{ route('register') }}" class="btn register">Register</a>
                @endif
            @endauth
        @endif
    </div>

<!-- Perfect continuous ocean waves -->
<div class="ocean">
    <svg class="wave-svg" viewBox="0 0 1200 200" preserveAspectRatio="none">
        <!-- Lapisan 1 (gelombang paling depan) -->
        <path d="M0,100 C150,150 350,50 600,100 C850,150 1050,50 1200,100 L1200,200 L0,200 Z" 
              fill="rgba(37,99,235,0.4)">
            <animateTransform attributeName="transform" attributeType="XML" type="translate"
                from="0 0" to="-600 0" dur="6s" repeatCount="indefinite" />
        </path>

        <!-- Lapisan 2 -->
        <path d="M0,110 C200,160 400,60 700,110 C950,160 1150,60 1400,110 L1400,200 L0,200 Z" 
              fill="rgba(37,99,235,0.25)">
            <animateTransform attributeName="transform" attributeType="XML" type="translate"
                from="0 0" to="-700 0" dur="12s" repeatCount="indefinite" />
        </path>

        <!-- Lapisan 3 -->
        <path d="M0,120 C180,170 380,70 650,120 C950,170 1150,70 1400,120 L1400,200 L0,200 Z" 
              fill="rgba(37,99,235,0.18)">
            <animateTransform attributeName="transform" attributeType="XML" type="translate"
                from="0 0" to="-700 0" dur="18s" repeatCount="indefinite" />
        </path>

        <!-- Lapisan 4 -->
        <path d="M0,130 C180,180 380,80 650,130 C950,180 1150,80 1400,130 L1400,200 L0,200 Z" 
              fill="rgba(37,99,235,0.15)">
            <animateTransform attributeName="transform" attributeType="XML" type="translate"
                from="0 0" to="-700 0" dur="24s" repeatCount="indefinite" />
        </path>

        <!-- Lapisan 5 (ombak jauh / horizon) -->
        <path d="M0,140 C200,190 400,90 700,140 C950,190 1150,90 1400,140 L1400,200 L0,200 Z" 
              fill="rgba(37,99,235,0.12)">
            <animateTransform attributeName="transform" attributeType="XML" type="translate"
                from="0 0" to="-700 0" dur="36s" repeatCount="indefinite" />
        </path>
    </svg>
</div>



    <footer>
        © {{ date('Y') }} AQUA — Developed by Naufal Nadhif
    </footer>
</body>
</html>
