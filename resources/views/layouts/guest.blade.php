<!DOCTYPE html>
<html lang="{{ str_replace('_', '-', app()->getLocale()) }}">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>{{ config('app.name', 'AQUA') }}</title>

    <!-- Fonts -->
    <link rel="preconnect" href="https://fonts.bunny.net">
    <link href="https://fonts.bunny.net/css?family=figtree:400,500,600&display=swap" rel="stylesheet" />

    <!-- Scripts -->
    @vite(['resources/css/app.css', 'resources/js/app.js'])

    <style>
        body {
            font-family: 'Figtree', sans-serif;
            background: linear-gradient(180deg, #e0f2fe 0%, #93c5fd 100%);
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            position: relative;
            overflow: hidden;
        }

        .auth-container {
            background: rgba(255, 255, 255, 0.85);
            backdrop-filter: blur(8px);
            border-radius: 1.5rem;
            box-shadow: 0 8px 20px rgba(0,0,0,0.15);
            padding: 2.5rem;
            width: 400px;
            z-index: 10;
        }

        .wave-wrapper {
            position: absolute;
            bottom: 0;
            left: 0;
            width: 100%;
            overflow: hidden;
            height: 200px;
            line-height: 0;
        }

        .wave-svg {
            width: 300%;
            height: 200px;
        }
    </style>
</head>
<body>
    <div class="auth-container">
        {{ $slot }}
    </div>

    <!-- Waves -->
    <div class="wave-wrapper">
        <svg class="wave-svg" viewBox="0 0 1200 200" preserveAspectRatio="none">
            <path d="M0,100 C150,150 350,50 600,100 C850,150 1050,50 1200,100 L1200,200 L0,200 Z" fill="rgba(37,99,235,0.4)">
                <animateTransform attributeName="transform" attributeType="XML" type="translate"
                    from="0 0" to="-600 0" dur="6s" repeatCount="indefinite" />
            </path>
            <path d="M0,110 C200,160 400,60 700,110 C950,160 1150,60 1400,110 L1400,200 L0,200 Z" fill="rgba(37,99,235,0.25)">
                <animateTransform attributeName="transform" attributeType="XML" type="translate"
                    from="0 0" to="-700 0" dur="12s" repeatCount="indefinite" />
            </path>
            <path d="M0,120 C180,170 380,70 650,120 C950,170 1150,70 1400,120 L1400,200 L0,200 Z" fill="rgba(37,99,235,0.18)">
                <animateTransform attributeName="transform" attributeType="XML" type="translate"
                    from="0 0" to="-700 0" dur="18s" repeatCount="indefinite" />
            </path>
            <path d="M0,130 C180,180 380,80 650,130 C950,180 1150,80 1400,130 L1400,200 L0,200 Z" fill="rgba(37,99,235,0.15)">
                <animateTransform attributeName="transform" attributeType="XML" type="translate"
                    from="0 0" to="-700 0" dur="24s" repeatCount="indefinite" />
            </path>
        </svg>
    </div>
</body>
</html>
