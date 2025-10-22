<?php

namespace App\Http\Middleware;

use Illuminate\Foundation\Http\Middleware\VerifyCsrfToken as Middleware;
use Closure;

class VerifyCsrfToken extends Middleware
{
    /**
     * Jalankan middleware.
     */
    public function handle($request, Closure $next)
    {
        // Kalau environment LOCAL, skip CSRF check
        if (app()->environment('local')) {
            return $next($request);
        }

        // Selain itu (production, staging, dll) tetap cek CSRF
        return parent::handle($request, $next);
    }

    protected $except = [
        '*', // opsional, tapi biar pasti semua route dilewatin
    ];
}
