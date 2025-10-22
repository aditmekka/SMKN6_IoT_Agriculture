<?php

namespace App\Http\Middleware;

use Closure;
use Illuminate\Support\Facades\Cookie;

class TunnelCookieFix
{
    public function handle($request, Closure $next)
    {
        $response = $next($request);

        // Bikin cookie CSRF yang sesuai domain .loclx.io
        if ($request->is('login') || $request->is('register')) {
            $token = csrf_token();
            Cookie::queue('XSRF-TOKEN', $token, 120, '/', '.loclx.io', false, false);
        }

        return $response;
    }
}
