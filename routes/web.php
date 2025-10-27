<?php

use App\Http\Controllers\ProfileController;
use App\Http\Controllers\SensorDataController;
use Illuminate\Support\Facades\Route;
use Illuminate\Support\Facades\URL;
use Illuminate\Support\Facades\Config;

/*
|--------------------------------------------------------------------------
| Fix domain & session buat LocalXpose
|--------------------------------------------------------------------------
| Baris ini memastikan session, cookie, dan CSRF tetap valid
| meskipun domain tunnel berubah (contoh: xxxxx.loclx.io)
*/
if (app()->environment('local')) {
    URL::forceScheme('http');
    Config::set('session.domain', null);
    Config::set('session.secure', false);
}

/*
|--------------------------------------------------------------------------
| Web Routes
|--------------------------------------------------------------------------
*/

Route::get('/', function () {
    return view('welcome');
});

Route::get('/dashboard', function () {
    return view('dashboard');
})->middleware(['auth', 'verified'])->name('dashboard');

Route::get('/settings', function () {
    return view('settings');
})->middleware(['auth', 'verified'])->name('settings');

Route::middleware('auth')->group(function () {
    Route::get('/profile', [ProfileController::class, 'edit'])->name('profile.edit');
    Route::patch('/profile', [ProfileController::class, 'update'])->name('profile.update');
    Route::delete('/profile', [ProfileController::class, 'destroy'])->name('profile.destroy');
});

Route::get('/device-control', function () {
    return view('device-control');
})->name('device-control');

/*
|--------------------------------------------------------------------------
| Sensor Data Route
|--------------------------------------------------------------------------
*/
Route::get('/sensor-data', [SensorDataController::class, 'index'])
    ->middleware(['auth', 'verified'])
    ->name('sensor.data');

require __DIR__ . '/auth.php';
