@echo off
setlocal
echo ===============================
echo   Laravel Docker Setup Script
echo ===============================
echo.

REM --- Paths ---
set LARAVEL_DIR=laravel
set ENV_FILE=%LARAVEL_DIR%\.env
set NGINX_CONF=%LARAVEL_DIR%\nginx.conf

REM --- Ensure .env exists ---
if not exist "%ENV_FILE%" (
    echo Creating default .env file...
    (
        echo APP_NAME=ESP32Dashboard
        echo APP_ENV=local
        echo APP_KEY=
        echo APP_DEBUG=true
        echo APP_URL=http://localhost:8080
        echo.
        echo DB_CONNECTION=mysql
        echo DB_HOST=mariadb
        echo DB_PORT=3306
        echo DB_DATABASE=iot_data
        echo DB_USERNAME=iotuser
        echo DB_PASSWORD=iotpass
    ) > "%ENV_FILE%"
    echo .env file created.
) else (
    echo .env file already exists.
)

REM --- Ensure nginx.conf exists and is a file ---
if exist "%NGINX_CONF%\" (
    echo Detected nginx.conf is a folder, renaming it...
    ren "%NGINX_CONF%" "nginx_conf_backup"
)

if not exist "%NGINX_CONF%" (
    echo Creating default nginx.conf...
    (
        echo server {
        echo     listen 80;
        echo     index index.php index.html;
        echo     server_name localhost;
        echo.
        echo     root /var/www/html/public;
        echo.
        echo     location / {
        echo         try_files ^$uri ^$uri/ /index.php?$query_string;
        echo     }
        echo.
        echo     location ~ \.php$ {
        echo         fastcgi_pass laravel-app:9000;
        echo         fastcgi_index index.php;
        echo         fastcgi_param SCRIPT_FILENAME ^$realpath_root^$fastcgi_script_name;
        echo         include fastcgi_params;
        echo     }
        echo.
        echo     location ~ /\.ht {
        echo         deny all;
        echo     }
        echo }
    ) > "%NGINX_CONF%"
    echo nginx.conf created.
) else (
    echo nginx.conf already exists.
)

REM --- Start Docker containers ---
echo.
echo Starting Docker containers...
docker compose up -d --build
if errorlevel 1 (
    echo Docker failed to start. Check Docker Desktop or your compose file.
    pause
    exit /b
)

REM --- Install Composer dependencies ---
echo.
echo Installing Composer dependencies...
docker compose exec laravel-app composer install --no-interaction --prefer-dist

REM --- Generate APP_KEY if not set ---
echo.
echo Generating Laravel APP_KEY...
docker compose exec laravel-app php artisan key:generate

REM --- Optional: run migrations ---
echo.
echo Running migrations (optional)...
docker exec laravel-app php artisan migrate || echo Skipping migrations.

echo.
echo Setup complete! You can now open http://localhost:8080
pause
