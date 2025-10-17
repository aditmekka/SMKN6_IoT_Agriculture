@echo off
REM ----------------------------
REM Laravel Docker Setup Script
REM ----------------------------

REM Go to the folder where docker-compose.yml is located
cd docker

echo Starting Docker containers...
docker compose up -d

REM Wait a few seconds to let containers initialize
timeout /t 10 /nobreak

echo Checking .env file...
docker compose exec laravel-app bash -c "if [ ! -f .env ]; then cp .env.example .env; fi"

echo Ensuring required directories exist and are writable...
docker compose exec laravel-app bash -c "mkdir -p bootstrap/cache storage && chmod -R 775 bootstrap/cache storage && chown -R www-data:www-data bootstrap/cache storage"

echo Installing Composer dependencies...
docker compose exec laravel-app bash -c "composer install --no-interaction --optimize-autoloader"

echo Generating Laravel APP_KEY...
docker compose exec laravel-app bash -c "php artisan key:generate"

echo Running migrations (optional)...
docker compose exec laravel-app bash -c "php artisan migrate --force"

echo.
echo Setup complete! You can now open http://localhost:8080
pause