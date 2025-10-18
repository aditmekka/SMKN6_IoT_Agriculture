# ===========================
# Stage 1: Frontend build
# ===========================
FROM node:20-alpine AS build-assets

WORKDIR /var/www/html

# Copy only files needed for npm install first (for caching)
COPY package*.json vite.config.* ./

# Install node dependencies and build
RUN npm install
COPY resources ./resources
COPY public ./public
RUN npm run build


# ===========================
# Stage 2: PHP + Laravel
# ===========================
FROM php:8.3-fpm

# Install PHP extensions
RUN apt-get update && apt-get install -y \
    git curl zip unzip libpng-dev libonig-dev libxml2-dev \
    && docker-php-ext-install pdo_mysql mbstring exif pcntl bcmath gd

WORKDIR /var/www/html

# Copy all Laravel files
COPY . .

# Copy built Vite assets from previous stage
COPY --from=build-assets /var/www/html/public/build ./public/build

# Copy Composer from official image
COPY --from=composer:2 /usr/bin/composer /usr/bin/composer

# Install PHP dependencies
RUN composer install --no-dev --optimize-autoloader

# Optimize Laravel for production
RUN php artisan config:cache && php artisan route:cache && php artisan view:cache

# Set permissions
RUN chown -R www-data:www-data storage bootstrap/cache

EXPOSE 9000
CMD ["php-fpm"]
