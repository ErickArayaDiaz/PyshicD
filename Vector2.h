#pragma once
#include <cmath>   // std::sqrt
#include "Constants.h" // EPSILON

struct Vector2
{
    float x = 0.0f;
    float y = 0.0f;

    // Constructores
    constexpr Vector2() = default;
    constexpr Vector2(float x, float y) : x(x), y(y) {}

    // Suma, resta y negación 
    constexpr Vector2 operator+(const Vector2& o) const { return {x + o.x, y + o.y}; }
    constexpr Vector2 operator-(const Vector2& o) const { return {x - o.x, y - o.y}; }
    constexpr Vector2 operator-() const { return {-x, -y}; }

    // Escalar 
    constexpr Vector2 operator*(float s) const { return {x * s, y * s}; }
    constexpr Vector2 operator/(float s) const { return {x / s, y / s}; }

    // Operadores compuestos 
    constexpr Vector2& operator+=(const Vector2& o) { x += o.x; y += o.y; return *this; }
    constexpr Vector2& operator-=(const Vector2& o) { x -= o.x; y -= o.y; return *this; }
    constexpr Vector2& operator*=(float s)          { x *= s;   y *= s;   return *this; }
    constexpr Vector2& operator/=(float s)          { x /= s;   y /= s;   return *this; }

    // Producto punto
    constexpr float dot(const Vector2& o) const { return x * o.x + y * o.y; }

    // Longitud
    constexpr float lengthSq() const { return x * x + y * y; }
    float length() const { return std::sqrt(lengthSq()); }

    // Normalización 
    Vector2 normalized() const
    {
        const float len = length();
        if (len < EPSILON)
            return {0.0f, 0.0f};   // vector nulo: evita NaN
        return {x / len, y / len};
    }

    // Alias común para normalizar: v.normalize()
    Vector2 normalize() const { return normalized(); }
};

// Escalar * vector 
constexpr Vector2 operator*(float s, const Vector2& v)
{
    return {v.x * s, v.y * s};
}