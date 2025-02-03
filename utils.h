#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <memory>

//Vec2D créée mais remplacée par std::vector
template <typename T>
struct Vec2D {
    T x, y;

    Vec2D(T x = 0, T y = 0) : x(x), y(y) {}

    Vec2D operator+(const Vec2D& other) const {
        return Vec2D(x + other.x, y + other.y);
    }

    Vec2D operator-(const Vec2D& other) const {
        return Vec2D(x - other.x, y - other.y);
    }

    Vec2D operator*(T scalar) const {
        return Vec2D(x * scalar, y * scalar);
    }

    Vec2D operator/(T scalar) const {
        return Vec2D(x / scalar, y / scalar);
    }

    T dot(const Vec2D& other) const {
        return x * other.x + y * other.y;
    }

    T magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    Vec2D normalize() const {
        T mag = magnitude();
        return Vec2D(x / mag, y / mag);
    }

    T& operator[](std::size_t idx) {
        if (idx == 0) return x;
        if (idx == 1) return y;
        throw std::out_of_range("Index out of range");
    }

    const T& operator[](std::size_t idx) const {
        if (idx == 0) return x;
        if (idx == 1) return y;
        throw std::out_of_range("Index out of range");
    }
};

float scalar_product(std::vector<float> v1, std::vector<float> v2) {
    return v1[0] * v2[0] + v1[1] * v2[1];
}