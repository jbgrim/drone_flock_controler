#ifndef VEC3_HPP
#define VEC3_HPP

#include <cmath>
#include <iostream>
#include <array>

class Vec3 {
public:
    float x, y, z;

    // Constructors
    Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3(const Vec3& other) : x(other.x), y(other.y), z(other.z) {}

    // Assignment operator
    Vec3& operator=(const Vec3& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            z = other.z;
        }
        return *this;
    }

    // Arithmetic operators
    Vec3 operator+(const Vec3& other) const { return Vec3(x + other.x, y + other.y, z + other.z); }
    Vec3 operator-(const Vec3& other) const { return Vec3(x - other.x, y - other.y, z - other.z); }
    Vec3 operator*(float scalar) const { return Vec3(x * scalar, y * scalar, z * scalar); }
    Vec3 operator/(float scalar) const { return Vec3(x / scalar, y / scalar, z / scalar); }
    Vec3 operator-() const { return Vec3(-x, -y, -z); }

    // Compound assignment operators
    Vec3& operator+=(const Vec3& other) { x += other.x; y += other.y; z += other.z; return *this; }
    Vec3& operator-=(const Vec3& other) { x -= other.x; y -= other.y; z -= other.z; return *this; }
    Vec3& operator*=(float scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
    Vec3& operator/=(float scalar) { x /= scalar; y /= scalar; z /= scalar; return *this; }

    // Vector operations
    float dot(const Vec3& other) const { return x * other.x + y * other.y + z * other.z; }
    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
    float magnitude() const { return std::sqrt(x * x + y * y + z * z); }
    float length() const { return magnitude(); }
    Vec3 normalized() const {
        float mag = magnitude();
        if (mag == 0.0f) return Vec3(0.0f, 0.0f, 0.0f);
        return (*this) / mag;
    }

    // Convert to std::array<float,3>
    std::array<float,3> to_array() const { return std::array<float,3>{x, y, z}; }

    // Utility functions
    void print() const { std::cout << "Vec3(" << x << ", " << y << ", " << z << ")" << std::endl; }
};

// Non-member operators
inline Vec3 operator*(double scalar, const Vec3& vec) { return vec * scalar; }
inline std::ostream& operator<<(std::ostream& os, const Vec3& vec) { os << "Vec3(" << vec.x << ", " << vec.y << ", " << vec.z << ")"; return os; }

#endif // VEC3_HPP
