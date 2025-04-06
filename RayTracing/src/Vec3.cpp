#include <iostream>
#include <math.h>

class Vec3 {
    public:
        float x, y, z;
    
        Vec3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    
        Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
        Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
        Vec3 operator*(float s) const { return Vec3(x*s, y*s, z*s); }
        Vec3 operator/(float s) const { return Vec3(x/s, y/s, z/s); }
        Vec3& operator+=(const Vec3& v) {
            x += v.x; y += v.y; z += v.z;
            return *this;
        }
        float dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
        Vec3 cross(const Vec3& v) const {
            return Vec3(
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x
            );
        }
    
        float length() const { return std::sqrt(x*x + y*y + z*z); }
        Vec3 normalize() const {
            float len = length();
            if (len == 0) return *this;
            return *this / len;
        }
    
        void print() const {
            std::cout << x << ", " << y << ", " << z << "\n";
        }
    };

    // Scalar * Vec3
Vec3 operator*(float s, const Vec3& v) {
    return Vec3(v.x * s, v.y * s, v.z * s);
}

// Vec3 * Vec3 (component-wise multiplication)
Vec3 operator*(const Vec3& a, const Vec3& b) {
    return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

