#include "Vec3.cpp"
class Ray {
    public:
        Vec3 origin;
        Vec3 direction; 
    
        Ray() {}
    
        Ray(const Vec3& origin, const Vec3& direction)
            : origin(origin), direction(direction.normalize()) {} // normalize to ensure it's a unit vector
    
        // Get a point along the ray at distance t
        Vec3 at(float t) const {
            return origin + direction * t;
        }
    };