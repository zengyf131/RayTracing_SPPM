#ifndef HIT_H
#define HIT_H

#include <vecmath.h>
#include <vector>
#include "ray.hpp"

class Material;

class Hit {
public:

    // constructors
    Hit() {
        material = nullptr;
        t = 1e38;
    }

    Hit(float _t, Material *m, const Vector3f &n) {
        t = _t;
        material = m;
        normal = n;
    }

    Hit(const Hit &h) {
        t = h.t;
        material = h.material;
        normal = h.normal;
        u = h.u;
        v = h.v;
    }

    // destructor
    ~Hit() = default;

    float getT() const {
        return t;
    }

    Material *getMaterial() const {
        return material;
    }

    const Vector3f &getNormal() const {
        return normal;
    }

    void set(float _t, Material *m, const Vector3f &n, float _u = -1, float _v = -1) {
        t = _t;
        material = m;
        normal = n;
        if (_u != -1 && _v != -1)
        {
            u = _u;
            v = _v;
        }
    }

    std::vector<Vector3f> hitPos;   // for curve surface
    std::vector<float> hitT;    // for curve surface
    float u = -1, v = -1; // for texture

private:

    float t;
    Material *material;
    Vector3f normal;
    

};

inline std::ostream &operator<<(std::ostream &os, const Hit &h) {
    os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
    return os;
}

#endif // HIT_H
