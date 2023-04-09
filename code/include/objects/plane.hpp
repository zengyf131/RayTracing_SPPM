#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Plane() {
        planeNormal = Vector3f(0, 0, 0);
        planeD = 0;
    }

    Plane(const Vector3f &normal, float d, Material *m) : Object3D(m) {
        planeNormal = normal;
        planeD = d;
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        float tScale = r.getDirection().length();
        float t = (planeD - Vector3f::dot(planeNormal, r.getOrigin())) / Vector3f::dot(planeNormal, r.getDirection().normalized());
        if (t / tScale < h.getT() && t / tScale >= tmin)
        {
            h.set(t / tScale, material, planeNormal);
            return true;
        }
        return false;
    }

protected:
    Vector3f planeNormal;
    float planeD;

};

#endif //PLANE_H
		

