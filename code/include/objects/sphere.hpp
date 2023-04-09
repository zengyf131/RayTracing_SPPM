#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement functions and add more fields as necessary

class Sphere : public Object3D {
public:
    Sphere() {
        sphereCenter = Vector3f(0, 0, 0);
        sphereRadius = 1;
    }

    Sphere(const Vector3f &center, float radius, Material *material) : Object3D(material) {
        sphereCenter = center;
        sphereRadius = radius;
    }

    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        Vector3f l = sphereCenter - r.getOrigin();
        bool isOutside = (Vector3f::dot(l, l) - sphereRadius * sphereRadius > tmin_in);
        float tScale = r.getDirection().length();
        float tp = Vector3f::dot(l, r.getDirection().normalized());
        if (isOutside && tp < 0)
        {
            return false;
        }
        float d2 = Vector3f::dot(l, l) - tp * tp;
        if (d2 <= sphereRadius * sphereRadius)
        {
            float dt2 = sphereRadius * sphereRadius - d2;
            float t = 0;
            if (isOutside)
            {
                t = tp - sqrt(dt2);
            }
            else
            {
                t = tp + sqrt(dt2);
            }
            if (t / tScale >= h.getT())
            {
                return false;
            }
            if (t / tScale >= tmin)
            {
                Vector3f n = r.pointAtParameter(t) - sphereCenter;
                n.normalize();
                h.set(t / tScale, material, n);
                return true;
            }
        }
        return false;
    }

protected:
    Vector3f sphereCenter;
    float sphereRadius;
    float tmin_in = 1.0 / 10000;

};


#endif
