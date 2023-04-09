#ifndef PLATE_H
#define PLATE_H

#include "object3d.hpp"
#include "plane.hpp"
#include "sphere.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement functions and add more fields as necessary

class Plate : public Object3D {
public:
    Plate() = delete;

    Plate(const Vector3f &center, const Vector3f dir, float radius, Material *material) : Object3D(material) {
        plateCenter = center;
        plateDirection = dir.normalized();
        plateRadius = radius;
        plane = new Plane(plateDirection, Vector3f::dot(plateDirection, plateCenter), material);
    }

    ~Plate() override
    {
        delete plane;
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        if (plane->intersect(r, h, tmin))
        {
            Vector3f hitPos = r.pointAtParameter(h.getT());
            if ((hitPos - plateCenter).length() < plateRadius)
            {
                return true;
            }
        }
        return false;
    }

protected:
    Vector3f plateCenter = Vector3f(0, 0, 0);
    Vector3f plateDirection = (1, 0, 0);
    float plateRadius = 1;
    Plane* plane;
};


#endif
