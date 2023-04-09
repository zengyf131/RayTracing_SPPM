#ifndef SQUARE_H
#define SQUARE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

class Square : public Object3D {
public:
    Square() {
        minPoint = Vector3f(0, 0, 0);
        maxPoint = Vector3f(0, 0, 0);
    }

    Square(const Vector3f &min, const Vector3f &max, Material *m) : Object3D(m) 
    {
        // z = 0
        minPoint = min;
        maxPoint = max;
    }

    ~Square() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override 
    {
        float tScale = r.getDirection().length();
        if (r.getDirection().z() == 0) return false;
        float t = -r.getOrigin().z() / r.getDirection().z();
        Vector3f hitPos = r.getOrigin() + t * r.getDirection();
        if (hitPos.x() <= maxPoint.x() && hitPos.x() >= minPoint.x() && hitPos.y() <= maxPoint.y() && hitPos.y() >= minPoint.y() && t / tScale < h.getT() && t / tScale >= tmin)
        {
            
            float u = (hitPos.x() - minPoint.x()) / (maxPoint.x() - minPoint.x());
            float v = (hitPos.y() - minPoint.y()) / (maxPoint.y() - minPoint.y());
            Vector3f normal(0, 0, 1);
            if (material->normal_texture)
            {
                normal = 2*material->normal_texture->GetPixel(u*material->normal_texture->Width(), v*material->normal_texture->Height()) - Vector3f(1, 1, 1);
                // std::cout << normal.x() << ' ' << normal.y() << ' ' << normal.z() << std::endl;
            }
            h.set(t / tScale, material, normal, u, v);
            return true;
        }
        return false;
    }

protected:
    Vector3f minPoint, maxPoint;

};

#endif //PLANE_H
		

