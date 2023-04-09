#ifndef BOX_H
#define BOX_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

class Box : public Object3D {
public:
    Box() {
        minPoint = Vector3f::ZERO;
        maxPoint = Vector3f::ZERO;
        isAABB = false;
    }

    Box(const Vector3f &minP, const Vector3f &maxP, bool AABB, Material *m) : Object3D(m) {
        minPoint = minP;
        maxPoint = maxP;
        isAABB = AABB;
    }

    ~Box() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        float tScale = r.getDirection().length();
        Vector3f origin = r.getOrigin();
        Vector3f dir = r.getDirection().normalized();
        float max1 = -inf, min2 = inf;
        int maxPos = 0;
        for (int i = 0; i < 3; i++)
        {
            if (r.getDirection()[i] != 0)
            {
                float t1 = (minPoint[i] - origin[i]) / dir[i];
                float t2 = (maxPoint[i] - origin[i]) / dir[i];
                if (t1 > t2)
                {
                    float temp = t2;
                    t2 = t1;
                    t1 = temp;
                }
                if (t1 > max1) 
                {
                    max1 = t1;
                    maxPos = i;
                }
                if (t2 < min2) 
                {
                    min2 = t2;
                }
            }
            else if (minPoint[i] > origin[i] || origin[i] > maxPoint[i])
            {
                return false;
            }
        }
        float t;
        if (max1 / tScale < tmin)
        {
            t = min2;
        }
        else
        {
            t = max1;
        }
        if (max1 < min2 && t / tScale < h.getT() && t / tScale >= tmin)
        {
            if (!isAABB)
            {
                Vector3f normal = Vector3f::ZERO;
                normal[maxPos] = 1;
                normal = -Vector3f::dot(normal, dir);
                normal.normalize();
                h.set(t / tScale, material, normal);
            }
            return true;
        }
        return false;
    }

protected:
    Vector3f minPoint, maxPoint;
    bool isAABB;
    float inf = 1000000;
};

#endif //PLANE_H
		

