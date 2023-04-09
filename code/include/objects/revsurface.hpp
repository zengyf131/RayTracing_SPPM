#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "object3d.hpp"
#include "curve.hpp"
#include "mesh.hpp"
#include <tuple>

class RevSurface : public Object3D {

    Curve *pCurve;
    struct Surface {
        std::vector<float> VT;
        std::vector<Vector3f> VV;
        std::vector<Vector3f> VN;
        std::vector<TriangleIndex> VF;
    } surface;
    Mesh *surfaceMesh;
    const float inf = 1000000;
    const float epsilon = 1.0 / 1000;
    const int steps = 160;
    const int resolution = 30;
    const int repeatTime = 40;
    int yesRate = 0;
    int noRate = 0;
    float tRange[2];

public:

    RevSurface(Curve *pCurve, Material* material) : pCurve(pCurve), Object3D(material) {
        // Check flat.
        for (const auto &cp : pCurve->getControls()) {
            if (cp.z() != 0.0) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(0);
            }
        }
        pCurve->getTRange(tRange[0], tRange[1]);

        Vector3f minPoint = Vector3f(inf, inf, inf), maxPoint = Vector3f(-inf, -inf, -inf);
        std::vector<CurvePoint> curvePoints;
        pCurve->discretize(resolution, curvePoints);
        for (unsigned int ci = 0; ci < curvePoints.size(); ++ci) {
            const CurvePoint &cp = curvePoints[ci];
            for (unsigned int i = 0; i < steps; ++i) {
                float t = (float) i / steps;
                Quat4f rot;
                rot.setAxisAngle(t * 2 * M_PI, Vector3f::UP);
                Vector3f pnew = Matrix3f::rotation(rot) * cp.V;
                Vector3f pNormal = Vector3f::cross(cp.T, -Vector3f::FORWARD);
                Vector3f nnew = Matrix3f::rotation(rot) * pNormal;
                surface.VV.push_back(pnew);
                for (int j = 0; j < 3; j++)
                {
                    if (pnew[j] < minPoint[j])
                    {
                        minPoint[j] = pnew[j];
                    }
                    if (pnew[j] > maxPoint[j])
                    {
                        maxPoint[j] = pnew[j];
                    }
                }
                surface.VN.push_back(nnew);
                // cout << cp.t << endl;
                surface.VT.push_back(cp.t);
                int i1 = (i + 1 == steps) ? 0 : i + 1;
                if (ci != curvePoints.size() - 1) {
                    TriangleIndex t1, t2;
                    t1[0] = (ci + 1) * steps + i; t1[1] = ci * steps + i1; t1[2] = ci * steps + i;
                    t2[0] = (ci + 1) * steps + i; t2[1] = (ci + 1) * steps + i1; t2[2] = ci * steps + i1;
                    surface.VF.push_back(t1);
                    surface.VF.push_back(t2);
                }
            }
        }
        surfaceMesh = new Mesh(surface.VV, surface.VF, minPoint, maxPoint, material);
        surfaceMesh->fromCurve = true;
    }

    ~RevSurface() override {
        delete pCurve;
        delete surfaceMesh;
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {

        Hit originalHit = h;
        if (surfaceMesh->intersect(r, h, tmin))
        {
            // cout << h.getT() << endl;
            int totalHitNum = (int)h.hitPos.size() / 3;
            for (int hitNum = 0; hitNum < totalHitNum; hitNum++)
            {
                bool curveIntersect = false;
                float t = 0, theta = 0;
                int count_0 = 0, count_step = 0;
                for (int v = 0; v < 3; v++)
                {
                    for (int i = 0; i < (int)surface.VV.size(); i++)
                    {
                        if (surface.VV[i] == h.hitPos[h.hitPos.size() - 1])
                        {
                            // cout << "found" << endl;
                            t += surface.VT[i];
                            theta += (float)(i % steps) / steps * 2 * M_PI;
                            if (i % steps == 0) count_0++;
                            else if (i % steps == steps - 1) count_step++;
                            break;
                        }
                    }
                    h.hitPos.pop_back();
                }
                t /= 3;
                if (count_0 > 0 && count_step > 0)
                {
                    theta += count_0 * 2 * M_PI;
                }
                theta /= 3;
                Vector3f x(h.hitT[h.hitT.size() - 1], t, theta);
                h.hitT.pop_back();
                // cout << "begin" << endl;
                int i;
                Vector3f F = Vector3f::ZERO;
                Vector3f normal = Vector3f::ZERO;
                for (i = 0; i < repeatTime; i++)
                {
                    // cout << i << ": " << x[0] << ' ' << x[1] << ' ' << x[2] << endl;
                    if (x[1] < tRange[0]) x[1] = tRange[0] + epsilon;
                    else if (x[1] >= tRange[1]) x[1] = tRange[1] - epsilon;
                    if (x[2] < 0) x[2] += 2 * M_PI;
                    else if (x[2] > 2 * M_PI) x[2] = fmod(x[2], 2 * M_PI);
                    CurvePoint cp = pCurve->pointAtParameter(x[1]);
                    Quat4f rot;
                    rot.setAxisAngle(x[2], Vector3f::UP);
                    Vector3f pnew = Matrix3f::rotation(rot) * cp.V;
                    Vector3f dt = Matrix3f::rotation(rot) * cp.T;
                    Vector3f dtheta = Vector3f(-cp.V[0]*sin(x[2]), 0, -cp.V[0]*cos(x[2]));
                    normal = Vector3f::cross(dt, dtheta);
                    F = r.pointAtParameter(x[0]) - pnew;
                    // cout << F.length() << endl;
                    if (F.length() < epsilon)
                    {
                        curveIntersect = true;
                        break;
                    }
                    float D = Vector3f::dot(r.getDirection(), normal);
                    x[0] -= Vector3f::dot(dt, Vector3f::cross(dtheta, F)) / D;
                    x[1] -= Vector3f::dot(r.getDirection(), Vector3f::cross(dtheta, F)) / D;
                    x[2] += Vector3f::dot(r.getDirection(), Vector3f::cross(dt, F)) / D;
                }
                // cout << "end" << endl;
                if (curveIntersect && x[0] > tmin && x[0] < originalHit.getT())
                {
                    // cout << "curve: " << x[0] << ' ' << F.length() << endl;
                    yesRate++;
                    // cout << "yes! " << yesRate << '/' << noRate << endl;
                    h.set(x[0], material, normal);
                    return true;
                }
                // cout << "no" << endl;
                // if (i == 40)
                // {
                    
                //     cout << i << ':' << F.length() << endl;
                // }
                
                // cout << "no! " << yesRate << '/' << noRate << endl;
            }
            // return true;
            noRate++;
        }
        h = originalHit;
        return false;
    }

    
};


#endif //REVSURFACE_HPP
