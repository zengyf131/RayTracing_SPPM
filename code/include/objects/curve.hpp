#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include <vecmath.h>
#include <vector>
#include <utility>
#include <cmath>

#include <algorithm>

// The CurvePoint object stores information about a point on a curve
// after it has been tesselated: the vertex (V) and the tangent (T)
// It is the responsiblility of functions that create these objects to fill in all the data.
struct CurvePoint {
    double t = 0;
    Vector3f V = Vector3f::ZERO; // Vertex
    Vector3f T = Vector3f::ZERO; // Tangent  (unit)
};

class Curve : public Object3D {
protected:
    std::vector<Vector3f> controls;
public:
    explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {}

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        return false;
    }

    std::vector<Vector3f> &getControls() {
        return controls;
    }

    virtual void discretize(int resolution, std::vector<CurvePoint>& data) = 0;
    virtual CurvePoint pointAtParameter(double t) = 0;
    virtual void getTRange(float &l, float &h) = 0;
};

class BezierCurve : public Curve {
public:
    explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }
        n = controls.size() - 1;
        Cn = new int[n + 1];
        for (int i = 0; i <= n; i++)
        {
            int c = 1;
            for (int j = 1; j <= i; j++)
            {
                c = c * (n - i + j) / j;
            }
            Cn[i] = c;
        }
        Cn_1 = new int[n];
        for (int i = 0; i < n; i++)
        {
            int c = 1;
            for (int j = 1; j <= i; j++)
            {
                c = c * (n - i + j) / j;
            }
            Cn_1[i] = c;
        }
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();

        for (int p = 0; p < resolution; p++)
        {
            double t = (double)p / resolution;
            CurvePoint res;
            for (int i = 0; i <= n; i++)
            {
                res.V += Cn[i] * pow(t, i) * pow(1 - t, n - i) * controls[i];
            }
            Vector3f tan;
            for (int i = 0; i < n; i++)
            {
                res.T += Cn_1[i] * pow(t, i) * pow(1 - t, n - 1 - i) * (controls[i + 1] - controls[i]);
            }
            res.T *= n;
            res.t = t;
            data.push_back(res);
        }
    }

    CurvePoint pointAtParameter(double t) override
    {
        CurvePoint res;
        for (int i = 0; i <= n; i++)
        {
            res.V += Cn[i] * pow(t, i) * pow(1 - t, n - i) * controls[i];
        }
        Vector3f tan;
        for (int i = 0; i < n; i++)
        {
            res.T += Cn_1[i] * pow(t, i) * pow(1 - t, n - 1 - i) * (controls[i + 1] - controls[i]);
        }
        res.T *= n;
        res.t = t;
        return res;
    }

    void getTRange(float &l, float &h) override
    {
        l = 0;
        h = 1;
    }

protected:
    int* Cn, * Cn_1;
    int n;
};

class BsplineCurve : public Curve {
public:
    BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4) {
            printf("Number of control points of BspineCurve must be more than 4!\n");
            exit(0);
        }
        n = controls.size() - 1;
        
        knots = new double[n + k + 2];
        for (int i = 0; i < n + k + 2; i++)
        {
            knots[i] = (double)i / (n + k + 1);
        }
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        for (int i = k; i < n + 1; i++)
        {
            for (int d = 0; d < resolution; d++)
            {
                CurvePoint res;
                double t = (i + (double)d / resolution) / (n + k + 1);
                double* B = new double[n + k + 2];
                for (int j = 0; j <= n + k + 1; j++)
                {
                    B[j] = 0;
                }
                B[i] = 1;

                for (int time = 1; time < k; time++)
                {
                    for (int p = i - k; p <= i; p++)
                    {
                        B[p] = (t - knots[p]) / (knots[p+time] - knots[p]) * B[p]
                             + (knots[p+time+1] - t) / (knots[p+time+1] - knots[p+1]) * B[p+1];
                    }
                }
                
                for (int p = i - k; p <= i; p++)
                {
                    res.T += (B[p] / (knots[p+k] - knots[p]) - B[p+1] / (knots[p+k+1] - knots[p+1]))* controls[p];
                }
                res.T *= k;
                for (int p = i - k; p <= i; p++)
                {
                    B[p] = (t - knots[p]) / (knots[p+k] - knots[p]) * B[p]
                         + (knots[p+k+1] - t) / (knots[p+k+1] - knots[p+1]) * B[p+1];
                }
                for (int p = i - k; p <= i; p++)
                {
                    res.V += B[p] * controls[p];
                }
                res.t = t;
                data.push_back(res);
                delete[] B;
            }
        }
    }

    CurvePoint pointAtParameter(double t) override
    {
        int i = (int)(t * (n + k + 1));
        CurvePoint res;
        double* B = new double[n + k + 2];
        for (int j = 0; j <= n + k + 1; j++)
        {
            B[j] = 0;
        }
        B[i] = 1;

        for (int time = 1; time < k; time++)
        {
            for (int p = i - k; p <= i; p++)
            {
                B[p] = (t - knots[p]) / (knots[p+time] - knots[p]) * B[p]
                        + (knots[p+time+1] - t) / (knots[p+time+1] - knots[p+1]) * B[p+1];
            }
        }
        
        for (int p = i - k; p <= i; p++)
        {
            res.T += (B[p] / (knots[p+k] - knots[p]) - B[p+1] / (knots[p+k+1] - knots[p+1]))* controls[p];
        }
        res.T *= k;
        for (int p = i - k; p <= i; p++)
        {
            B[p] = (t - knots[p]) / (knots[p+k] - knots[p]) * B[p]
                    + (knots[p+k+1] - t) / (knots[p+k+1] - knots[p+1]) * B[p+1];
        }
        for (int p = i - k; p <= i; p++)
        {
            res.V += B[p] * controls[p];
        }
        res.t = t;
        delete[] B;
        return res;
    }

    void getTRange(float &l, float &h) override
    {
        l = (float)k / (n + k + 1);
        h = (float)(n + 1) / (n + k + 1);
    }

protected:
    int k = 3;
    int n;
    double* knots;
};

#endif // CURVE_HPP
