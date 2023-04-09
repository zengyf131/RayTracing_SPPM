#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: implement this class and add more fields as necessary,
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m, bool i = false, const Vector3f &an = Vector3f::ZERO, const Vector3f &bn = Vector3f::ZERO, const Vector3f &cn = Vector3f::ZERO) : Object3D(m) {
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
		normal = Vector3f::cross(a - b, b - c).normalized();
		interpolate = i;
		vertexNormal[0] = an;
		vertexNormal[1] = bn;
		vertexNormal[2] = cn;
	}

	bool intersect( const Ray& ray,  Hit& hit , float tmin) override {
		float tScale = ray.getDirection().length();
		Vector3f E1 = vertices[0] - vertices[1], E2 = vertices[0] - vertices[2];
		Vector3f S = vertices[0] - ray.getOrigin();
		float coefficient = 1 / Matrix3f(ray.getDirection().normalized(), E1, E2).determinant();
		float t = coefficient * Matrix3f(S, E1, E2).determinant();
		float beta = coefficient * Matrix3f(ray.getDirection().normalized(), S, E2).determinant();
		float gamma = coefficient * Matrix3f(ray.getDirection().normalized(), E1, S).determinant();
		if (t / tScale < hit.getT() && t / tScale >= tmin && beta <= 1 && beta >= 0 && gamma <= 1 && gamma >= 0 && beta + gamma <= 1)
		{
			Vector3f n = normal;
			if (interpolate)
			{
				n = (1 - beta - gamma) * vertexNormal[0] + beta * vertexNormal[1] + gamma * vertexNormal[2];
			}
			if (Vector3f::dot(n, S) > 0)
			{
				n = -n;
			}
			hit.set(t / tScale, material, n);
			return true;
		}

        return false;
	}

	bool interpolate;
	Vector3f normal;
	Vector3f vertices[3];
	Vector3f vertexNormal[3];
protected:
};

#endif //TRIANGLE_H
