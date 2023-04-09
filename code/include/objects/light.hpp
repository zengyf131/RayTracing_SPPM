#ifndef LIGHT_H
#define LIGHT_H

#include <Vector3f.h>
#include <random>
#include <ctime>
#include <cmath>
#include "object3d.hpp"
#include "ray.hpp"
#include "plate.hpp"
#include "sphere.hpp"

class Light : public Object3D {
public:
    Light(Material *m) : Object3D(m) {}

    virtual ~Light() = default;

    virtual bool intersect(const Ray &r, Hit &h, float tmin) = 0;

    virtual void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const = 0;

    virtual void getPhoton(Ray &ray, float &power) = 0;

    virtual Vector3f getColor() const = 0;

    virtual int getPhotonAmount() const = 0;

    float getRand()
    {
        return (randInt() % 1000000) / 1000000.0;
    }

    std::mt19937 randInt;
    float epsilon = 1.0 / 1000000;
};

class DirectionalLight : public Light {
public:
    DirectionalLight() = delete;

    DirectionalLight(const Vector3f &d, const Vector3f &c, const Vector3f &pos, const float &r, const float &p, const int &amount, Material *m) : Light(m) 
    {
        direction = d.normalized();
        if (direction[0] == 0 && direction[1] == 0)
        {
            direction_angle[0] = 0;
            direction_angle[1] = M_PI_2;
        }
        else
        {
            Vector3f temp = direction - Vector3f::dot(direction, Vector3f(0, 0, 1))*Vector3f(0, 0, 1);
            direction_angle[0] = acos(Vector3f::dot(temp, Vector3f(1, 0, 0)) / temp.length());
            if (temp.y() < 0) direction_angle[0] = -direction_angle[0];
            direction_angle[1] = acos(Vector3f::dot(direction, Vector3f(0, 0, 1)));
            if (direction_angle[1] > M_PI / 2) direction_angle[1] -= M_PI;
        }
        
        color = c;
        position = pos;
        radius = r;
        power = p;
        photonAmount = amount;
        randomEngine = std::default_random_engine(time(0));
        randomDist = std::uniform_real_distribution<float>(0, radius);
        randomAngle = std::uniform_real_distribution<float>(0, 2 * M_PI);
        plate = new Plate(position, direction, radius, material);
        randInt = std::mt19937(time(0));
    }

    ~DirectionalLight() override 
    {
        delete plate;
    }

    ///@param p unsed in this function
    ///@param distanceToLight not well defined because it's not a point light
    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = -direction;
        col = color;
    }

    void getPhoton(Ray &r, float &p) override
    {
        float dist = randomDist(randomEngine);
        float angle = randomAngle(randomEngine);
        Vector3f offset(dist*(-sin(angle)*sin(direction_angle[0]) - cos(angle)*sin(direction_angle[1])*cos(direction_angle[0])), 
                        dist*(sin(angle)*cos(direction_angle[0]) - cos(angle)*sin(direction_angle[1])*sin(direction_angle[0])), 
                        dist*cos(angle)*cos(direction_angle[1]));
        r = Ray(offset + position, direction, 1);
        // std::cout << offset.x() << ' ' << offset.y() << ' ' << offset.z() << std::endl;
        // std::cout << direction_angle[0] << ' ' << direction_angle[1] << std::endl;
        p = power / (M_PI * radius * radius) / photonAmount;   // TODO necessary?
    }

    bool intersect(const Ray &r, Hit &h, float tmin)
    {
        return plate->intersect(r, h, tmin);
    }

    Vector3f getColor() const override
    {
        return color;
    }

    int getPhotonAmount() const override
    {
        return photonAmount;
    }

private:

    std::default_random_engine randomEngine;
    std::uniform_real_distribution<float> randomDist;
    std::uniform_real_distribution<float> randomAngle;
    Vector3f direction;
    float direction_angle[2];
    Vector3f color;
    Vector3f position;
    float radius;
    float power;
    int photonAmount;
    Plate* plate = nullptr;
};

class PointLight : public Light {
public:
    PointLight() = delete;

    PointLight(const Vector3f &p, const Vector3f &c, const float &r, const float &pow, const int &amount, Material *m) : Light(m) 
    {
        position = p;
        color = c;
        radius = r;
        power = pow;
        photonAmount = amount;
        randInt = std::mt19937(time(0));
        if (radius > 0)
        {
            sphere = new Sphere(position, radius, material);
        }
    }

    ~PointLight() override
    {
        if (sphere) delete sphere;
    }

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = (position - p);
        dir = dir / dir.length();
        col = color;
    }

    void getPhoton(Ray &r, float &p) override
    {
        float u = getRand() * 2 * M_PI;
        float v = getRand() * M_PI - M_PI / 2;
        Vector3f dir = Vector3f(sin(u)*cos(v), cos(u)*cos(v), sin(v));
        dir.normalize();
        r = Ray(position + radius * dir, dir, 1);
        p = power / (4 * M_PI) / photonAmount;
    }

    bool intersect(const Ray &r, Hit &h, float tmin)
    {
        return sphere && sphere->intersect(r, h, tmin);
    }

    Vector3f getColor() const override
    {
        return color;
    }

    int getPhotonAmount() const override
    {
        return photonAmount;
    }

private:
    
    Vector3f position;
    Vector3f color;
    float radius;
    float power;
    int photonAmount;
    Sphere* sphere = nullptr;
};

class SpotLight : public Light {
public:
    SpotLight() = delete;

    SpotLight(const Vector3f &d, const Vector3f &c, const Vector3f &pos, const float &r, const float a, const float &p, const int &amount, Material *m) : Light(m) 
    {
        direction = d.normalized();
        if (direction[0] == 0 && direction[1] == 0)
        {
            direction_angle[0] = 0;
            direction_angle[1] = (direction[2] > 0) ? M_PI_2 : -M_PI_2;
        }
        else
        {
            Vector3f temp = direction - Vector3f::dot(direction, Vector3f(0, 0, 1))*Vector3f(0, 0, 1);
            direction_angle[0] = acos(Vector3f::dot(temp, Vector3f(1, 0, 0)) / temp.length());
            if (temp.y() < 0) direction_angle[0] = -direction_angle[0];
            direction_angle[1] = acos(temp.length() / direction.length());
            if (direction[2] < 0) direction_angle[1] = -direction_angle[1];
        }
        for (int i = 0; i < 2; i++)
        {
            sin_angle[i] = sin(direction_angle[i]);
            cos_angle[i] = cos(direction_angle[i]);
            if (sin_angle[i] > -epsilon && sin_angle[i] < epsilon)
            {
                sin_angle[i] = 0;
            }
            else if (sin_angle[i] - 1 > -epsilon && sin_angle[i] - 1 < epsilon)
            {
                sin_angle[i] = 1;
            }
            if (cos_angle[i] > -epsilon && cos_angle[i] < epsilon)
            {
                cos_angle[i] = 0;
            }
            else if (cos_angle[i] - 1 > -epsilon && cos_angle[i] - 1 < epsilon)
            {
                cos_angle[i] = 1;
            }
        }
        color = c;
        position = pos;
        radius = r;
        maxAngle = a / 180 * M_PI;
        power = p;
        photonAmount = amount;
        plate = new Plate(position, direction, radius, material);
        randInt = std::mt19937(time(0));
    }

    ~SpotLight() override 
    {
        delete plate;
    }

    ///@param p unsed in this function
    ///@param distanceToLight not well defined because it's not a point light
    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = -direction;
        col = color;
    }

    void getPhoton(Ray &r, float &p) override
    {
        float oriDist = radius / tan(maxAngle);
        float angle = getRand() * maxAngle;
        float dist = sqrt(getRand() * radius * radius);
        float theta = getRand() * 2 * M_PI;
        Vector3f offset(dist*(-sin(theta)*sin_angle[0] - cos(theta)*sin_angle[1]*cos_angle[0]), 
                        dist*(sin(theta)*cos_angle[0] - cos(theta)*sin_angle[1]*sin_angle[0]), 
                        dist*cos(theta)*cos_angle[1]);
        Vector3f dir = direction + dist / oriDist * direction.length() * offset.normalized();
        dir.normalize();
        r = Ray(offset + position, dir, 1);
        
        // std::cout << offset.x() << ' ' << offset.y() << ' ' << offset.z() << std::endl;
        // std::cout << dir.x() << ' ' << dir.y() <<  ' ' << dir.z() << std::endl;
        p = power / (M_PI * radius * radius) / photonAmount;   // TODO necessary?
    }

    bool intersect(const Ray &r, Hit &h, float tmin)
    {
        return plate->intersect(r, h, tmin);
    }

    Vector3f getColor() const override
    {
        return color;
    }

    int getPhotonAmount() const override
    {
        return photonAmount;
    }

private:

    Vector3f direction;
    float direction_angle[2];
    float sin_angle[2], cos_angle[2];
    Vector3f color;
    Vector3f position;
    float radius;
    float maxAngle;
    float power;
    int photonAmount;
    Plate* plate = nullptr;
};

#endif // LIGHT_H
