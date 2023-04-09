#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>
#include <random>
#include <iostream>

#include "ray.hpp"
#include "hit.hpp"
#include "image.hpp"

// TODO: Implement Shade function that computes Phong introduced in class.
class Material {
public:

    explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, float s = 0, float dr = 1, float sr = 0, float rr = 0, float ri = 1, Image *tex = nullptr, Image* n_tex = nullptr, bool l = false, Vector3f lc = Vector3f::ZERO) :
            diffuseColor(d_color), specularColor(s_color), shininess(s), diffuseRate(dr), specularRate(sr), refractRate(rr), refractIndex(ri), texture(tex), normal_texture(n_tex), isLight(l), lightColor(lc)
    {
        randomEngine = std::default_random_engine(time(0));
        randomRate = std::uniform_real_distribution<float>(0, 1);
    }

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }

    void photonSimulate(Ray inRay, Hit hit, Ray &reRay, int &reType, int setType = -1, bool dev = false)    // reType: -1 absorb, 0 diffuse, 1 specular, 2 refract
    {
        Vector3f normal = hit.getNormal().normalized();
        Vector3f hitPos = inRay.pointAtParameter(hit.getT());
        Vector3f inDir = inRay.getDirection().normalized();
        float inRefract = inRay.getRefractIndex();
        float dirScale = inRay.getDirection().length();
        Vector3f reDir;
        float rate = randomRate(randomEngine);
        if ((setType == -1 && (rate -= diffuseRate) < 0) || setType == 0)
        {
            reType = 0;
            for (int i = 0; i < 3; i++)
            {
                reDir[i] = (randomRate(randomEngine) * 2) - 1;
            }
            if (Vector3f::dot(reDir, normal) < 0)
            {
                reDir = -reDir;
            }
            reRay = Ray(hitPos, dirScale*reDir.normalized(), inRefract);
        }
        else if ((setType == -1 && (rate -= specularRate) < 0) || setType == 1)
        {
            if (setType == -1 && inRay.getRefractIndex() == refractIndex && refractIndex != 1)
            {
                photonSimulate(inRay, hit, reRay, reType, setType = 2);
                return;
            }
            reType = 1;
            reDir =  inDir - 2 * (Vector3f::dot(inDir, normal)) * normal;
            reRay = Ray(hitPos, reDir, inRefract);
        }
        else if ((setType == -1 && (rate -= refractRate) < 0) || setType == 2)
        {
            
            reType = 2;
            float outRefract = refractIndex;
            // if (inRefract == outRefract && Vector3f::dot(inDir, normal) < 0)
            // {
            //     std::cout << inRefract << ' ' << outRefract << std::endl;
            //     std::cout << inDir.x() << ' ' << inDir.y() << ' ' << inDir.z() << std::endl;
            //     std::cout << normal.x() << ' ' << normal.y() << ' ' << normal.z() << std::endl;
            // }
            if (Vector3f::dot(inDir, normal) > 0)
            {
                outRefract = 1;
                normal = -normal;
            }
            float cosTheta2 = 1 - inRefract * inRefract * (1 - Vector3f::dot(inDir, normal) * Vector3f::dot(inDir, normal)) / (outRefract * outRefract);
            float cosTheta;
            if (cosTheta2 >= 0)
            {
                cosTheta = sqrt(cosTheta2);
            }
            else
            {
                if (setType == 2 && dev)
                {
                    std::cout << "reflect!" << std::endl;
                }
                photonSimulate(inRay, hit, reRay, reType, 1);
                return;
            }
            if (setType == 2 && dev)
            {
                std::cout << inRefract << ' ' << outRefract << ' ' << cosTheta << std::endl;
            }
            reDir = inRefract / outRefract * inDir + (inRefract / outRefract * Vector3f::dot(-inDir, normal) - cosTheta) * normal;
            reRay = Ray(hitPos, reDir, outRefract);
        }
        else
        {
            reType = -1;
        }
    }

    Vector3f Shade(const Ray &ray, const Hit &hit, const Vector3f &dirToLight, const Vector3f &lightColor) 
    {
        Vector3f shaded = Vector3f::ZERO;
        float diffuseIntensity = Vector3f::dot(dirToLight, hit.getNormal());
        if (diffuseIntensity < 0)
        {
            diffuseIntensity = 0;
        }
        Vector3f R = 2 * Vector3f::dot(hit.getNormal(), dirToLight) * hit.getNormal() - dirToLight;
        float specularIntensity = Vector3f::dot(-ray.getDirection(), R);
        if (specularIntensity < 0)
        {
            specularIntensity = 0;
        }
        else
        {
            specularIntensity = pow(specularIntensity, shininess);
        }

        shaded = (diffuseColor * diffuseIntensity) * lightColor;
        return shaded;
    }

    Vector3f Shade(const Vector3f &rayDir, const Hit &hit, const Vector3f &dirToLight, const Vector3f &lightColor) 
    {
        Vector3f shaded = Vector3f::ZERO;
        Vector3f hitNormal = hit.getNormal();
        Vector3f dColor;
        if (texture != nullptr && hit.u != -1 && hit.v != -1)
        {
            dColor = texture->GetPixel(hit.u * texture->Width(), hit.v * texture->Height());
        }
        else
        {
            dColor = diffuseColor;
        }
        float diffuseIntensity = Vector3f::dot(dirToLight, hitNormal);
        if (diffuseIntensity < 0)
        {
            diffuseIntensity = 0;
        }
        Vector3f R = 2 * Vector3f::dot(hitNormal, dirToLight) * hitNormal - dirToLight;
        float specularIntensity = Vector3f::dot(-rayDir, R);
        if (specularIntensity < 0)
        {
            specularIntensity = 0;
        }
        else
        {
            specularIntensity = pow(specularIntensity, shininess);
        }

        shaded = (dColor * diffuseIntensity) * lightColor;

        return shaded;
    }

    float diffuseRate = 0.5;
    float specularRate = 0.2;
    float refractRate = 0;
    float refractIndex = 0;
    bool isLight = false;
    Vector3f lightColor = Vector3f::ZERO;
    Image *texture;
    Image *normal_texture;

protected:
    
    std::default_random_engine randomEngine;
    std::uniform_real_distribution<float> randomRate;
    Vector3f diffuseColor;
    Vector3f specularColor;
    Vector3f embientColor = Vector3f(0.9, 0.9, 0.9);
    float shininess;
    
};


#endif // MATERIAL_H
