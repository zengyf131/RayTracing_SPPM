#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>
#include <random>


class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point, const float &pixelLength) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }
    Vector3f getCenter() const { return center; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

// TODO: Implement Perspective camera
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, float angle) : Camera(center, direction, up, imgW, imgH) {
        fx = imgW / (2 * tan(angle / 2));
        fy = imgH / (2 * tan(angle / 2));
        randomEngine = std::default_random_engine(time(0));
        randomPos = std::uniform_real_distribution<float>(0, 1);
    }

    Ray generateRay(const Vector2f &point, const float &pixelLength) override {
        Matrix3f R(horizontal, -up, direction);
        Vector3f dirRC((point.x() + randomPos(randomEngine)*pixelLength - getWidth()/2) / fx, (getHeight()/2 - point.y() - randomPos(randomEngine)*pixelLength) / fy, 1);
        dirRC = (R*dirRC).normalized();
        Ray r(center, dirRC, 1);
        return r;
    }

private:
    
    float fx, fy;
    std::default_random_engine randomEngine;
    std::uniform_real_distribution<float> randomPos;

};

#endif //CAMERA_H
