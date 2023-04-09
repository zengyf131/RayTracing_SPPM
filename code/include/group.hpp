#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>

// TODO: Implement Group - add data structure to store a list of Object*
class Group : public Object3D {

public:

    Group() {
        objList = std::vector<Object3D*>();
    }

    ~Group() override {
        for (int i = 0; i < objList.size(); i++)
        {
            if (objList.at(i) != nullptr)
            {
                delete objList.at(i);
            }
        }
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        bool isIntersecting = false;
        for (int i = 0; i < (int)objList.size(); i++)
        {
            Object3D* obj = objList.at(i);
            if (obj != nullptr)
            {
                if (obj->intersect(r, h, tmin))
                {
                    isIntersecting = true;
                }
            }
        }
        if (isIntersecting)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void addObject(Object3D *obj) {
        objList.push_back(obj);
    }

    int getGroupSize() {
        return objList.size();
    }

private:

    std::vector<Object3D*> objList;
    
};

#endif
	
