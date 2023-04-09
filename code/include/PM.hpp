#ifndef PM_H
#define PM_H

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <time.h>
#include <vector>
#include <string>
#include <algorithm>
#include <queue>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "objects/light.hpp"

using namespace std;

struct Photon
{
    float power;
    Vector3f position;
    Vector3f direction;
    Vector3f color;
};

struct SPPMPixel
{
    Vector3f flux;
    int nearbyPhotonAmount;
    int nearbyPhotonRadius2;
    Vector3f radiance;
    bool firstHit;
    vector<Vector3f> camRayPos;
};

struct BSP_node
{
    BSP_node(Vector3f minp, Vector3f maxp)
    {
        minPoint = minp;
        maxPoint = maxp;
    }
    BSP_node* lc = nullptr, * rc = nullptr;
    bool isLeaf = false;
    int method = -1;
    Vector3f minPoint = Vector3f::ZERO;
    Vector3f maxPoint = Vector3f::ZERO;
    vector<Photon> photonList;
};

int method = 0;
Vector3f camRayPos = Vector3f::ZERO;

struct distCmp
{
    bool operator() (Photon a, Photon b)
    {
        return (a.position - camRayPos).squaredLength() < (b.position - camRayPos).squaredLength();
    }
};

bool photonCmp(const Photon &a, const Photon &b)
{
    return a.position[method] < b.position[method];
}

class SPPM
{
public:

    SceneParser* sceneParser = nullptr;
    Group* baseGroup = nullptr;
    Image* img = nullptr;
    Image* lightMap_img = nullptr;
    BSP_node* root = nullptr;
    priority_queue<Photon, vector<Photon>, distCmp> nearbyPhoton;
    string outputFile;


    int bounceLimit = 6;
    float alpha = 0.7;
    float maxNearbyDist_square = 0;
    int maxNearbyNum = 0;
    float inf = 1000000;
    float tmin = 1.0 / 10000;
    int x, y;

    SPPM(string inFile, string outFile)
    {
        sceneParser = new SceneParser(inFile.c_str());
        img = new Image(sceneParser->getCamera()->getWidth(), sceneParser->getCamera()->getHeight());
        lightMap_img = new Image(sceneParser->getCamera()->getWidth(), sceneParser->getCamera()->getHeight());
        outputFile = outFile;
        method = 0;
        camRayPos = Vector3f::ZERO;

        maxNearbyDist_square = sceneParser->getMaxNearbyDist_square();
        maxNearbyNum = sceneParser->getMaxNearbyPhoton();
    }

    int sameAmount = 0;
    void buildBSP(BSP_node* r, std::vector<Photon> pList, int count)
    {
        if (pList.size() <= 5 || count > 20)
        {
            // cout << pList.size() << endl;
            r->isLeaf = true;
            r->photonList = pList;
            return;
        }

        r->method = method;
        
        sort(pList.begin(), pList.end(), photonCmp);
        // float pos = pList[0].position[method];
        // for (int i = 1; i < (int)pList.size(); i++)
        // {
        //     if (pList[i].position[method] = pos)
        //     {
        //         cout << "no!" << endl;
        //         break;
        //     }
        //     pos = pList[i].position[method];
        // }
        float halfPos;
        int halfSize;
        if (pList.size() % 2 == 0)
        {
            halfSize = pList.size() / 2 - 1;
            halfPos = (pList[halfSize].position[method] + pList[halfSize + 1].position[method]) / 2;
        }
        else
        {
            halfSize = (pList.size() - 1) / 2;
            halfPos = pList[halfSize].position[method];
        }
        Vector3f newMinPoint = r->minPoint, newMaxPoint = r->maxPoint;
        newMinPoint[method] = halfPos;
        newMaxPoint[method] = halfPos;
        BSP_node* lc = new BSP_node(r->minPoint, newMaxPoint);
        BSP_node* rc = new BSP_node(newMinPoint, r->maxPoint);
        r->lc = lc;
        r->rc = rc;
        std::vector<Photon> lcP, rcP;
        
        int splitPos = halfSize + 1;
        for (; splitPos < (int)pList.size(); splitPos++)
        {
            if (pList[splitPos].position[method] != halfPos)
            {
                break;
            }
            sameAmount++;
        }
        lcP.assign(pList.begin(), pList.begin() + splitPos);
        rcP.assign(pList.begin() + splitPos, pList.end());
        // cout << sameAmount << endl;
        // sameAmount = 0;
        // exit(0);
        // for (int i = (pList.size() % 2 == 0) ? (halfSize) : (halfSize - 1); i >= 0; i--)
        // {
        //     if (pList[i].position[method] == halfPos)
        //     {
        //         sameAmount++;
        //         rcP.push_back(pList[i]);
        //     }
        //     else
        //     {
        //         break;
        //     }
        // }
        if (method == 2)
        {
            method = 0;
        }
        else
        {
            method++;
        }
        int nextMethod = method;
        buildBSP(lc, lcP, count + 1);
        method = nextMethod;
        buildBSP(rc, rcP, count + 1);
    }

    void deleteBSP(BSP_node* r)
    {
        if (r->isLeaf)
        {
            delete r;
            return;
        }
        deleteBSP(r->lc);
        deleteBSP(r->rc);
        delete r;
    }

    float findNearbyPhoton(BSP_node* node, float maxDist)
    {
        if (node->isLeaf)
        {
            vector<Photon> &pList = node->photonList;
            for (int i = 0; i < (int)pList.size(); i++)
            {
                if ((pList[i].position - camRayPos).squaredLength() < maxDist)
                {
                    nearbyPhoton.push(pList[i]);
                    if ((int)nearbyPhoton.size() > maxNearbyNum)
                    {
                        nearbyPhoton.pop();
                        maxDist = (nearbyPhoton.top().position - camRayPos).squaredLength();
                    }
                }
            }
            return maxDist;
        }

        float dist = camRayPos[node->method] - node->lc->maxPoint[node->method];
        if (dist < 0)
        {
            maxDist = findNearbyPhoton(node->lc, maxDist);
            if (dist * dist < maxDist)
            {
                maxDist = findNearbyPhoton(node->rc, maxDist);
            }
        }
        else
        {
            maxDist = findNearbyPhoton(node->rc, maxDist);
            if (dist * dist < maxDist)
            {
                maxDist = findNearbyPhoton(node->lc, maxDist);
            }
        }
        return maxDist;
    }

    bool pixelRayTracing(Ray camRay, float throughput, int bounce, Vector3f &Lr)
    {
        Hit hit;
        bool isIntersect = baseGroup->intersect(camRay, hit, tmin);
        // bool isIntersect = false;
        // if (y == 70 && x <= 40)
        // {
        //     isIntersect = baseGroup->intersect(camRay, hit, tmin);
        // }
        if (!isIntersect) 
        {
            return false;
        }
        // if (bounce == 3 && y == 33 && x >= 160)
        // {
        //     cout << hit.getT() << endl;
        // }
        Material* material = hit.getMaterial();
        if (material->diffuseRate > 0)
        {
            Vector3f flux = Vector3f::ZERO;
            camRayPos = camRay.pointAtParameter(hit.getT());
            float maxDist = findNearbyPhoton(root, maxNearbyDist_square);
            int nearbyPhotonAmount = (int)nearbyPhoton.size();
            for (int i = 0; i < nearbyPhotonAmount; i++)
            {
                Photon p = nearbyPhoton.top();
                Vector3f f = material->Shade(camRay, hit, -p.direction, p.color);
                // cout << f.x() << ' ' << f.y() << ' ' << f.z() << endl;
                flux += f * p.power;
                nearbyPhoton.pop();
            }
            // cout << flux.x() << ' ' << flux.y() << ' ' << flux.z() << endl;
            Vector3f thisLr = flux / (M_PI * maxDist);
            Lr += throughput * material->diffuseRate * thisLr;
            if (material->isLight)
            {
                Lr += material->lightColor;
            }
            for (int i = 0; i < 3; i++)
            {
                if (Lr[i] > 1) Lr[i] = 1;
            }
            
            // cout << Lr.x() << ' ' << Lr.y() << ' ' << Lr.z() << endl;
            if (bounce == 1 && nearbyPhotonAmount > 0)
            {
                lightMap_img->SetPixel(x, y, Vector3f(1, 1, 1)*(float)nearbyPhotonAmount/maxNearbyNum*10);
                // cout << 1/maxDist/100 << endl;
                // img->SetPixel(x, y, Vector3f(1, 1, 1)/maxDist/100);
            }
            
        }
        
        if (material->specularRate > 0 && bounce < bounceLimit)
        {
            Ray reRay(Vector3f::ZERO, Vector3f::ZERO, 1);
            int reType;
            hit.getMaterial()->photonSimulate(camRay, hit, reRay, reType, 1);
            pixelRayTracing(reRay, throughput * material->specularRate, bounce + 1, Lr);
        }
        
        if (material->refractRate > 0 && bounce < bounceLimit)
        {
            Ray reRay(Vector3f::ZERO, Vector3f::ZERO, 1);
            int reType;
            // if (bounce == 2 && y == 33 && x >= 160)
            // {
            //     cout << "begin" << endl;
            //     hit.getMaterial()->photonSimulate(camRay, hit, reRay, reType, 2, true);
            //     cout << x << endl;
            //     // cout << Lr.x() << ' ' << Lr.y() << ' ' << Lr.z() << endl;
            //     cout << camRay.getOrigin().x() << ' ' << camRay.getOrigin().y() << ' ' << camRay.getOrigin().z() << endl;
            //     cout << camRay.getDirection().x() << ' ' << camRay.getDirection().y() << ' ' << camRay.getDirection().z() << endl;
            //     cout << reRay.getOrigin().x() << ' ' << reRay.getOrigin().y() << ' ' << reRay.getOrigin().z() << endl;
            //     cout << reRay.getDirection().x() << ' ' << reRay.getDirection().y() << ' ' << reRay.getDirection().z() << endl;
            //     // cout << hit.getT() << endl;
            //     // cout << camRay.pointAtParameter(hit.getT()).x() << ' ' << camRay.pointAtParameter(hit.getT()).y() << ' ' << camRay.pointAtParameter(hit.getT()).z() << endl;
            //     // cout << camRayPos.x() << ' ' << camRayPos.y() << ' ' << camRayPos.z() << endl;
            //     cout << endl;
            // }
            // else
            // {
            hit.getMaterial()->photonSimulate(camRay, hit, reRay, reType, 2);
            // }
            pixelRayTracing(reRay, throughput * material->refractRate, bounce + 1, Lr);
        }
        return true;
    }

    void photonMapping()
    {
        baseGroup = sceneParser->getGroup();
        vector<Photon> photonList;
        
        cout << "Begin photon mapping..." << endl;
        Vector3f minPoint(inf, inf, inf), maxPoint(-inf, -inf, -inf);
        for (int lightId = 0; lightId < sceneParser->getNumLights(); lightId++)
        {
            Light* light = sceneParser->getLight(lightId);
            for (int i = 0; i < light->getPhotonAmount(); i++)
            {
                Ray lightRay(Vector3f::ZERO, Vector3f::ZERO, 1);
                float power;
                light->getPhoton(lightRay, power);
                int bounce = 0;
                while (bounce < bounceLimit)
                {
                    Hit hit;
                    Ray reRay(Vector3f::ZERO, Vector3f::ZERO, 1);
                    int reType;
                    bool isIntersect = baseGroup->intersect(lightRay, hit, tmin);
                    if (!isIntersect) break;
                    hit.getMaterial()->photonSimulate(lightRay, hit, reRay, reType);
                    if (reType == -1)  // absorb
                    {
                        break;
                    }
                    else
                    {
                        if (reType == 0)    // diffuse
                        {
                            Photon p;
                            p.position = lightRay.pointAtParameter(hit.getT());
                            p.direction = lightRay.getDirection();
                            p.power = power;
                            p.color = light->getColor();
                            photonList.push_back(p);
                            for (int j = 0; j < 3; j++)
                            {
                                if (p.position[j] < minPoint[j]) minPoint[j] = p.position[j];
                                if (p.position[j] > maxPoint[j]) maxPoint[j] = p.position[j];
                            }
                        }
                        lightRay = reRay;
                    }
                    bounce++;
                }
            }
        }
        cout << photonList.size() << endl;
        root = new BSP_node(minPoint, maxPoint);
        buildBSP(root, photonList, 0); 
        cout << sameAmount << endl;
        cout << "Photon mapping done." << endl;
        cout << "Begin ray tracing..." << endl;
        for (x = 0; x < sceneParser->getCamera()->getWidth(); ++x) 
        {
            for (y = 0; y < sceneParser->getCamera()->getHeight(); ++y) 
            {
                Vector3f Lr = Vector3f::ZERO;
                Ray camRay = sceneParser->getCamera()->generateRay(Vector2f(x, y));
                bool hasIntersect = pixelRayTracing(camRay, 1, 1, Lr);
                if (hasIntersect)
                {
                    // if (y == 29 && x % 5 == 0 && x >= 120 && x <= 165)
                    // {
                    //     // cout << x << ',' << y << ':' << Lr.x() << ' ' << Lr.y() << ' ' << Lr.z() << endl;
                    //     img->SetPixel(x, y, Vector3f(0,1,1));
                    //     continue;
                    // }
                    img->SetPixel(x, y, Lr);
                }
                else
                {
                    img->SetPixel(x, y, sceneParser->getBackgroundColor());
                }
            }
            if (x % 10 == 0) img->SaveImage(outputFile.c_str());
        }
        cout << "Ray tracing done." << endl;
        lightMap_img->SaveImage((outputFile.substr(0, outputFile.length() - 4) + "_lightMap.bmp").c_str());
        img->SaveImage(outputFile.c_str());
    }
};

#endif