#ifndef SPPM_H
#define SPPM_H

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
#include <stack>
#include <omp.h>
#include <functional>

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

struct CamWatchPoint
{
    float throughput;
    Vector3f pos;
    Vector3f dir;
    Material* material;
    Vector3f normal;
    Hit hit;
};

struct SPPMPixel
{
    Vector3f flux = Vector3f::ZERO;
    int nearbyPhotonAmount = 0;
    float nearbyPhotonRadius2 = 0;
    Vector3f radiance = Vector3f::ZERO;
    bool firstHit = true;
    float x = 0, y = 0, length = 1;
    vector<CamWatchPoint> watchPoints;
};

struct Pixel_node
{
    bool isLeaf = true;
    Pixel_node *child[4];
    Pixel_node *parent = nullptr;
    SPPMPixel *pixel;
    int treeHeight = 1;
    int pixelCount = 1;
    float depth = 0;
    bool firstIntersect = false;

    Pixel_node(Pixel_node *p = nullptr)
    {
        pixel = new SPPMPixel();
        parent = p;
    }

    void splitNode()
    {
        if (!isLeaf) return;
        isLeaf = false;
        Pixel_node *node = this;
        while (node != nullptr)
        {
            node->treeHeight++;
            node->pixelCount += 3;
            node = node->parent;
        }
        for (int i = 0; i < 4; i++)
        {
            child[i] = new Pixel_node(this);
            child[i]->pixel->flux = pixel->flux;
            child[i]->pixel->nearbyPhotonAmount = pixel->nearbyPhotonAmount;
            child[i]->pixel->nearbyPhotonRadius2 = pixel->nearbyPhotonRadius2;
            child[i]->pixel->radiance = pixel->radiance;
            child[i]->pixel->firstHit = pixel->firstHit;
            child[i]->pixel->length = pixel->length / 2;
            child[i]->pixel->x = pixel->x + (i % 2) * pixel->length / 2;
            child[i]->pixel->y = pixel->y + (i / 2) * pixel->length / 2;
        }
    }

    void radianceIterate(Vector3f &Lr, int currentIterate)
    {
        if (isLeaf)
        {
            pixel->radiance = pixel->flux / (M_PI * pixel->nearbyPhotonRadius2) / (currentIterate + 1);
            Lr += pixel->radiance;
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                child[i]->radianceIterate(Lr, currentIterate);
            }
            pixel->radiance = Lr;
        }
    }

    float depthIterate()
    {
        if (isLeaf)
        {
            firstIntersect = false;
            return depth;
        }
        else
        {
            depth = child[0]->depthIterate();
            for (int i = 1; i < 4; i++)
            {
                float d = child[i]->depthIterate();
                if (depth > d) depth = d;
            }
            return depth;
        }
    }
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

// int method = 0;
// Vector3f camRayPos = Vector3f::ZERO;



// bool photonCmp(const Photon &a, const Photon &b)
// {
//     return a.position[method] < b.position[method];
// }

static omp_lock_t lock;

class SPPM
{
public:

    SceneParser* sceneParser = nullptr;
    Group* baseGroup = nullptr;
    Image* img = nullptr;
    Image* img_DOF = nullptr;
    Image* treeMap_img = nullptr;
    Image* lightMap_img;
    BSP_node* root = nullptr;
    string outputFile;
    Pixel_node ***imgPixels;
    

    int bounceLimit = 10;
    float bounceThres = 0.01;
    float splitThres = 0.1;
    int pixelTreeLimit = 4;
    float alpha = 0.7;
    float inf = 1000000;
    float tmin = 1.0 / 1000;

    float maxNearbyDist_square = 0;
    int maxNearbyNum = 0;
    int sppm_iteration = 0;
    int currentIterate = 0;
    bool enableDOF = true;
    float focusDist = 4;
    float focusRange = 1;
    int pixelNum[2];
    // int x, y;

    SPPM(string inFile, string outFile)
    {
        sceneParser = new SceneParser(inFile.c_str());

        outputFile = outFile;
        // method = 0;
        // camRayPos = Vector3f::ZERO;
        maxNearbyDist_square = sceneParser->getMaxNearbyDist_square();
        maxNearbyNum = sceneParser->getMaxNearbyPhoton();
        sppm_iteration = sceneParser->getSPPM_iteration();
        pixelNum[0] = sceneParser->getCamera()->getWidth();
        pixelNum[1] = sceneParser->getCamera()->getHeight();

        baseGroup = sceneParser->getGroup();
        img = new Image(pixelNum[0], pixelNum[1]);
        img_DOF = new Image(pixelNum[0], pixelNum[1]);
        treeMap_img = new Image(pixelNum[0], pixelNum[1]);
        lightMap_img = new Image(pixelNum[0], pixelNum[1]);
        imgPixels = new Pixel_node**[pixelNum[0]];
        for (int i = 0; i < pixelNum[0]; i++)
        {
            imgPixels[i] = new Pixel_node*[pixelNum[1]];
            for (int j = 0; j < pixelNum[1]; j++)
            {
                imgPixels[i][j] = new Pixel_node();
                imgPixels[i][j]->pixel->x = i;
                imgPixels[i][j]->pixel->y = j;
                imgPixels[i][j]->pixel->nearbyPhotonRadius2 = maxNearbyDist_square;
            }
        }
    }

    bool pixelRayTracing(Ray camRay, float throughput, int bounce, bool foundDiffuse, Pixel_node *node)
    {
        Hit hit;
        bool isIntersect = baseGroup->intersect(camRay, hit, tmin);
        if (!isIntersect) 
        {
            return false;
        }
        if (!node->firstIntersect)
        {
            node->depth = (camRay.pointAtParameter(hit.getT()) - camRay.getOrigin()).length();
            node->firstIntersect = true;
        }
        Material* material = hit.getMaterial();
        if (material->diffuseRate > 0)
        {
            CamWatchPoint p;
            p.pos = camRay.pointAtParameter(hit.getT());
            // cout << p.pos.x() << ' ' << p.pos.y() << ' ' << p.pos.z() << endl;
            p.dir = camRay.getDirection();
            p.throughput = throughput * material->diffuseRate;
            p.material = material;
            p.normal = hit.getNormal();
            p.hit = hit;
            omp_set_lock(&lock);
            node->pixel->watchPoints.push_back(p);
            omp_unset_lock(&lock);
            foundDiffuse = true;
        }
        // if (x == 180 && y >= 75 && y <= 85)
        // {
        //     cout << x << ' ' << y << endl;
        //     cout << hit.getT() << endl;
        //     cout << throughput << endl;
        //     cout << camRay.pointAtParameter(hit.getT()).x() << ' ' << camRay.pointAtParameter(hit.getT()).y() << ' ' << camRay.pointAtParameter(hit.getT()).z() << endl;
        //     cout << camRay.getDirection().x() << ' ' << camRay.getDirection().y() << ' ' << camRay.getDirection().z() << endl;
        // }
        bool continueBounce = (bounce < bounceLimit || (!foundDiffuse && (material->specularRate*material->refractRate == 0 || bounce < bounceLimit * 2)));
        if (material->specularRate * throughput > bounceThres && continueBounce)
        {
            Ray reRay(Vector3f::ZERO, Vector3f::ZERO, 1);
            int reType;
            hit.getMaterial()->photonSimulate(camRay, hit, reRay, reType, 1);
            pixelRayTracing(reRay, throughput * material->specularRate, bounce + 1, foundDiffuse, node);
        }
        if (material->refractRate * throughput > bounceThres && continueBounce)
        {
            Ray reRay(Vector3f::ZERO, Vector3f::ZERO, 1);
            int reType;
            // if (x == 180 && y >= 75 && y <= 85)
            // {
            //     hit.getMaterial()->photonSimulate(camRay, hit, reRay, reType, 2);
            // }
            // else
            // {
            hit.getMaterial()->photonSimulate(camRay, hit, reRay, reType, 2);
            // }
            pixelRayTracing(reRay, throughput * material->refractRate, bounce + 1, foundDiffuse, node);

        }
        return true;
    }

    void rayTracing()
    {
        cout << "Begin ray tracing..." << endl;
        int pixelCount = 0;
        
        #pragma omp parallel for num_threads(15)
        for (int x = 0; x < pixelNum[0]; x++) 
        {
            // cout << endl << x << endl;
            for (int y = 0; y < pixelNum[1]; y++) 
            {
                if (imgPixels[x][y]->isLeaf)
                {
                    pixelCount++;
                    Ray camRay = sceneParser->getCamera()->generateRay(Vector2f(imgPixels[x][y]->pixel->x, imgPixels[x][y]->pixel->y), imgPixels[x][y]->pixel->length);
                    pixelRayTracing(camRay, 1, 1, false, imgPixels[x][y]);
                    continue;
                }

                stack<Pixel_node*> nodeStack;
                for (int i = 0; i < 4; i++)
                {
                    nodeStack.push(imgPixels[x][y]->child[i]);
                }
                while (!nodeStack.empty())
                {
                    Pixel_node *node = nodeStack.top();
                    nodeStack.pop();
                    if (node->isLeaf)
                    {
                        // cout << node->pixel->x << ' ' << node->pixel->y << endl;
                        pixelCount++;
                        Ray camRay = sceneParser->getCamera()->generateRay(Vector2f(node->pixel->x, node->pixel->y), node->pixel->length);
                        pixelRayTracing(camRay, 1, 1, false, node);
                    }
                    else
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            nodeStack.push(node->child[i]);
                        }
                    }
                }
                // cout << y << ' ';
            }
            
        }
        
        cout << pixelCount << endl;
        cout << "Ray tracing done." << endl;
    }

    void buildBSP(BSP_node* r, std::vector<Photon> pList, int method, int count)
    {
        if (pList.size() <= 5 || count > 20)
        {
            // cout << pList.size() << endl;
            r->isLeaf = true;
            r->photonList = pList;
            return;
        }

        r->method = method;
        auto cmp = [method] (const Photon &a, const Photon &b)
        {
            return a.position[method] < b.position[method];
        };
        
        sort(pList.begin(), pList.end(), cmp);
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
        }
        lcP.assign(pList.begin(), pList.begin() + splitPos);
        rcP.assign(pList.begin() + splitPos, pList.end());
        if (method == 2)
        {
            method = 0;
        }
        else
        {
            method++;
        }
        int nextMethod = method;
        buildBSP(lc, lcP, method, count + 1);
        method = nextMethod;
        buildBSP(rc, rcP, method, count + 1);
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

    void photonMapping()
    {
        vector<Photon> photonList;
        
        cout << "Begin photon mapping..." << endl;
        Vector3f minPoint(inf, inf, inf), maxPoint(-inf, -inf, -inf);
        for (int lightId = 0; lightId < sceneParser->getNumLights(); lightId++)
        {
            Light* light = sceneParser->getLight(lightId);
            #pragma omp parallel for num_threads(15)
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
                            omp_set_lock(&lock);
                            photonList.push_back(p);
                            omp_unset_lock(&lock);
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
        buildBSP(root, photonList, 0, 0); 
        cout << "Photon mapping done." << endl;
    }

    float findNearbyPhoton(BSP_node* node, float maxDist, Vector3f camRayPos, function<bool(Photon &a, Photon &b)> cmp, priority_queue<Photon, vector<Photon>, decltype(cmp)> &nearbyPhoton)
    {
        if (node->isLeaf)
        {
            vector<Photon> &pList = node->photonList;
            // cout << pList.size() << endl;
            for (int i = 0; i < (int)pList.size(); i++)
            {
                // cout << "here" << endl;
                // cout << pList[i].position.x() << ' ' << pList[i].position.y() << ' ' << pList[i].position.z() << endl;
                // cout << camRayPos.x() << ' ' << camRayPos.y() << ' ' << camRayPos.z() << endl;
                if ((pList[i].position - camRayPos).squaredLength() < maxDist)
                {
                    // cout << "here" << endl;
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
            maxDist = findNearbyPhoton(node->lc, maxDist, camRayPos, cmp, nearbyPhoton);
            if (dist * dist < maxDist)
            {
                maxDist = findNearbyPhoton(node->rc, maxDist, camRayPos, cmp, nearbyPhoton);
            }
        }
        else
        {
            maxDist = findNearbyPhoton(node->rc, maxDist, camRayPos, cmp, nearbyPhoton);
            if (dist * dist < maxDist)
            {
                maxDist = findNearbyPhoton(node->lc, maxDist, camRayPos, cmp, nearbyPhoton);
            }
        }
        return maxDist;
    }

    void radianceEstimate_calc(SPPMPixel *pixel)
    {
        for (int watchNum = 0; watchNum < (int)pixel->watchPoints.size(); watchNum++)
        {
            CamWatchPoint point = pixel->watchPoints[watchNum];
            Vector3f flux = Vector3f::ZERO;
            Vector3f camRayPos = point.pos;
            // cout << camRayPos.x() << ' ' << camRayPos.y() << ' ' << camRayPos.z() << endl;
            function<bool(Photon &a, Photon &b)> cmp = [camRayPos] (Photon &a, Photon &b)->bool
            {
                return (a.position - camRayPos).squaredLength() < (b.position - camRayPos).squaredLength();
            };
            priority_queue<Photon, vector<Photon>, decltype(cmp)> nearbyPhoton(cmp);
            float maxDist = findNearbyPhoton(root, pixel->nearbyPhotonRadius2, camRayPos, cmp, nearbyPhoton);            
            int nearbyPhotonAmount = (int)nearbyPhoton.size();
            for (int i = 0; i < nearbyPhotonAmount; i++)
            {
                Photon p = nearbyPhoton.top();
                Vector3f f = point.material->Shade(point.dir, point.hit, -p.direction, p.color);
                flux += f * p.power;
                nearbyPhoton.pop();
            }
            pixel->flux += flux * point.throughput;
            if (point.material->isLight)
            {
                pixel->flux += point.material->lightColor;
            }
            if (pixel->firstHit)
            {
                pixel->firstHit = false;
                pixel->nearbyPhotonRadius2 = maxDist;
                pixel->nearbyPhotonAmount = nearbyPhotonAmount;
                lightMap_img->SetPixel(pixel->x, pixel->y, Vector3f(1, 1, 1)*nearbyPhotonAmount/maxDist/50000);
                // cout << maxDist << endl;
            }
            else
            {
                int new_nearbyPhotonAmount = pixel->nearbyPhotonAmount + alpha * nearbyPhotonAmount;
                float scale = (float)new_nearbyPhotonAmount / (pixel->nearbyPhotonAmount + nearbyPhotonAmount);
                pixel->nearbyPhotonRadius2 = pixel->nearbyPhotonRadius2 * scale;
                pixel->flux = pixel->flux * scale;
                pixel->nearbyPhotonAmount = new_nearbyPhotonAmount;
            }
        }
    }

    void radianceEstimate()
    {
        cout << "Begin radiance estimate..." << endl;
        #pragma omp parallel for num_threads(15)
        for (int x = 0; x < pixelNum[0]; x++) 
        {
            for (int y = 0; y < pixelNum[1]; y++) 
            {
                Pixel_node *node = imgPixels[x][y];
                node->depthIterate();
                if (node->isLeaf)
                {
                    radianceEstimate_calc(node->pixel);
                    node->pixel->watchPoints.clear();
                    continue;
                }

                stack<Pixel_node*> nodeStack;
                for (int i = 0; i < 4; i++)
                {
                    nodeStack.push(imgPixels[x][y]->child[i]);
                }
                while (!nodeStack.empty())
                {
                    node = nodeStack.top();
                    nodeStack.pop();
                    if (node->isLeaf)
                    {
                        radianceEstimate_calc(node->pixel);
                        node->pixel->watchPoints.clear();
                    }
                    else
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            nodeStack.push(node->child[i]);
                        }
                    }
                }
            }
        }
        cout << "Radiance estimate done." << endl;
    }

    void pixelPaint()
    {
        img->SetAllPixels(Vector3f(0, 0, 0));
        for (int x = 0; x < pixelNum[0]; x++)
        {
            for (int y = 0; y < pixelNum[1]; y++)
            {
                Pixel_node* p = imgPixels[x][y];
                Vector3f Lr = Vector3f::ZERO;
                p->radianceIterate(Lr, currentIterate);
                Lr = Lr / p->pixelCount;
                for (int i = 0; i < 3; i++)
                {
                    if (Lr[i] > 1) Lr[i] = 1;
                }
                img->SetPixel(x, y, Lr);
                // if (x >= 40 && x <= 80 && y >= 20 && y <= 70 && y % 5 == 0 && x % 5 == 0)
                // {
                //     img->SetPixel(x, y, Vector3f(0, 1, 1));
                // }
                // cout << pixel->watchPointNum << ' ';
            }
            // cout << endl;
        }
        if (enableDOF)
        {
            for (int x = 0; x < pixelNum[0]; x++)
            {
                for (int y = 0; y < pixelNum[1]; y++)
                {
                    Pixel_node* p = imgPixels[x][y];
                    Vector3f color = img->GetPixel(x, y);
                    float coc = (p->depth - focusDist) / focusRange;
                    if (coc < 0) coc = -coc;
                    if (coc > (float)pixelNum[0] / 50) coc = (float)pixelNum[0] / 50;
                    float num = 1;
                    for (int i = x - coc; i <= x + 1 + coc; i++)
                    {
                        for (int j = y - coc; j <= y + 1 + coc; j++)
                        {
                            float dist2 = (i - x) * (i - x) + (j - y) * (j - y);
                            if (dist2 <= (coc + 1) * (coc + 1) && i >= 0 && i < pixelNum[0] && j >= 0 && j < pixelNum[1] && (i != x || j != y))
                            {
                                Vector3f c = img->GetPixel(i, j);
                                if (coc < 1)
                                {
                                    color += coc * c;
                                    num += coc;
                                }
                                else
                                {
                                    color += c;
                                    num++;
                                }
                                
                            }
                        }
                    }
                    img_DOF->SetPixel(x, y, color / num);
                }
            }
        }
    }

    void pixelSubdivide()
    {
        cout << "Begin pixels subdivide..." << endl;
        int splitNum = 0;
        for (int x = 0; x < pixelNum[0]; x++)
        {
            for (int y = 0; y < pixelNum[1]; y++)
            {
                Pixel_node *n = imgPixels[x][y];
                if (n->isLeaf)
                {
                    Vector3f radiance = n->pixel->radiance;
                    float squareError = 0;
                    int num = 0;
                    for (int i = -1; i <= 1; i++)
                    {
                        for (int j = -1; j <= 1; j++)
                        {
                            int nx = x + i; int ny = y + j;
                            if (nx >= 0 && nx < pixelNum[0] && ny >= 0 && ny < pixelNum[1] && (nx != x || ny != y))
                            {
                                squareError += (imgPixels[nx][ny]->pixel->radiance - radiance).squaredLength();
                                num++;
                            }
                        }
                    }
                    squareError = squareError / num;
                    // if (x >= 40 && x <= 80 && y >= 20 && y <= 70)
                    // {
                    //     cout << x << ' ' << y << ':' << squareError << endl;
                    // }
                    if (squareError > splitThres && n->treeHeight < pixelTreeLimit)
                    {
                        splitNum++;
                        n->splitNode();
                    }
                }
                else
                {
                    stack<Pixel_node*> nodeStack;
                    for (int i = 0; i < 4; i++)
                    {
                        nodeStack.push(imgPixels[x][y]->child[i]);
                    }
                    while (!nodeStack.empty())
                    {
                        Pixel_node *node = nodeStack.top();
                        nodeStack.pop();
                        if (node->isLeaf)
                        {
                            Vector3f radiance = node->pixel->radiance;
                            float squareError = 0;
                            for (int i = 0; i < 4; i++)
                            {
                                Pixel_node *sibb = node->parent->child[i];
                                if (sibb != node)
                                {
                                    squareError += (sibb->pixel->radiance - radiance).squaredLength();
                                }
                            }
                            squareError = squareError / 3;
                            // if (x >= 40 && x <= 80 && y >= 20 && y <= 70)
                            // {
                            //     cout << x << ' ' << y << ':' << squareError << endl;
                            // }
                            if (squareError > splitThres && imgPixels[x][y]->treeHeight < pixelTreeLimit)
                            {
                                splitNum++;
                                node->splitNode();
                            }
                        }
                        else
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                nodeStack.push(node->child[i]);
                            }
                        }
                    }
                }
                treeMap_img->SetPixel(x, y, Vector3f(1, 1, 1) * (1 - (float)n->treeHeight / pixelTreeLimit));          
            }
        }
        cout << splitNum << endl;
        cout << "Pixels subdivide done." << endl;
    }

    void rayIterate()
    {
        omp_init_lock(&lock);
        for (currentIterate = 0; currentIterate < sppm_iteration; currentIterate++)
        {
            double start = omp_get_wtime( );
            cout << "Iterate " << currentIterate << ":" << endl;

            rayTracing();
            photonMapping();
            radianceEstimate();
            pixelPaint();
            pixelSubdivide();

            img->SaveImage(outputFile.c_str());
            img_DOF->SaveImage((outputFile.substr(0, outputFile.length() - 4) + "_DOF.bmp").c_str());
            treeMap_img->SaveImage((outputFile.substr(0, outputFile.length() - 4) + "_treeMap.bmp").c_str());
            lightMap_img->SaveImage((outputFile.substr(0, outputFile.length() - 4) + "_lightMap.bmp").c_str());
            deleteBSP(root);

            double end = omp_get_wtime( );
            std::cout << "Iterate " << currentIterate << " done in " << end - start << "s." << std::endl;
        }
        omp_destroy_lock(&lock);
    }
};

#endif