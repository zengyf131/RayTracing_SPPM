#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "box.hpp"
#include "Vector2f.h"
#include "Vector3f.h"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>

struct TriangleIndex {
    TriangleIndex() {
        x[0] = 0; x[1] = 0; x[2] = 0;
    }
    int &operator[](const int i) { return x[i]; }
    // By Computer Graphics convention, counterclockwise winding is front face
    int x[3]{};
};

class Mesh : public Object3D {

public:

    struct BSP_node
    {
        BSP_node(Vector3f minp, Vector3f maxp)
        {
            minPoint = minp;
            maxPoint = maxp;
        }
        BSP_node* lc = nullptr, * rc = nullptr;
        bool isLeaf = false;
        Vector3f minPoint = Vector3f::ZERO;
        Vector3f maxPoint = Vector3f::ZERO;
        std::vector<TriangleIndex> tList;
        std::vector<Vector3f> nList;
    };

    Mesh(const char *filename, Material *m);
    Mesh(std::vector<Vector3f> &_v, std::vector<TriangleIndex> &_t, Vector3f minPoint, Vector3f maxPoint, Material *m);
    ~Mesh();
    

    BSP_node* root = nullptr;
    std::vector<Vector3f> v;
    std::vector<TriangleIndex> t;
    std::vector<Vector3f> n;
    std::vector<Vector3f> vn;
    bool intersect(const Ray &r, Hit &h, float tmin) override;
    bool BSP_intersect(const Ray &r, Hit &h, float tmin, BSP_node* node);
    float inf = 1000000;
    bool fromCurve = false;

private:

    // Normal can be used for light estimation
    void computeNormal();
    void buildBSP(BSP_node* r, std::vector<TriangleIndex> tList, std::vector<Vector3f> nList, int method, int count);
    void deleteBSP(BSP_node* r);
};

Mesh::Mesh(const char *filename, Material *material) : Object3D(material) {
    // Optional: Use tiny obj loader to replace this simple one.
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    std::string line;
    std::string vTok("v");
    std::string fTok("f");
    std::string texTok("vt");
    char bslash = '/', space = ' ';
    std::string tok;
    int texID;
    Vector3f minPoint(inf, inf, inf);
    Vector3f maxPoint(-inf, -inf, -inf);
    while (true) {
        std::getline(f, line);
        if (f.eof()) {
            break;
        }
        if (line.size() < 3) {
            continue;
        }
        if (line.at(0) == '#') {
            continue;
        }
        std::stringstream ss(line);
        ss >> tok;
        if (tok == vTok) {
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            v.push_back(vec);
            for (int i = 0; i < 3; i++)
            {
                if (vec[i] < minPoint[i])
                {
                    minPoint[i] = vec[i];
                }
                if (vec[i] > maxPoint[i])
                {
                    maxPoint[i] = vec[i];
                }
            }
        } else if (tok == fTok) {
            if (line.find(bslash) != std::string::npos) {
                std::replace(line.begin(), line.end(), bslash, space);
                std::stringstream facess(line);
                TriangleIndex trig;
                facess >> tok;
                for (int ii = 0; ii < 3; ii++) {
                    facess >> trig[ii] >> texID;
                    trig[ii]--;
                }
                t.push_back(trig);
            } else {
                TriangleIndex trig;
                for (int ii = 0; ii < 3; ii++) {
                    ss >> trig[ii];
                    trig[ii]--;
                }
                t.push_back(trig);
            }
        } else if (tok == texTok) {
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
        }
    }
    computeNormal();
    f.close();
    root = new BSP_node(minPoint, maxPoint);
    buildBSP(root, t, n, 0, 0);
}

Mesh::Mesh(std::vector<Vector3f> &_v, std::vector<TriangleIndex> &_t, Vector3f minPoint, Vector3f maxPoint, Material *m) : Object3D(m)
{
    v = _v;
    t = _t;
    computeNormal();
    root = new BSP_node(minPoint, maxPoint);
    buildBSP(root, t, n, 0, 0);
}

Mesh::~Mesh()
{
    deleteBSP(root);
}

void Mesh::computeNormal() {
    n.resize(t.size());
    vn.resize(v.size());
    for (int i = 0; i < (int)vn.size(); i++)
    {
        vn[i] = Vector3f::ZERO;
    }
    for (int triId = 0; triId < (int) t.size(); ++triId) {
        TriangleIndex& triIndex = t[triId];
        Vector3f a = v[triIndex[1]] - v[triIndex[0]];
        Vector3f b = v[triIndex[2]] - v[triIndex[0]];
        b = Vector3f::cross(a, b);
        n[triId] = b / b.length();

        for (int i = 0; i < 3; i++)
        {
            vn[t[triId][i]] += n[triId];
        }
    }
    for (int i = 0; i < (int)vn.size(); i++)
    {
        vn[i].normalize();
    }
}

bool Mesh::intersect(const Ray &r, Hit &h, float tmin) 
{
    Hit oriHit = h;
    if (BSP_intersect(r, h, tmin, root))
    {
        return true;
    }
    else
    {
        h = oriHit;
        return false;
    }
}

bool Mesh::BSP_intersect(const Ray &r, Hit &h, float tmin, BSP_node* node)
{
    bool result = false;
    if (node->isLeaf)
    {
        for (int triId = 0; triId < (int) node->tList.size(); ++triId) 
        {
            TriangleIndex& triIndex = node->tList[triId];
            Triangle triangle(v[triIndex[0]], v[triIndex[1]], v[triIndex[2]], material, false, vn[triIndex[0]], vn[triIndex[1]], vn[triIndex[2]]);
            triangle.normal = node->nList[triId];
            bool res = triangle.intersect(r, h, tmin);
            if (fromCurve && res)
            {
                // cout << "mesh: " << h.getT() << endl;
                Vector3f hitPos = r.pointAtParameter(h.getT());
                float dist = inf;
                for (int i = 0; i < 3; i++)
                {
                    // float thisDist = (triangle.vertices[i] - hitPos).squaredLength();
                    // if (thisDist < dist)
                    // {
                    //     hitPos = triangle.vertices[i];
                    //     dist = thisDist;
                    // }
                    h.hitPos.push_back(triangle.vertices[i]);
                }
                h.hitT.push_back(h.getT());
                // cout << (hitPos - r.pointAtParameter(h.getT())).length() << endl;
            }
            result |= res;
        }
        return result;
    }
    Box lcBox(node->lc->minPoint, node->lc->maxPoint, true, material);
    Box rcBox(node->rc->minPoint, node->rc->maxPoint, true, material);
    if (lcBox.intersect(r, h, tmin))
    {
        result |= BSP_intersect(r, h, tmin, node->lc);
    }
    if (rcBox.intersect(r, h, tmin))
    {
        result |= BSP_intersect(r, h, tmin, node->rc);
    }
    return result;
}

void Mesh::buildBSP(BSP_node* r, std::vector<TriangleIndex> tList, std::vector<Vector3f> nList, int method, int count)
{
    if (tList.size() <= 5 || count > 10)
    {
        r->isLeaf = true;
        r->tList = tList;
        r->nList = nList;
        return;
    }

    Vector3f newMinPoint = r->minPoint, newMaxPoint = r->maxPoint;
    float halfPos = r->minPoint[method] + (r->maxPoint[method] - r->minPoint[method]) / 2;
    newMinPoint[method] = halfPos;
    newMaxPoint[method] = halfPos;
    BSP_node* lc = new BSP_node(r->minPoint, newMaxPoint);
    BSP_node* rc = new BSP_node(newMinPoint, r->maxPoint);
    r->lc = lc;
    r->rc = rc;
    std::vector<TriangleIndex> lcT, rcT;
    std::vector<Vector3f> lcN, rcN;
    for (int i = 0; i < (int)tList.size(); i++)
    {
        float minPos = v[tList[i][0]][method], maxPos = v[tList[i][0]][method];
        for (int j = 1; j < 3; j++)
        {
            if (minPos > v[tList[i][j]][method])
            {
                minPos = v[tList[i][j]][method];
            }
            else if (maxPos < v[tList[i][j]][method])
            {
                maxPos = v[tList[i][j]][method];
            }
        }
        if (minPos <= halfPos)
        {
            lcT.push_back(tList[i]);
            lcN.push_back(nList[i]);
        }
        if (maxPos >= halfPos)
        {
            rcT.push_back(tList[i]);
            rcN.push_back(nList[i]);
        }
    }
    if (method == 2)
    {
        method = 0;
    }
    else
    {
        method++;
    }
    int thisMethod = method;
    buildBSP(lc, lcT, lcN, method, count + 1);
    method = thisMethod;
    buildBSP(rc, rcT, rcN, method, count + 1);
}

void Mesh::deleteBSP(BSP_node* r)
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

#endif
