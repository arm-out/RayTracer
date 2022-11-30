#pragma once

#include <vector>
#include "Triangle.h"

class RTGeometry
{
public:
    int count;                      // number of elements to draw
    std::vector<Triangle> elements; // list of triangles

    virtual void init(){};
    virtual void init(const char *s){};
};