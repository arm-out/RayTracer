/**************************************************
RTObj is subclass class of RTGeometry
that loads an obj file.
*****************************************************/
#pragma once

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include "RTGeometry.h"

class RTObj : public RTGeometry
{
public:
    void init(const char *filename);
};