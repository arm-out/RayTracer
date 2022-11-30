#pragma once

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <vector>
#include "Material.h"

struct Triangle
{
    std::vector<glm::vec3> P; // 3 positions
    std::vector<glm::vec3> N; // 3 normals
    Material *material = NULL;
};