#pragma once

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <pthread.h>
#include "RTScene.h"
#include "Camera.h"
#include "Image.h"

struct Ray
{
    glm::vec3 p0;  // basepoint
    glm::vec3 dir; // direction
};

struct Intersection
{
    glm::vec3 P;        // Position of Intersection
    glm::vec3 N;        // Surface normal;
    glm::vec3 V;        // Direction to incoming ray
    Triangle *triangle; // Pointer to geometruc primitive (and material info)
    float dist;         // distance to the source of ray
};

namespace RayTracer
{
    void Raytrace(Camera &cam, RTScene &scene, Image &image);
    Ray RayThruPixel(Camera &cam, int i, int j, int width, int height);
    Intersection Intersect(Ray ray, Triangle &triangle);
    Intersection Intersect(Ray ray, RTScene &scene);
    glm::vec3 FindColor(Intersection hit, int recursion_depth);

    // Multithreaded Processing
};