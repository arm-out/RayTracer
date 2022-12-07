#pragma once

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <pthread.h>
#include "RTScene.h"
#include "Camera.h"
#include "Image.h"

struct Ray
{
    glm::vec3 p0;  // base point
    glm::vec3 dir; // direction
};

struct Intersection
{
    glm::vec3 P;        // Position of Intersection
    glm::vec3 N;        // Surface normal;
    glm::vec3 V;        // Direction to incoming ray
    Triangle *triangle; // Pointer to geometric primitive (and material info)
    float dist;         // distance to the source of ray
};

namespace RayTracer
{
    void Raytrace(Camera &cam, RTScene &scene, Image &image);
    void RenderPixel(Camera *cam, RTScene *scene, Image *image, int i, int j);
    Ray RayThruPixel(Camera &cam, int i, int j, int width, int height);
    Intersection Intersect(Ray ray, Triangle &triangle);
    Intersection Intersect(Ray ray, RTScene &scene);

    glm::vec3 FindColor(Camera *cam, RTScene *scene, int i, int j, int w, int h);
    Ray diffuseRay(Intersection hit);
    Ray specularRay(Intersection hit);
    Ray shadowRay(Intersection hit, glm::vec3 light_dir);
    glm::vec3 finalHitColor(Intersection hit, RTScene *scene, int path_len);

    // Multithreading Processing
    typedef struct renderTask
    {
        void (*renderFunction)(Camera *, RTScene *, Image *, int, int);
        Camera *cam;
        RTScene *scene;
        Image *image;
        int i, j;
    } renderTask;

    void executeTask(renderTask *task);
    void *startThread(void *args);
    void submitTask(renderTask task);
};