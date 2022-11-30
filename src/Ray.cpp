#include "Ray.h"
#include "progressbar.hpp"

using namespace RayTracer;

void RayTracer::Raytrace(Camera &cam, RTScene &scene, Image &image)
{
    int w = image.width;
    int h = image.height;
    progressbar bar(h);
    for (int j = 0; j < h; j++)
    {
        // std::cout << "Row " << j << "/" << h << std::endl;
        bar.update();
        for (int i = 0; i < w; i++)
        {
            Ray ray = RayThruPixel(cam, i, j, w, h);
            Intersection hit = Intersect(ray, scene);
            image.pixels[j * w + i] = FindColor(hit, 1);
        }
    }
    std::cout << std::endl;
};

Ray RayTracer::RayThruPixel(Camera &cam, int i, int j, int width, int height)
{
    Ray ray;
    ray.p0 = cam.eye;

    // camera basis Cez, Cey, Cex
    glm::vec3 w = glm::normalize(cam.eye - cam.target);
    glm::vec3 u = glm::normalize(glm::cross(cam.up, w));
    glm::vec3 v = glm::cross(w, u);

    float alpha = 2.0f * (static_cast<float>(i + 0.5) / static_cast<float>(width)) - 1.0f;
    float beta = 1.0f - 2.0f * (static_cast<float>(j + 0.5) / static_cast<float>(height));

    float a = cam.aspect;
    float fovy = cam.fovy * M_PI / 180.0f;

    ray.dir = glm::normalize((alpha * a * glm::tan(fovy / 2.0f) * u) + (-beta * glm::tan(fovy / 2.0f) * v) - w);
    return ray;
};

Intersection RayTracer::Intersect(Ray ray, Triangle &triangle)
{
    glm::vec3 p1 = triangle.P[0];
    glm::vec3 p2 = triangle.P[1];
    glm::vec3 p3 = triangle.P[2];
    glm::vec3 neg_d = -1.0f * ray.dir;

    glm::mat4 linear_system;
    linear_system[0] = glm::vec4(p1, 1.0f);
    linear_system[1] = glm::vec4(p2, 1.0f);
    linear_system[2] = glm::vec4(p3, 1.0f);
    linear_system[3] = glm::vec4(neg_d, 0.0f);

    glm::vec4 soln = glm::inverse(linear_system) * glm::vec4(ray.p0, 1.0f);

    float l1 = soln[0];
    float l2 = soln[1];
    float l3 = soln[2];
    float t = soln[3];

    Intersection hit;
    hit.P = (l1 * p1) + (l2 * p2) + (l3 * p3);
    hit.N = glm::normalize((l1 * triangle.N[0]) + (l2 * triangle.N[1]) + (l3 * triangle.N[3]));
    hit.V = neg_d;
    hit.triangle = &triangle;
    hit.dist = (l1 >= 0.f && l2 >= 0.f && l3 >= 0.f && t >= 0.f) ? t : std::numeric_limits<float>::infinity();

    return hit;
};

Intersection RayTracer::Intersect(Ray ray, RTScene &scene)
{
    float mindist = std::numeric_limits<float>::infinity();
    Intersection hit;
    hit.dist = mindist;

    for (Triangle &t : scene.triangle_soup)
    {
        Intersection hit_temp = Intersect(ray, t);
        if (hit_temp.dist < mindist)
        {
            mindist = hit_temp.dist;
            hit = hit_temp;
        }
    }
    // std::cout << "Intersection with distance " << hit.dist << std::endl;
    return hit;
};

glm::vec3 RayTracer::FindColor(Intersection hit, int recursion_depth)
{
    if (hit.dist != std::numeric_limits<float>::infinity())
    { // Intersection exists
        // std::cout << "Color Black" << std::endl;
        return 0.5f * hit.N + 0.5f;
    }
    else
    {
        // std::cout << "Color White" << std::endl;
        return glm::vec3(1.0f, 1.0f, 1.0f);
    }
};