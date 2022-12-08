#include <queue>
#include <pthread.h>
#include <random>
#include "Ray.h"
#include "progressbar.hpp"

using namespace RayTracer;

#define THREAD_NUM 8
#define NUM_SAMPLES 1000
#define LAMBDA 0.5f
#define DIFFUSE 0.5f
#define GLOBAL_ILLUM 1

std::queue<renderTask> taskQueue;
pthread_mutex_t mutex;
pthread_cond_t condQueue;
int numJobs;
progressbar bar;

void RayTracer::Raytrace(Camera &cam, RTScene &scene, Image &image)
{
    pthread_t th[THREAD_NUM];
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&condQueue, NULL);
    numJobs = image.width * image.height;
    bar.set_niter(numJobs);
    for (int i = 0; i < THREAD_NUM; i++)
    {
        if (pthread_create(&th[i], NULL, &startThread, NULL) != 0)
        {
            perror("Failed to create thread");
        }
    }

    int w = image.width;
    int h = image.height;
    auto start = std::chrono::high_resolution_clock::now();

    // height width
    for (int j = 0; j < h; j++)
    {
        for (int i = 0; i < w; i++)
        {
            // Create task
            renderTask t = {
                .renderFunction = &RenderPixel,
                .cam = &cam,
                .scene = &scene,
                .image = &image,
                .i = i,
                .j = j};
            // Submit task
            submitTask(t);
        }
    }

    for (int i = 0; i < THREAD_NUM; i++)
    {
        if (pthread_join(th[i], NULL) != 0)
        {
            perror("Failed to join thread");
        }
    }

    std::cout << std::endl;

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << "Rendered in " << duration.count() << " seconds" << std::endl;

    bar.reset();
    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&condQueue);
};

void RayTracer::RenderPixel(Camera *cam, RTScene *scene, Image *image, int i, int j)
{
    int w = image->width;
    int h = image->height;
    glm::vec3 pixel_color = FindColor(cam, scene, i, j, w, h);
    image->pixels[(h - j - 1) * w + i] = pixel_color;
};

Ray RayTracer::RayThruPixel(Camera &cam, int i, int j, int width, int height)
{
    Ray ray;
    ray.p0 = cam.eye;

    // camera basis Cez, Cey, Cex
    glm::vec3 w = glm::normalize(cam.eye - cam.target);
    glm::vec3 u = glm::normalize(glm::cross(cam.up, w));
    glm::vec3 v = glm::cross(w, u);

    // Random ray offset
    float x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    float alpha = 2.0f * (static_cast<float>(i + x) / static_cast<float>(width)) - 1.0f;
    float beta = 1.0f - 2.0f * (static_cast<float>(j + y) / static_cast<float>(height));

    float a = cam.aspect;
    float fovy = cam.fovy * M_PI / 180.0f;

    ray.dir = glm::normalize((alpha * a * glm::tan(fovy / 2.0f) * u) + (beta * glm::tan(fovy / 2.0f) * v) - w);
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
    hit.N = glm::normalize((l1 * triangle.N[0]) + (l2 * triangle.N[1]) + (l3 * triangle.N[2]));
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

glm::vec3 RayTracer::FindColor(Camera *cam, RTScene *scene, int i, int j, int w, int h)
{
    glm::vec3 color_acc = glm::vec3(0.0f, 0.0f, 0.0f);

    // Multisampling
    for (int k = 0; k < NUM_SAMPLES; k++)
    {
        glm::vec3 color = glm::vec3(0.0f, 0.0f, 0.0f);
        glm::vec3 total_weight = glm::vec3(1.0f, 1.0f, 1.0f);
        float factor = 1.0f;
        int path_len = 1;

        Ray ray = RayThruPixel(*cam, i, j, w, h);

        while (1)
        {
            Intersection hit = Intersect(ray, *scene);
            if (hit.dist == std::numeric_limits<float>::infinity())
            { // No hit
                break;
            }

            Material *hit_mat = hit.triangle->material;
            bool terminate = rand() < (RAND_MAX * LAMBDA);

            if (terminate)
            { // Final bounce
                factor *= LAMBDA;
                color += total_weight * (glm::vec3(hit_mat->emision) + finalHitColor(hit, scene, path_len));
                break;
            }
            else
            { // Next bounce
                path_len++;
                factor *= (1 - LAMBDA);
                color += total_weight * glm::vec3(hit_mat->emision);

                if (rand() < (RAND_MAX * DIFFUSE))
                { // Diffuse bounce
                    total_weight *= glm::vec3(hit_mat->diffuse);
                    ray = diffuseRay(hit);
                }
                else
                { // Specular bounce
                    total_weight *= glm::vec3(hit_mat->specular);
                    ray = specularRay(hit);
                }
            }
        }

        color_acc += color / factor;
    }

    color_acc /= NUM_SAMPLES;
    return color_acc;
};

glm::vec3 RayTracer::finalHitColor(Intersection hit, RTScene *scene, int path_len)
{
    glm::vec3 color = glm::vec3(0.0f, 0.0f, 0.0f);

    // Material properties
    glm::vec3 diffuse = glm::vec3(hit.triangle->material->diffuse);

    // Lights
    std::map<std::string, Light *> lights = scene->light;
    std::vector<glm::vec3> lightpositions;
    std::vector<glm::vec3> lightcolors;

    for (auto light : lights)
    {
        lightpositions.push_back(glm::vec3(light.second->position));
        lightcolors.push_back(glm::vec3(light.second->color));
    }

    for (int i = 0; i < lights.size(); i++)
    {
        glm::vec3 light_pos = lightpositions[i];
        glm::vec3 light_color = lightcolors[i];
        float dist_tolight = glm::length(light_pos - hit.P);

        glm::vec3 light_dir = glm::normalize(light_pos - hit.P);
        float Lambertian = glm::max(glm::dot(hit.N, light_dir), 0.0f);

        Ray shadow_ray = shadowRay(hit, light_dir);
        Intersection shadow = Intersect(shadow_ray, *scene);

        bool visible = shadow.dist < dist_tolight ? false : true;
        if (visible)
        {
            glm::vec3 L_cum = glm::vec3(0.0f, 0.0f, 0.0f);
            L_cum += diffuse * Lambertian;
            float attenuation = dist_tolight * 0.63f;
            // float attenuation = 1.2f;
            L_cum /= attenuation;
            color += L_cum;
        }
    }

    return color;
};

Ray RayTracer::diffuseRay(Intersection hit)
{
    // world up vector
    glm::vec3 up(0.0f, 0.0f, 0.0f);

    // Cosine weighted hemisphere sampling
    float s = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float t = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float u = 2 * glm::pi<float>() * s;
    float v = glm::sqrt(1.0f - t);

    glm::vec3 d(v * glm::cos(u), glm::sqrt(t), v * glm::sin(u));

    // Rotate hemisphere acc to surface normal
    if (hit.N == -1.0f * up)
    { // Same direction
        d = -1.0f * d;
    }
    else if (hit.N != up)
    { // Rotate ray
        float angle = glm::cos(glm::dot(hit.N, up));
        glm::vec3 axis = glm::cross(hit.N, up);
        glm::mat4 rotateMatrix = glm::rotate(glm::mat4(), angle, axis);
        d = glm::normalize(glm::vec3(rotateMatrix * glm::vec4(d, 0.0f)));
    }

    glm::vec3 offset_pos = hit.P + 0.01f * d;
    Ray ray{
        .p0 = offset_pos,
        .dir = d};

    return ray;
}
Ray RayTracer::specularRay(Intersection hit)
{
    glm::vec3 d = 2.0f * glm::dot(hit.N, hit.V) * hit.N - hit.V;
    glm::vec3 offset_pos = hit.P + 0.01f * d;
    Ray ray{
        .p0 = offset_pos,
        .dir = d};

    return ray;
}
Ray RayTracer::shadowRay(Intersection hit, glm::vec3 light_dir)
{
    glm::vec3 offset_pos = hit.P + 0.01f * light_dir;
    Ray ray{
        .p0 = offset_pos,
        .dir = light_dir};

    return ray;
}

void *RayTracer::startThread(void *args)
{
    while (numJobs != 0)
    {
        renderTask task;
        pthread_mutex_lock(&mutex);

        while (taskQueue.size() == 0)
        {
            pthread_cond_wait(&condQueue, &mutex);
        }

        task = taskQueue.front();
        taskQueue.pop();
        numJobs--;
        bar.update();
        pthread_mutex_unlock(&mutex);
        executeTask(&task);
    }
};

void RayTracer::executeTask(renderTask *task)
{
    task->renderFunction(task->cam, task->scene, task->image, task->i, task->j);
};

void RayTracer::submitTask(renderTask task)
{
    pthread_mutex_lock(&mutex);
    taskQueue.push(task);
    pthread_mutex_unlock(&mutex);
    pthread_cond_signal(&condQueue);
}