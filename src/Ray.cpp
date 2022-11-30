#include <queue>
#include <pthread.h>
#include "Ray.h"
#include "progressbar.hpp"

using namespace RayTracer;

#define THREAD_NUM 8

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

    for (int j = 0; j < h; j++)
    {
        for (int i = 0; i < w; i++)
        {
            Ray ray = RayThruPixel(cam, i, j, w, h);
            // Create task
            renderTask t = {
                .renderFunction = &RenderPixel,
                .scene = &scene,
                .image = &image,
                .ray = ray,
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
    task->renderFunction(task->ray, task->scene, task->image, task->i, task->j);
};

void RayTracer::submitTask(renderTask task)
{
    pthread_mutex_lock(&mutex);
    taskQueue.push(task);
    pthread_mutex_unlock(&mutex);
    pthread_cond_signal(&condQueue);
}

void RayTracer::RenderPixel(Ray ray, RTScene *scene, Image *image, int i, int j)
{
    int w = image->width;
    Intersection hit = Intersect(ray, *scene);
    image->pixels[j * w + i] = FindColor(hit, 1);
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