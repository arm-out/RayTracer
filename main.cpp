#include <stdlib.h>
#include <iostream>
// OSX systems need their own headers
#ifdef __APPLE__
#include <OpenGL/gl3.h>
#include <OpenGL/glext.h>
#include <GLUT/glut.h>
#else
#include <GL/glew.h>
#include <GL/glut.h>
#endif
// Use of degrees is deprecated. Use radians for GLM functions
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include "Screenshot.h"
#include "Scene.h"
#include "RTScene.h"
#include "Image.h"
#include "Ray.h"

static const int width = 400;
static const int height = 300;
static const char *title = "Scene viewer";
static const glm::vec4 background(0.1f, 0.2f, 0.3f, 1.0f);
static bool rt_mode = false;
static Scene scene;
static RTScene rt_scene;
static Image image(width, height);

void printHelp()
{
    std::cout << R"(
    Available commands:
      press 'H' to print this message again.
      press Esc to quit.
      press 'O' to save a screenshot.
      press the arrow keys to rotate camera.
      press 'A'/'Z' to zoom.
      press 'R' to reset camera.
      press 'L' to turn on/off the lighting.
      press 'spacebar' to toggle RT Mode
    
)";
}

// Add random pixels to image
void randomPixels(Image *image)
{
    for (int i = 0; i < image->pixels.size(); i++)
    {
        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        image->pixels[i] = glm::vec3(r, g, b);
    }
}

void initialize(void)
{
    printHelp();
    glClearColor(background[0], background[1], background[2], background[3]); // background color
    glViewport(0, 0, width, height);

    // Initialize scene
    scene.init();
    rt_scene.init();
    rt_scene.buildTriangleSoup();

    // Initialize image
    image.init();
    // randomPixels(&image);
    RayTracer::Raytrace(*(rt_scene.camera), rt_scene, image);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
}

void display(void)
{
    std::cout << "Inside DISPLAY " << std::endl;
    if (rt_mode)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        image.draw();

        glutSwapBuffers();
        glFlush();
    }
    else
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        scene.draw();
        glutSwapBuffers();
        glFlush();
    }
}

void saveScreenShot(const char *filename = "test.png")
{
    int currentwidth = glutGet(GLUT_WINDOW_WIDTH);
    int currentheight = glutGet(GLUT_WINDOW_HEIGHT);
    Screenshot imag = Screenshot(currentwidth, currentheight);
    imag.save(filename);
}

void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 27: // Escape to quit
        exit(0);
        break;
    case 'h': // print help
        printHelp();
        break;
    case 'o': // save screenshot
        saveScreenShot();
        break;
    case 'r':
        scene.camera->aspect_default = float(glutGet(GLUT_WINDOW_WIDTH)) / float(glutGet(GLUT_WINDOW_HEIGHT));
        scene.camera->reset();
        glutPostRedisplay();
        break;
    case 'a':
        scene.camera->zoom(0.9f);
        glutPostRedisplay();
        break;
    case 'z':
        scene.camera->zoom(1.1f);
        glutPostRedisplay();
        break;
    case 'l':
        scene.shader->enablelighting = !(scene.shader->enablelighting);
        glutPostRedisplay();
        break;
    case ' ':
        if (rt_mode)
        {
            rt_mode = !rt_mode;
            std::cout << "RT Mode OFF" << std::endl;
        }
        else
        {
            rt_mode = !rt_mode;
            std::cout << "RT Mode ON" << std::endl;
        }
        glutPostRedisplay();
        break;
    default:
        glutPostRedisplay();
        break;
    }
}
void specialKey(int key, int x, int y)
{
    switch (key)
    {
    case GLUT_KEY_UP: // up
        scene.camera->rotateUp(-10.0f);
        glutPostRedisplay();
        break;
    case GLUT_KEY_DOWN: // down
        scene.camera->rotateUp(10.0f);
        glutPostRedisplay();
        break;
    case GLUT_KEY_RIGHT: // right
        scene.camera->rotateRight(-10.0f);
        std::cout << "RIGHT" << std::endl;
        glutPostRedisplay();
        break;
    case GLUT_KEY_LEFT: // left
        scene.camera->rotateRight(10.0f);
        glutPostRedisplay();
        break;
    }
}

int main(int argc, char **argv)
{
    // BEGIN CREATE WINDOW
    glutInit(&argc, argv);

#ifdef __APPLE__
    glutInitDisplayMode(GLUT_3_2_CORE_PROFILE | GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
#else
    glutInitContextVersion(3, 1);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
#endif
    glutInitWindowSize(width, height);
    glutCreateWindow(title);
#ifndef __APPLE__
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        std::cerr << "Error: " << glewGetErrorString(err) << std::endl;
    }
#endif
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
    // END CREATE WINDOW

    initialize();
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKey);

    glutMainLoop();
    return 0; /* ANSI C requires main to return int. */
}
