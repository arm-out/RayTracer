#pragma once

#include <vector>
#include <glm/glm.hpp>

class Image
{
public:
    int width, height;             // Image dimensions
    std::vector<glm::vec3> pixels; // Image pixel array

    Image(int width, int height); // Constructor
    ~Image();                     // Destructor
    void init();                  // Initialize pixel array
    void draw();                  // Draw image

private:
    unsigned int fbo; // frame buffer object
    unsigned int tbo; // texture buffer object
};