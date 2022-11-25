#include "Image.h"

#include <OpenGL/gl3.h>
#include <iostream>

// Default constructor
Image::Image(const int width, const int height)
{
    // std::cout << "Creating Image Object" << std::endl;
    Image::width = width;
    Image::height = height;
}

// Default Destructor
Image::~Image() {}

// Initialize pixel array
void Image::init()
{
    // std::cout << "Initializing pixel array" << std::endl;
    pixels.resize(width * height, glm::vec3(0.f, 0.f, 0.f));
    // std::cout << "Image " << width << "x" << height << " pixel array length " << pixels.size() << std::endl;

    glGenFramebuffers(1, &fbo);
    glGenTextures(1, &tbo);
    // std::cout << "Buffers initialized" << std::endl;
}

void Image::draw()
{
    // std::cout << "Loading Textures" << std::endl;
    // Load texture
    glBindTexture(GL_TEXTURE_2D, tbo);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_FLOAT, &pixels[0][0]);

    // std::cout << "Reading frame buffer" << std::endl;
    // Attach texture and read frame
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tbo, 0);

    // Copy frame buffer from read to write
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    // std::cout << "Frame Drawn" << std::endl;
}
