/**************************************************
RTCube is subclass class of RTGeometry
that represents a 3D cube.
*****************************************************/
#pragma once
#include "RTGeometry.h"
#include <glm/glm.hpp>

class RTCube : public RTGeometry
{
public:
    void init(void)
    {
        // vertex positions
        const GLfloat positions[24][3] = {
            // Front face
            {-0.5f, -0.5f, 0.5f},
            {-0.5f, 0.5f, 0.5f},
            {0.5f, 0.5f, 0.5f},
            {0.5f, -0.5f, 0.5f},
            // Back face
            {-0.5f, -0.5f, -0.5f},
            {-0.5f, 0.5f, -0.5f},
            {0.5f, 0.5f, -0.5f},
            {0.5f, -0.5f, -0.5f},
            // Left face
            {-0.5f, -0.5f, 0.5f},
            {-0.5f, 0.5f, 0.5f},
            {-0.5f, 0.5f, -0.5f},
            {-0.5f, -0.5f, -0.5f},
            // Right face
            {0.5f, -0.5f, 0.5f},
            {0.5f, 0.5f, 0.5f},
            {0.5f, 0.5f, -0.5f},
            {0.5f, -0.5f, -0.5f},
            // Top face
            {0.5f, 0.5f, 0.5f},
            {-0.5f, 0.5f, 0.5f},
            {-0.5f, 0.5f, -0.5f},
            {0.5f, 0.5f, -0.5f},
            // Bottom face
            {0.5f, -0.5f, 0.5f},
            {-0.5f, -0.5f, 0.5f},
            {-0.5f, -0.5f, -0.5f},
            {0.5f, -0.5f, -0.5f}};
        // vertex normals
        const GLfloat normals[24][3] = {
            // Front face
            {0.0f, 0.0f, 1.0f},
            {0.0f, 0.0f, 1.0f},
            {0.0f, 0.0f, 1.0f},
            {0.0f, 0.0f, 1.0f},
            // Back face
            {0.0f, 0.0f, -1.0f},
            {0.0f, 0.0f, -1.0f},
            {0.0f, 0.0f, -1.0f},
            {0.0f, 0.0f, -1.0f},
            // Left face
            {-1.0f, 0.0f, 0.0f},
            {-1.0f, 0.0f, 0.0f},
            {-1.0f, 0.0f, 0.0f},
            {-1.0f, 0.0f, 0.0f},
            // Right face
            {1.0f, 0.0f, 0.0f},
            {1.0f, 0.0f, 0.0f},
            {1.0f, 0.0f, 0.0f},
            {1.0f, 0.0f, 0.0f},
            // Top face
            {0.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            // Bottom face
            {0.0f, -1.0f, 0.0f},
            {0.0f, -1.0f, 0.0f},
            {0.0f, -1.0f, 0.0f},
            {0.0f, -1.0f, 0.0f}};
        // Cube indices
        const GLuint indices[36] = {
            0, 1, 2, 0, 2, 3,       // Front face
            4, 5, 6, 4, 6, 7,       // Back face
            8, 9, 10, 8, 10, 11,    // Left face
            12, 13, 14, 12, 14, 15, // Right face
            16, 17, 18, 16, 18, 19, // Top face
            20, 21, 22, 20, 22, 23  // Bottom face
        };
        for (int i = 0; i < sizeof(indices) / sizeof(indices[0]); i += 3)
        {
            int idx1 = indices[i];
            int idx2 = indices[i + 1];
            int idx3 = indices[i + 2];
            Triangle tri;
            // positions
            tri.P.push_back(glm::vec3(positions[idx1][0], positions[idx1][1], positions[idx1][2]));
            tri.P.push_back(glm::vec3(positions[idx2][0], positions[idx2][1], positions[idx2][2]));
            tri.P.push_back(glm::vec3(positions[idx3][0], positions[idx3][1], positions[idx3][2]));
            // normals
            tri.N.push_back(glm::vec3(normals[idx1][0], normals[idx1][1], normals[idx1][2]));
            tri.N.push_back(glm::vec3(normals[idx2][0], normals[idx2][1], normals[idx2][2]));
            tri.N.push_back(glm::vec3(normals[idx3][0], normals[idx3][1], normals[idx3][2]));

            elements.push_back(tri);
        }
        count = sizeof(indices) / sizeof(indices[0]);
    }
};
