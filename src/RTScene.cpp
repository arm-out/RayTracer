/**************************************************
RTScene.cpp contains the implementation of the buildTriangleSoup command
*****************************************************/
#include "RTScene.h"
#include "RTCube.h"
#include "RTObj.h"
// The scene init definition
#include "RTScene.inl"

using namespace glm;
void RTScene::buildTriangleSoup(void)
{
    triangle_soup.clear();
    camera->computeMatrices();

    // Define stacks for depth-first search (DFS)
    std::stack<RTNode *> dfs_stack;
    std::stack<mat4> matrix_stack;

    // Initialize the current state variable for DFS
    RTNode *cur = node["world"]; // root of the tree
    mat4 cur_VM = glm::mat4(1.0f);

    dfs_stack.push(cur);
    matrix_stack.push(cur_VM);

    // Compute total number of connectivities in the graph; this would be an upper
    // bound for
    // the stack size in the depth first search over the directed acyclic graph
    int total_number_of_edges = 0;
    for (std::map<std::string, RTNode *>::iterator i_node = node.begin(); i_node !=
                                                                          node.end();
         i_node++)
    {
        total_number_of_edges += i_node->second->childnodes.size();
    }

    // If you want to print some statistics of your scene graph
    // std::cout<<"total numb of nodes = " << node.size() << std::endl;
    // std::cout<<"total number of edges = " << total_number_of_edges << std::endl;

    std::cout << "Building triangle soup..." << std::endl;
    while (!dfs_stack.empty())
    {
        // Detect whether the search runs into infinite loop by checking whether
        // the stack is longer than the number of edges in the graph.
        if (dfs_stack.size() > total_number_of_edges)
        {
            std::cerr << "Error: The scene graph has a closed loop." << std::endl;
            exit(-1);
        }

        // top-pop the stacks
        cur = dfs_stack.top();
        dfs_stack.pop();
        cur_VM = matrix_stack.top();
        matrix_stack.pop();

        // build triangles at the current node
        for (unsigned int i = 0; i < cur->models.size(); i++)
        {
            // Prepare to draw the geometry. Assign the modelview and the material.
            mat4 curModelView = cur_VM * cur->modeltransforms[i];
            mat3 invTMV = glm::transpose(glm::inverse(mat3(curModelView)));
            Material *curMat = cur->models[i]->material;

            for (Triangle ogTri : cur->models[i]->geometry->elements)
            {
                Triangle transTri;

                // Transformed position vectors
                vec4 transPos0 = curModelView * vec4(ogTri.P[0], 1);
                vec4 transPos1 = curModelView * vec4(ogTri.P[1], 1);
                vec4 transPos2 = curModelView * vec4(ogTri.P[2], 1);

                // Transformed normal vectors
                vec3 transNorm0 = glm::normalize(invTMV * ogTri.N[0]);
                vec3 transNorm1 = glm::normalize(invTMV * ogTri.N[1]);
                vec3 transNorm2 = glm::normalize(invTMV * ogTri.N[2]);

                transTri.P.insert(transTri.P.end(), {vec3(transPos0), vec3(transPos1), vec3(transPos2)});
                transTri.N.insert(transTri.N.end(), {transNorm0, transNorm1, transNorm2});
                transTri.material = curMat;
                triangle_soup.push_back(transTri);
            }

            // The draw command
            // shader->setUniforms();
            // (cur->models[i])->geometry->draw();
        }

        // Continue the DFS: put all the child nodes of the current node in the stack
        for (unsigned int i = 0; i < cur->childnodes.size(); i++)
        {
            dfs_stack.push(cur->childnodes[i]);
            matrix_stack.push(cur_VM * (cur->childtransforms[i]));
        }

    } // End of DFS while loop.

    std::cout << triangle_soup.size() << " triangles in scene" << std::endl;
}