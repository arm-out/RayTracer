/**************************************************
Scene.inl contains the definition of the scene graph
*****************************************************/
#include "RTScene.h"
#include "RTCube.h"
#include "RTObj.h"

using namespace glm;
void RTScene::init(void)
{
    // Create a geometry palette
    geometry["cube"] = new RTCube;
    geometry["cube"]->init();
    // geometry["ball"] = new RTObj;
    // geometry["ball"]->init("models/sphere.obj");

    // Create a material palette
    material["red"] = new Material;
    material["red"]->ambient = vec4(0.1f, 0.1f, 0.1f, 1.0f);
    material["red"]->diffuse = vec4(0.5f, 0.0f, 0.0f, 1.0f);
    material["red"]->specular = vec4(0.1f, 0.0f, 0.0f, 1.0f);
    material["red"]->shininess = 100.0f;

    material["white"] = new Material;
    material["white"]->ambient = vec4(0.5f, 0.5f, 0.5f, 1.0f);
    material["white"]->diffuse = vec4(0.725f, 0.71f, 0.68f, 1.0f);
    material["white"]->specular = vec4(0.1f, 0.1f, 0.1f, 1.0f);
    material["white"]->shininess = 100.0f;

    material["whitebox"] = new Material;
    material["whitebox"]->ambient = vec4(0.5f, 0.5f, 0.5f, 1.0f);
    material["whitebox"]->diffuse = vec4(0.725f, 0.71f, 0.68f, 1.0f);
    material["whitebox"]->specular = vec4(0.4f, 0.4f, 0.4f, 1.0f);
    material["whitebox"]->shininess = 100.0f;

    material["green"] = new Material;
    material["green"]->ambient = vec4(0.1f, 0.1f, 0.1f, 1.0f);
    material["green"]->diffuse = vec4(0.0f, 0.5f, 0.0f, 1.0f);
    material["green"]->specular = vec4(0.0f, 0.1f, 0.0f, 1.0f);
    material["green"]->shininess = 100.0f;

    material["bulb"] = new Material;
    material["bulb"]->ambient = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    material["bulb"]->diffuse = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    material["bulb"]->specular = vec4(1.0f, 1.0f, 1.0f, 1.0f);
    material["bulb"]->emision = vec4(1.0f, 0.2f, 0.1f, 1.0f);
    material["bulb"]->shininess = 200.0f;

    material["ceramic"] = new Material;
    material["ceramic"]->ambient = vec4(0.02f, 0.07f, 0.2f, 1.0f);
    material["ceramic"]->diffuse = vec4(0.1f, 0.25f, 0.7f, 1.0f);
    material["ceramic"]->specular = vec4(0.9f, 0.9f, 0.9f, 1.0f);
    material["ceramic"]->shininess = 150.0f;

    material["silver"] = new Material;
    material["silver"]->ambient = vec4(0.1f, 0.1f, 0.1f, 1.0f);
    material["silver"]->diffuse = vec4(0.2f, 0.2f, 0.2f, 1.0f);
    material["silver"]->specular = vec4(0.9f, 0.9f, 0.9f, 1.0f);
    material["silver"]->shininess = 50.0f;

    material["wood"] = new Material;
    material["wood"]->ambient = vec4(0.1f, 0.1f, 0.1f, 1.0f);
    material["wood"]->diffuse = vec4(0.3f, 0.15f, 0.1f, 1.0f);
    material["wood"]->specular = vec4(0.3f, 0.15f, 0.1f, 1.0f);
    material["wood"]->shininess = 100.0f;

    material["turquoise"] = new Material;
    material["turquoise"]->ambient = vec4(0.1f, 0.2f, 0.17f, 1.0f);
    material["turquoise"]->diffuse = vec4(0.2f, 0.375f, 0.35f, 1.0f);
    material["turquoise"]->specular = vec4(0.3f, 0.3f, 0.3f, 1.0f);
    material["turquoise"]->shininess = 100.0f;

    // Create a model palette
    model["floor"] = new RTModel;
    model["floor"]->geometry = geometry["cube"];
    model["floor"]->material = material["white"];
    model["redwall"] = new RTModel;
    model["redwall"]->geometry = geometry["cube"];
    model["redwall"]->material = material["red"];
    model["greenwall"] = new RTModel;
    model["greenwall"]->geometry = geometry["cube"];
    model["greenwall"]->material = material["green"];
    model["backwall"] = new RTModel;
    model["backwall"]->geometry = geometry["cube"];
    model["backwall"]->material = material["white"];
    model["ceiling"] = new RTModel;
    model["ceiling"]->geometry = geometry["cube"];
    model["ceiling"]->material = material["white"];
    model["tall"] = new RTModel;
    model["tall"]->geometry = geometry["cube"];
    model["tall"]->material = material["silver"];
    model["short"] = new RTModel;
    model["short"]->geometry = geometry["cube"];
    model["short"]->material = material["wood"];
    // model["ball"] = new RTModel;
    // model["ball"]->geometry = geometry["ball"];
    // model["ball"]->material = material["wood"];

    // Create a light palette
    light["bulb"] = new Light;
    light["bulb"]->position = vec4(0.0f, 1.8f, -0.5f, 1.0f);       //-1
    light["bulb"]->color = 10.0f * vec4(1.0f, 0.84f, 0.67f, 1.0f); // 10

    // Build the scene graph
    node["floor"] = new RTNode;
    node["redwall"] = new RTNode;
    node["greenwall"] = new RTNode;
    node["backwall"] = new RTNode;
    node["ceiling"] = new RTNode;
    node["tall"] = new RTNode;
    node["short"] = new RTNode;
    // node["ball"] = new RTNode;

    node["floor"]->models.push_back(model["floor"]);
    node["floor"]->modeltransforms.push_back(scale(vec3(5.0f, 0.1f, 4.0f)));

    node["redwall"]->models.push_back(model["redwall"]);
    node["redwall"]->modeltransforms.push_back(scale(vec3(0.1f, 2.7f, 4.0f)));

    node["greenwall"]->models.push_back(model["greenwall"]);
    node["greenwall"]->modeltransforms.push_back(scale(vec3(0.1f, 2.7f, 4.0f)));

    node["backwall"]->models.push_back(model["backwall"]);
    node["backwall"]->modeltransforms.push_back(scale(vec3(5.0f, 2.7f, 0.1f)));

    node["ceiling"]->models.push_back(model["ceiling"]);
    node["ceiling"]->modeltransforms.push_back(scale(vec3(5.0f, 0.1f, 4.0f)));

    node["tall"]->models.push_back(model["tall"]);
    node["tall"]->modeltransforms.push_back(scale(vec3(1.0f, 2.0f, 1.0f)));
    node["short"]->models.push_back(model["short"]);
    node["short"]->modeltransforms.push_back(scale(vec3(1.0f)));
    // node["ball"]->models.push_back(model["ball"]);
    // node["ball"]->modeltransforms.push_back(scale(vec3(0.25f, 0.25f, 0.25f)));

    node["world"]->childnodes.push_back(node["floor"]);
    node["world"]->childtransforms.push_back(translate(vec3(0.0f, -0.34f, -1.8f)));
    node["world"]->childnodes.push_back(node["redwall"]);
    node["world"]->childtransforms.push_back(translate(vec3(-1.78f, 1.0f, -1.8f)));
    node["world"]->childnodes.push_back(node["greenwall"]);
    node["world"]->childtransforms.push_back(translate(vec3(1.78f, 1.0f, -1.8f)));
    node["world"]->childnodes.push_back(node["backwall"]);
    node["world"]->childtransforms.push_back(translate(vec3(0.0f, 1.0f, -3.35f)));
    node["world"]->childnodes.push_back(node["ceiling"]);
    node["world"]->childtransforms.push_back(translate(vec3(0.0f, 2.35f, -1.8f)));
    node["world"]->childnodes.push_back(node["tall"]);
    node["world"]->childtransforms.push_back(translate(vec3(-0.6f, 0.3f, -1.9f)) * rotate(18.0f * float(M_PI) / 180.0f, vec3(0.0f, 1.0f, 0.0f)));
    // node["world"]->childtransforms.push_back(translate(vec3(-0.6f, 0.3f, -1.9f)) * rotate(30.0f * float(M_PI) / 180.0f, vec3(0.0f, 1.0f, 0.0f)));

    node["world"]->childnodes.push_back(node["short"]);
    node["world"]->childtransforms.push_back(translate(vec3(0.6f, 0.0f, -0.8f)) * rotate(-18.0f * float(M_PI) / 180.0f, vec3(0.0f, 1.0f, 0.0f)));
    // node["world"]->childnodes.push_back(node["ball"]);
    // node["world"]->childtransforms.push_back(translate(vec3(-0.3f, -0.1f, -1.0f)));

    // Put a camera
    camera = new Camera;
    camera->target_default = vec3(0.0f, 1.0f, 0.0f);
    camera->eye_default = vec3(0.0f, 1.0f, 5.0f);
    camera->up_default = vec3(0.0f, 1.0f, 0.0f);
    camera->reset();
}
