/**************************************************
A model is a geometry with material.
*****************************************************/
#pragma once

#include "RTGeometry.h"
#include "Material.h"

struct RTModel
{
    RTGeometry *geometry;
    Material *material;
};