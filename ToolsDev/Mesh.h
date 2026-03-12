#pragma once

#include <vector>
#include <string>

struct Vertex
{
    float position[3];
    float normal[3];
    float texCoord[2];
};

struct MeshData
{
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::string name;
};

struct ModelData
{
    std::vector<MeshData> meshes;
};