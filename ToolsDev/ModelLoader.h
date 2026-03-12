#pragma once

#include "Mesh.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/config.h>
#include <string>
#include <memory>

class ModelLoader
{
public:
    ModelLoader() = default;

    bool Load(const std::string& filePath, ModelData& outModel);
    const std::string& GetLastError() const { return m_lastError; }

private:
    void ProcessNode(aiNode* node, const aiScene* scene, const aiMatrix4x4& parentTransform, ModelData& outModel);
    MeshData ProcessMesh(aiMesh* mesh, const aiMatrix4x4& transform);

    Assimp::Importer m_importer;
    std::string m_lastError;
};