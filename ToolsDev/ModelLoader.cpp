#include "ModelLoader.h"

bool ModelLoader::Load(const std::string& filePath, ModelData& outModel)
{
    const aiScene* scene = m_importer.ReadFile(filePath,
        aiProcess_Triangulate |
        aiProcess_GenSmoothNormals |
        aiProcess_FlipUVs |
        aiProcess_CalcTangentSpace |
        aiProcess_JoinIdenticalVertices);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        m_lastError = m_importer.GetErrorString();
        return false;
    }

    outModel.meshes.clear();
    aiMatrix4x4 identity;
    ProcessNode(scene->mRootNode, scene, identity, outModel);
    return true;
}

void ModelLoader::ProcessNode(aiNode* node, const aiScene* scene, const aiMatrix4x4& parentTransform, ModelData& outModel)
{
    // Accumulate this node's transform with its parent
    aiMatrix4x4 globalTransform = parentTransform * node->mTransformation;

    for (unsigned int i = 0; i < node->mNumMeshes; i++)
    {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        outModel.meshes.push_back(ProcessMesh(mesh, globalTransform));
    }

    for (unsigned int i = 0; i < node->mNumChildren; i++)
    {
        ProcessNode(node->mChildren[i], scene, globalTransform, outModel);
    }
}

MeshData ModelLoader::ProcessMesh(aiMesh* mesh, const aiMatrix4x4& transform)
{
    MeshData data;
    data.name = mesh->mName.C_Str();
    data.vertices.reserve(mesh->mNumVertices);

    // Normal matrix: inverse-transpose of upper-left 3x3
    aiMatrix3x3 normalMatrix(transform);
    normalMatrix.Inverse().Transpose();

    for (unsigned int i = 0; i < mesh->mNumVertices; i++)
    {
        Vertex v{};

        // Transform position by the node's world matrix
        aiVector3D pos = transform * mesh->mVertices[i];
        v.position[0] = pos.x;
        v.position[1] = pos.y;
        v.position[2] = pos.z;

        if (mesh->HasNormals())
        {
            // Transform normal by the inverse-transpose matrix
            aiVector3D n = normalMatrix * mesh->mNormals[i];
            n.Normalize();
            v.normal[0] = n.x;
            v.normal[1] = n.y;
            v.normal[2] = n.z;
        }

        if (mesh->mTextureCoords[0])
        {
            v.texCoord[0] = mesh->mTextureCoords[0][i].x;
            v.texCoord[1] = mesh->mTextureCoords[0][i].y;
        }

        data.vertices.push_back(v);
    }

    for (unsigned int i = 0; i < mesh->mNumFaces; i++)
    {
        aiFace& face = mesh->mFaces[i];
        for (unsigned int j = 0; j < face.mNumIndices; j++)
        {
            data.indices.push_back(face.mIndices[j]);
        }
    }

    return data;
}