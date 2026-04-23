#pragma once
#include <vector>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "Mesh.h"

// Define mesh type for the whole project
typedef OpenMesh::TriMesh_ArrayKernelT<> MeshType;

namespace MeshProcessor
{
	// Phase 1: Convert the Assimp arrays, into the OpenMesh object
	MeshType convertRawToOpenMesh(const ModelData& model);

	// Phase 2: Decimate the mesh using OpenMesh's decimation tools
	void decimateMesh(MeshType& mesh, int TargetVertexCount);

	// Phase 3: Extract flat arrays from the optimised OpenMesh object
	void extractRawFromOpenMesh(const MeshType& mesh, std::vector<Vertex>& outVertices, std::vector<unsigned int>& outIndices);
}
