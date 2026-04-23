#include "MeshProcessor.h"

// including the Decimation tools from OpenMesh
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>

namespace MeshProcessor
{
	MeshType convertRawToOpenMesh(const ModelData& model)
	{
		MeshType mesh;

		unsigned int vertexOffset = 0;
		for (const auto& meshData : model.meshes)
		{
			// Add vertices to the mesh
			for (const auto& v : meshData.vertices)
			{
				mesh.add_vertex(MeshType::Point(v.position[0], v.position[1], v.position[2]));
			}

			// Add faces to the mesh
			for (size_t i = 0; i < meshData.indices.size(); i += 3)
			{
				mesh.add_face(
					MeshType::VertexHandle(meshData.indices[i] + vertexOffset),
					MeshType::VertexHandle(meshData.indices[i + 1] + vertexOffset),
					MeshType::VertexHandle(meshData.indices[i + 2] + vertexOffset)
				);
			}

			vertexOffset += static_cast<unsigned int>(meshData.vertices.size());
		}

		return mesh;
	}

	void decimateMesh(MeshType& mesh, int TargetVertexCount)
	{
		typedef OpenMesh::Decimater::DecimaterT<MeshType> Decimater;
		typedef OpenMesh::Decimater::ModQuadricT<MeshType>::Handle HModQuadric;
		Decimater decimater(mesh);
		HModQuadric hModQuadric;
		decimater.add(hModQuadric);
		decimater.module(hModQuadric).unset_max_err();

		decimater.initialize();
		decimater.decimate_to(TargetVertexCount);
		mesh.garbage_collection();
	}

	void extractRawFromOpenMesh(const MeshType& mesh, std::vector<Vertex>& outVertices, std::vector<unsigned int>& outIndices)
	{
		outVertices.clear();
		outIndices.clear();

		// Pre-allocate memory for efficiency
		outVertices.reserve(mesh.n_vertices() * 3);
		outIndices.reserve(mesh.n_faces() * 3);

		// Extract vertices
		for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			auto pt = mesh.point(*v_it);
			outVertices.push_back({ pt[0], pt[1], pt[2] });
		}

		// 2: Extract indices
		for (auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
		{
			// Loop over the 3 vertices of the face
			for (auto fv_it = mesh.cfv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			{
				// fv_it->idx() returns the integer of the vertex
				outIndices.push_back(fv_it->idx());
			}
		}
	}
}