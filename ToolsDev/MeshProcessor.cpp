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

		// Recalculate lighting normals for the new low-poly shape
		mesh.request_face_normals();
		mesh.request_vertex_normals();
		mesh.update_normals();
	}

	void extractRawFromOpenMesh(const MeshType& mesh, std::vector<Vertex>& outVertices, std::vector<unsigned int>& outIndices)
	{
		outVertices.clear();
		outIndices.clear();

		outVertices.reserve(mesh.n_vertices() * 3);
		outIndices.reserve(mesh.n_faces() * 3);

		for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			auto pt = mesh.point(*v_it);
			auto n = mesh.normal(*v_it); // Grab the newly calculated normal

			// Pack the position, normal, and an empty UV coordinate into your struct
			outVertices.push_back({ {pt[0], pt[1], pt[2]}, {n[0], n[1], n[2]}, {0.0f, 0.0f} });
		}

		for (auto f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
		{
			for (auto fv_it = mesh.cfv_iter(*f_it); fv_it.is_valid(); ++fv_it)
			{
				outIndices.push_back(fv_it->idx());
			}
		}
	}
}