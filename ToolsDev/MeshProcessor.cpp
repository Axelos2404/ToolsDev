#include "MeshProcessor.h"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

// including the Decimation tools from OpenMesh
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>
#include <vector>
#include <OpenMesh/Tools/HoleFiller/HoleFillerT.hh>
#include <chrono>
#include <map>
#include <tuple>
#include <cmath>

class ScopeTimer
{
public:
	explicit ScopeTimer(const char* name)
		: name_(name), start_(std::chrono::steady_clock::now())
	{
		const std::string message = std::string("[Mesh] ") + name_ + " start\n";
		OutputDebugStringA(message.c_str());
	}

	~ScopeTimer()
	{
		const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - start_).count();
		const std::string message = std::string("[Mesh] ") + name_ + " took " + std::to_string(elapsed) + " ms\n";
		OutputDebugStringA(message.c_str());
	}

private:
	const char* name_;
	std::chrono::steady_clock::time_point start_;
};

static void logBoundaryInfo(MeshType& mesh, const char* tag)
{
	std::vector<bool> visited(mesh.n_halfedges(), false);
	int loopCount = 0;
	int minLoop = INT_MAX;
	int maxLoop = 0;

	for (auto heh : mesh.halfedges())
	{
		if (!mesh.is_boundary(heh) || visited[heh.idx()])
			continue;

		int count = 0;
		auto start = heh;
		auto curr = heh;
		do
		{
			visited[curr.idx()] = true;
			++count;
			curr = mesh.next_halfedge_handle(curr);
		} while (curr.is_valid() && curr != start);

		loopCount++;
		minLoop = std::min(minLoop, count);
		maxLoop = std::max(maxLoop, count);
	}

	if (loopCount == 0) minLoop = 0;
	OutputDebugStringA(("[Mesh] " + std::string(tag) +
		" loops=" + std::to_string(loopCount) +
		" min=" + std::to_string(minLoop) +
		" max=" + std::to_string(maxLoop) + "\n").c_str());
}

static bool isManifoldMesh(const MeshType& mesh)
{
	for (auto vh : mesh.vertices())
	{
		if (!mesh.is_manifold(vh))
			return false;
	}
	return true;
}

static bool hasOnlyValidBoundaryLoops(MeshType& mesh)
{
	std::vector<bool> visited(mesh.n_halfedges(), false);

	for (auto heh : mesh.halfedges())
	{
		if (!mesh.is_boundary(heh) || visited[heh.idx()])
			continue;

		int count = 0;
		auto start = heh;
		auto curr = heh;

		do
		{
			visited[curr.idx()] = true;
			++count;
			curr = mesh.next_halfedge_handle(curr);
		} while (curr.is_valid() && curr != start);

		if (count < 3)
			return false;
	}
	return true;
}

static void removeNonManifoldVertices(MeshType& mesh)
{
	mesh.request_face_status();
	mesh.request_edge_status();
	mesh.request_vertex_status();
	mesh.request_halfedge_status();

	int totalRemoved = 0;
	bool changed = true;

	while (changed)
	{
		changed = false;
		std::vector<MeshType::VertexHandle> badVerts;

		for (auto vh : mesh.vertices())
		{
			if (!mesh.status(vh).deleted() && !mesh.is_manifold(vh))
			{
				badVerts.push_back(vh);
			}
		}

		for (auto vh : badVerts)
		{
			if (!mesh.status(vh).deleted())
			{
				// Deleting the vertex automatically removes incident faces and edges safely
				mesh.delete_vertex(vh, false);
				changed = true;
				totalRemoved++;
			}
		}
	}

	if (totalRemoved > 0)
		mesh.garbage_collection();

	OutputDebugStringA(("[Mesh] removed non-manifold vertices=" + std::to_string(totalRemoved) + "\n").c_str());
}

static bool safeFillAllHoles(MeshType& mesh)
{
	if (!isManifoldMesh(mesh))
	{
		OutputDebugStringA("[Mesh] Hole fill skipped: non-manifold mesh\n");
		return false;
	}

	if (!hasOnlyValidBoundaryLoops(mesh))
	{
		OutputDebugStringA("[Mesh] Hole fill skipped: boundary loop < 3\n");
		return false;
	}

	OpenMesh::HoleFiller::HoleFillerT<MeshType> filler(mesh);
	filler.fill_all_holes();
	OutputDebugStringA("[Mesh] Hole fill completed\n");
	return true;
}

namespace MeshProcessor
{
	MeshType convertRawToOpenMesh(const ModelData& model)
	{
		MeshType mesh;
		std::map<std::tuple<long long, long long, long long>, MeshType::VertexHandle> vertexMap;

		const float tolerance = 10000.0f; // Welds vertices that are virtually identical in position

		for (const auto& meshData : model.meshes)
		{
			std::vector<MeshType::VertexHandle> localHandles;
			localHandles.reserve(meshData.vertices.size());

			for (const auto& v : meshData.vertices)
			{
				long long kx = static_cast<long long>(std::round(v.position[0] * tolerance));
				long long ky = static_cast<long long>(std::round(v.position[1] * tolerance));
				long long kz = static_cast<long long>(std::round(v.position[2] * tolerance));
				auto key = std::make_tuple(kx, ky, kz);

				auto it = vertexMap.find(key);
				if (it == vertexMap.end())
				{
					auto vh = mesh.add_vertex(MeshType::Point(v.position[0], v.position[1], v.position[2]));
					vertexMap[key] = vh;
					localHandles.push_back(vh);
				}
				else
				{
					localHandles.push_back(it->second);
				}
			}

			for (size_t i = 0; i < meshData.indices.size(); i += 3)
			{
				MeshType::VertexHandle v0 = localHandles[meshData.indices[i]];
				MeshType::VertexHandle v1 = localHandles[meshData.indices[i + 1]];
				MeshType::VertexHandle v2 = localHandles[meshData.indices[i + 2]];

				// Prevent degenerate faces from being added
				if (v0 != v1 && v1 != v2 && v2 != v0)
				{
					mesh.add_face(v0, v1, v2);
				}
			}
		}
		return mesh;
	}

	void decimateMesh(MeshType& mesh, int TargetVertexCount)
	{
		ScopeTimer decimateTimer("DecimateMesh");
		typedef OpenMesh::Decimater::DecimaterT<MeshType> Decimater;
		typedef OpenMesh::Decimater::ModQuadricT<MeshType>::Handle HModQuadric;
		Decimater decimater(mesh);
		HModQuadric hModQuadric;
		decimater.add(hModQuadric);
		decimater.module(hModQuadric).unset_max_err();

		{
			ScopeTimer stepTimer("Decimater.initialize");
			decimater.initialize();
		}
		{
			ScopeTimer stepTimer("Decimater.decimate_to");
			decimater.decimate_to(TargetVertexCount);
		}
		{
			ScopeTimer stepTimer("Mesh.garbage_collection");
			mesh.garbage_collection();
		}
		{
			ScopeTimer stepTimer("RemoveNonManifoldVertices");
			removeNonManifoldVertices(mesh);
		}
		{
			ScopeTimer stepTimer("LogBoundaryInfo.before");
			logBoundaryInfo(mesh, "before hole fill");
		}
		{
			ScopeTimer stepTimer("SafeFillAllHoles");
			safeFillAllHoles(mesh);
		}
		{
			ScopeTimer stepTimer("LogBoundaryInfo.after");
			logBoundaryInfo(mesh, "after hole fill");
		}

		// Recalculate lighting normals for the new low-poly shape
		{
			ScopeTimer stepTimer("Mesh.update_normals");
			mesh.request_face_normals();
			mesh.request_vertex_normals();
			mesh.update_normals();
		}
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