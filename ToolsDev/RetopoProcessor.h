#pragma once

#include <vector>
#include "Mesh.h"

namespace RetopoProcessor
{
	bool processRetopology(const std::vector<Vertex>& inVertices, const std::vector<unsigned int>& inIndices,
		std::vector<Vertex>& outVertices, std::vector<unsigned int>& outIndices,
		double scale = 30.0);
}

