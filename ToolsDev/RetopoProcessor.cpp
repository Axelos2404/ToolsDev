// 1. Windows and Standard Library Protections
#define NOMINMAX
#define _HAS_STD_BYTE 0
#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
#include <Windows.h>

// Instant Meshes' field.cpp expects this global variable to exist.
// Setting it to -1 tells TBB to use all available CPU cores.
int nprocs = -1;

// 2. The C++17 TBB Hack
// Old TBB relies on functions removed in C++17, so I injected dummy versions 
// here so the TBB headers compile flawlessly in current Visual Studio.
// I got this off of internet, but it seems to work fine and doesn't cause any issues with the rest of the code.
#if __cplusplus >= 201703L || _MSVC_LANG >= 201703L
namespace std {
    template <typename Arg1, typename Arg2, typename Result>
    struct binary_function {
        typedef Arg1 first_argument_type;
        typedef Arg2 second_argument_type;
        typedef Result result_type;
    };
    template <typename Arg, typename Result>
    struct unary_function {
        typedef Arg argument_type;
        typedef Result result_type;
    };
}
#endif

// 3. Project and Instant Meshes Includes
#include "RetopoProcessor.h"
#include <instant-meshes/src/common.h>
#include <instant-meshes/src/adjacency.h>
#include <instant-meshes/src/hierarchy.h>
#include <instant-meshes/src/field.h>
#include <instant-meshes/src/extract.h>
#include <instant-meshes/src/dedge.h>

// LibIGL for calculation (Normals and Areas)
#include <igl/per_vertex_normals.h>
#include <igl/doublearea.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/remove_unreferenced.h>
#include <igl/remove_duplicate_vertices.cpp>
#include <igl/remove_unreferenced.cpp>

static bool Fail(const char* msg)
{
    OutputDebugStringA("[Retopo] ");
    OutputDebugStringA(msg);
    OutputDebugStringA("\n");
    return false;
}

namespace RetopoProcessor {
    bool processRetopology(const std::vector<Vertex>& inVertices,
        const std::vector<unsigned int>& inIndices,
        std::vector<Vertex>& outVertices,
        std::vector<unsigned int>& outIndices,
        int targetVertexCount)
    {
        if (inVertices.empty() || inIndices.empty()) return Fail("Empty input mesh.");

        OutputDebugStringA("[Retopo] 1. Initializing and Welding Data Matrices...\n");

        // Load raw data into Eigen (Row-Major for LibIGL)
        Eigen::MatrixXd V_raw(inVertices.size(), 3);
        for (size_t i = 0; i < inVertices.size(); ++i) {
            V_raw.row(i) << (double)inVertices[i].position[0], (double)inVertices[i].position[1], (double)inVertices[i].position[2];
        }

        Eigen::MatrixXi F_raw(inIndices.size() / 3, 3);
        for (size_t i = 0; i < inIndices.size(); i += 3) {
            F_raw.row(i / 3) << (int)inIndices[i], (int)inIndices[i + 1], (int)inIndices[i + 2];
        }

        try {
            // STEP 2: WELD THE MESH (Crucial for cars)
            OutputDebugStringA("[Retopo] 2. Welding Seams...\n");
            Eigen::MatrixXd SV;          // Welded vertices
            Eigen::MatrixXi SF;          // Welded faces (Nx3)
            Eigen::VectorXi SVI, SVJ;    // Mapping vectors (Explicitly 1D)

            // Increased tolerance to 1e-4 to catch Assimp's split vertices
            igl::remove_duplicate_vertices(V_raw, F_raw, 1e-4, SV, SVI, SVJ, SF);

			Eigen::MatrixXd V_clean;    // Final cleaned vertex list (Row-Major)
			Eigen::MatrixXi F_clean;    // Final cleaned face list (Row-Major)
			Eigen::VectorXi I_unref;    // Mapping vector for unreferenced vertices (Explicitly 1D)
            igl::remove_unreferenced(SV, SF, V_clean, F_clean, I_unref);

            // Convert to Instant Meshes Format (Column-Major Float/UInt)
            MatrixXf V_im = V_clean.cast<float>().transpose();
            MatrixXu F_im = F_clean.cast<uint32_t>().transpose();

            OutputDebugStringA("[Retopo] 3. Calculating Fields...\n");
            Eigen::MatrixXd N_d;
            igl::per_vertex_normals(V_clean, F_clean, N_d);
            MatrixXf N_im = N_d.cast<float>().transpose();

            Eigen::VectorXd A_d;
            igl::doublearea(V_clean, F_clean, A_d);
            VectorXf A_im = (A_d.cast<float>() / 2.0f);

            // STEP 4: TOPOLOGY
            VectorXu V2E, E2E;
            VectorXb boundary, nonManifold;
            build_dedge(F_im, V_im, V2E, E2E, boundary, nonManifold, nullptr, true);
            AdjacencyMatrix adj = generate_adjacency_matrix_uniform(F_im, V2E, E2E, nonManifold);

            // STEP 5: HIERARCHY
            MultiResolutionHierarchy mRes;
            mRes.setV(std::move(V_im));
            mRes.setF(std::move(F_im));
            mRes.setE2E(std::move(E2E));
            mRes.setN(std::move(N_im));
            mRes.setA(std::move(A_im));
            mRes.setAdj(std::move(adj));

            Float totalArea = mRes.A().sum();
            Float target_scale = std::sqrt(totalArea / std::max(1, targetVertexCount));
            mRes.setScale(target_scale);

            mRes.build(false);
            mRes.resetSolution();

            // STEP 6: OPTIMIZE
            Optimizer opt(mRes, false);
            opt.setRoSy(4);
            opt.setPoSy(4);
            opt.setExtrinsic(true);

            opt.optimizeOrientations(-1); // -1 = full hierarchy
            opt.notify();
            opt.wait();

            opt.optimizePositions(-1);
            opt.notify();
            opt.wait();
            opt.shutdown();

            // STEP 7: EXTRACT
            std::vector<std::vector<TaggedLink>> adj_new;
            MatrixXf O_new, N_new, Nf_new;
            std::set<uint32_t> crease_in, crease_out;
            MatrixXu F_out;

            extract_graph(mRes, true, 4, 4, adj_new, O_new, N_new, crease_in, crease_out, false, true, true, true);
            extract_faces(adj_new, O_new, N_new, Nf_new, F_out, 4, target_scale, crease_out, true, true, nullptr, 2);

            // STEP 8: OUTPUT
            outVertices.clear();
            outIndices.clear();
            for (int i = 0; i < O_new.cols(); ++i) {
                Vertex v;
                v.position[0] = O_new(0, i); v.position[1] = O_new(1, i); v.position[2] = O_new(2, i);
                v.normal[0] = N_new(0, i); v.normal[1] = N_new(1, i); v.normal[2] = N_new(2, i);
                v.texCoord[0] = 0; v.texCoord[1] = 0;
                outVertices.push_back(v);
            }
            for (int i = 0; i < F_out.cols(); ++i) {
                outIndices.push_back(F_out(0, i)); outIndices.push_back(F_out(1, i)); outIndices.push_back(F_out(2, i));
                outIndices.push_back(F_out(0, i)); outIndices.push_back(F_out(2, i)); outIndices.push_back(F_out(3, i));
            }
        }
        catch (...) { return Fail("Processing failure."); }
        return true;
    }
}