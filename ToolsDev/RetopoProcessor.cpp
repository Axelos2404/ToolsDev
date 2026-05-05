// 1. Windows and Standard Library Protections
#define NOMINMAX
#define _HAS_STD_BYTE 0
#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
#include <Windows.h>

// Instant Meshes' field.cpp expects this global variable to exist.
// Setting it to -1 tells TBB to use all available CPU cores.
int nprocs = -1;

// 2. The C++17 TBB Hack
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

        Eigen::MatrixXd V_raw(inVertices.size(), 3);
        for (size_t i = 0; i < inVertices.size(); ++i) {
            V_raw.row(i) << (double)inVertices[i].position[0], (double)inVertices[i].position[1], (double)inVertices[i].position[2];
        }

        Eigen::MatrixXi F_raw(inIndices.size() / 3, 3);
        for (size_t i = 0; i < inIndices.size(); i += 3) {
            F_raw.row(i / 3) << (int)inIndices[i], (int)inIndices[i + 1], (int)inIndices[i + 2];
        }

        try {
            // WELD LOCALLY
            Eigen::MatrixXd SV;
            Eigen::MatrixXi SF;
            Eigen::VectorXi SVI, SVJ;
            igl::remove_duplicate_vertices(V_raw, F_raw, 1e-7, SV, SVI, SVJ, SF);

            Eigen::MatrixXd V_clean;
            Eigen::MatrixXi F_clean;
            Eigen::VectorXi I_unref;
            igl::remove_unreferenced(SV, SF, V_clean, F_clean, I_unref);

            MatrixXf V_im = V_clean.cast<float>().transpose();
            MatrixXu F_im = F_clean.cast<uint32_t>().transpose();

            Eigen::MatrixXd N_d;
            igl::per_vertex_normals(V_clean, F_clean, N_d);
            MatrixXf N_im = N_d.cast<float>().transpose();

            Eigen::VectorXd A_d;
            igl::doublearea(V_clean, F_clean, A_d);
            VectorXf A_im = (A_d.cast<float>() / 2.0f);

            VectorXu V2E, E2E;
            VectorXb boundary, nonManifold;
            build_dedge(F_im, V_im, V2E, E2E, boundary, nonManifold, nullptr, true);
            AdjacencyMatrix adj = generate_adjacency_matrix_uniform(F_im, V2E, E2E, nonManifold);

            MultiResolutionHierarchy mRes;
            mRes.setV(std::move(V_im));
            mRes.setF(std::move(F_im));
            mRes.setE2E(std::move(E2E));
            mRes.setN(std::move(N_im));
            mRes.setA(std::move(A_im));
            mRes.setAdj(std::move(adj));

            // DYNAMIC SCALE CALCULATION PER-PART
            Float totalArea = mRes.A().sum();
            Float target_scale = std::sqrt(totalArea / std::max(1, targetVertexCount));
            mRes.setScale(target_scale);

            mRes.build(false);
            mRes.resetSolution();

            Optimizer opt(mRes, false);
            opt.setRoSy(4);
            opt.setPoSy(4);
            opt.setExtrinsic(true);

            opt.optimizeOrientations(-1);
            opt.notify(); opt.wait();

            opt.optimizePositions(-1);
            opt.notify(); opt.wait();
            opt.shutdown();

            std::vector<std::vector<TaggedLink>> adj_new;
            MatrixXf O_new, N_new, Nf_new;
            std::set<uint32_t> crease_in, crease_out;
            MatrixXu F_out;

            // --- THE FIX: PRESERVE SUBMESH BOUNDARIES ---
            // This locks the outer border of the part so it doesn't shrink-wrap and erode.
            for (uint32_t i = 0; i < E2E.size(); ++i) {
                if (boundary[i]) {
                    crease_in.insert(i);
                }
            }
            // --------------------------------------------

            extract_graph(mRes, true, 4, 4, adj_new, O_new, N_new, crease_in, crease_out, false, true, true, true);

            // Extract using the local target_scale
            extract_faces(adj_new, O_new, N_new, Nf_new, F_out, 4, target_scale, crease_out, false, true, nullptr, 2);

            outVertices.clear();
            outIndices.clear();
            for (int i = 0; i < O_new.cols(); ++i) {
                Vertex v;
                v.position[0] = O_new(0, i); v.position[1] = O_new(1, i); v.position[2] = O_new(2, i);
                v.normal[0] = N_new(0, i); v.normal[1] = N_new(1, i); v.normal[2] = N_new(2, i);
                v.texCoord[0] = 0; v.texCoord[1] = 0;
                outVertices.push_back(v);
            }

            // DYNAMIC N-GON PARSER
            for (int i = 0; i < F_out.cols(); ++i) {
                std::vector<uint32_t> faceVerts;

                // Safely read only the rows that actually exist in the matrix
                for (int r = 0; r < F_out.rows(); ++r) {
                    uint32_t v = F_out(r, i);
                    // Ignore Instant Meshes' padding (-1)
                    if (v != (uint32_t)-1) {
                        faceVerts.push_back(v);
                    }
                }

                if (faceVerts.size() < 3) continue; // Skip degenerate lines/points

                if (faceVerts.size() == 3) {
                    // Standard Triangle
                    outIndices.push_back(faceVerts[0]);
                    outIndices.push_back(faceVerts[1]);
                    outIndices.push_back(faceVerts[2]);
                }
                else if (faceVerts.size() == 4) {
                    // Standard Quad (Split perfectly down the diagonal for OpenGL)
                    outIndices.push_back(faceVerts[0]);
                    outIndices.push_back(faceVerts[1]);
                    outIndices.push_back(faceVerts[2]);

                    outIndices.push_back(faceVerts[2]);
                    outIndices.push_back(faceVerts[3]);
                    outIndices.push_back(faceVerts[0]);
                }
                else {
                    // N-Gon (Pentagon/Hexagon) Triangulation Fan
                    for (size_t v = 1; v + 1 < faceVerts.size(); ++v) {
                        outIndices.push_back(faceVerts[0]);
                        outIndices.push_back(faceVerts[v]);
                        outIndices.push_back(faceVerts[v + 1]);
                    }
                }
            }
        }
        catch (...) { return Fail("Processing failure."); }
        return true;
    }
}