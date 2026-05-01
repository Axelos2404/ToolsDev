#include "RetopoProcessor.h"
#include <Eigen/Core>
#include <igl/principal_curvature.h>

// Generate missing 3-column matrix math locally ---
#include <igl/local_basis.h>
#include <igl/local_basis.cpp>

// GMM++ Bug Bypas
// GMM++ accidentally references MUMPS_solve in an unused template, which you can see by going to the gmm_solver_Schwarz_additive.h file.
// This dummy declaration satisfies MSVC's strict parser without needing to install MUMPS.
// I dunno why it exists, or works like that, but it does, lol. Gotta love french devs.
namespace gmm {
    template <typename... Args>
    inline void MUMPS_solve(Args&&...) {}
}

#include <igl/copyleft/comiso/miq.h>
#include <igl/copyleft/comiso/miq.cpp>

namespace RetopoProcessor
{
    bool processRetopology(const std::vector<Vertex>& inVertices,
        const std::vector<unsigned int>& inIndices,
        std::vector<Vertex>& outVertices,
        std::vector<unsigned int>& outIndices,
        double scale)
    {
        if (inVertices.empty() || inIndices.empty()) return false;


        // Convert to LibIGL Format
        Eigen::MatrixXd V(inVertices.size(), 3);
        for (size_t i = 0; i < inVertices.size(); ++i) {
            V(i, 0) = inVertices[i].position[0];
            V(i, 1) = inVertices[i].position[1];
            V(i, 2) = inVertices[i].position[2];
        }

        Eigen::MatrixXi F(inIndices.size() / 3, 3);
        for (size_t i = 0; i < inIndices.size(); i += 3) {
            F(i / 3, 0) = inIndices[i];
            F(i / 3, 1) = inIndices[i + 1];
            F(i / 3, 2) = inIndices[i + 2];
        }

        // Calculate Surface Flow (Curvature)
        // These matrices hold the "Cross Field" - the invisible guide lines
        Eigen::MatrixXd PD1, PD2; // Principal Directions (The cross-field lines)
        Eigen::MatrixXd PV1, PV2; // Principal Values (How sharp the curves are)

        // We use a radius of 5 and 'use_k_ring=true' to smooth out noise
        igl::principal_curvature(V, F, PD1, PD2, PV1, PV2, 5, true);

        // CoMISo MIQ Solver
        Eigen::MatrixXd UV;   // The output parameters
        Eigen::MatrixXi F_UV; // The output topology

        // This is the heavy solver. It forces the cross-field to snap into integer grids.
        // stiffness = 5.0, direct_round = false, integer_spacing = 1
        igl::copyleft::comiso::miq(V, F, PD1, PD2, UV, F_UV, scale, 5.0, false, 1);

        // Convert Back to Your GPU Format
        outVertices.clear();
        outIndices.clear();

        // MIQ cuts mathematical "seams" into the mesh to unwrap it flat.
        // We must unroll our mesh (1 face = 3 unique vertices) so OpenGL renders these seams correctly.
        outVertices.reserve(F.rows() * 3);
        outIndices.reserve(F.rows() * 3);

        for (int i = 0; i < F.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                int v_idx = F(i, j);
                int uv_idx = F_UV(i, j);

                Vertex newV;
                // Original 3D Position
                newV.position[0] = V(v_idx, 0);
                newV.position[1] = V(v_idx, 1);
                newV.position[2] = V(v_idx, 2);

                // Original Normal
                newV.normal[0] = inVertices[v_idx].normal[0];
                newV.normal[1] = inVertices[v_idx].normal[1];
                newV.normal[2] = inVertices[v_idx].normal[2];

                // The CoMISo Quad-Grid Coordinates
                newV.texCoord[0] = static_cast<float>(UV(uv_idx, 0));
                newV.texCoord[1] = static_cast<float>(UV(uv_idx, 1));

                outVertices.push_back(newV);
                outIndices.push_back(static_cast<unsigned int>(outVertices.size() - 1));
            }
        }

        return true;
    }
}