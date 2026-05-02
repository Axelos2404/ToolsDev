#include "RetopoProcessor.h"
#include <Eigen/Core>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/per_vertex_normals.h>
#include <igl/remove_unreferenced.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/resolve_duplicated_faces.h>
#include <igl/principal_curvature.h>
#include <igl/doublearea.h>
#include <igl/boundary_loop.h>
#include <map>
#include <utility>
#include <Windows.h>

// --- LibIGL Linker Fix ---
#include <igl/local_basis.h>
#undef IGL_STATIC_LIBRARY
#include <igl/local_basis.cpp>
#define IGL_STATIC_LIBRARY

// --- CoMISo Linker Fix ---
namespace gmm {
    template <typename... Args>
    inline void MUMPS_solve(Args&&...) {}
}

#ifndef COMISO_GMM_AVAILABLE
#define COMISO_GMM_AVAILABLE 1
#endif
#ifndef INCLUDE_TEMPLATES
#define INCLUDE_TEMPLATES
#endif

#include <comiso/Solver/ConstrainedSolver.hh>
#include <comiso/Solver/ConstrainedSolver.cc>

// --- Load MIQ ---
#include <igl/copyleft/comiso/miq.h>
#include <igl/copyleft/comiso/miq.cpp>

static bool Fail(const char* msg)
{
    OutputDebugStringA("[Retopo] ");
    OutputDebugStringA(msg);
    OutputDebugStringA("\n");
    return false;
}

static void RemoveDegenerateFaces(Eigen::MatrixXd& V, Eigen::MatrixXi& F, double minDblArea)
{
    Eigen::VectorXd dblA;
    igl::doublearea(V, F, dblA);

    Eigen::MatrixXi F_out(F.rows(), 3);
    int fc = 0;
    for (int i = 0; i < F.rows(); ++i)
    {
        if (dblA(i) > minDblArea)
            F_out.row(fc++) = F.row(i);
    }
    F_out.conservativeResize(fc, 3);

    Eigen::VectorXi I;
    igl::remove_unreferenced(V, F_out, V, F, I);
}

static bool SanitizeFaces(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    if (F.rows() == 0 || V.rows() == 0) return false;

    const int vcount = static_cast<int>(V.rows());
    Eigen::MatrixXi F_tmp(F.rows(), 3);
    int fc = 0;

    for (int i = 0; i < F.rows(); ++i)
    {
        int a = F(i, 0), b = F(i, 1), c = F(i, 2);
        if (a < 0 || b < 0 || c < 0) continue;
        if (a >= vcount || b >= vcount || c >= vcount) continue;
        if (a == b || b == c || c == a) continue;
        F_tmp.row(fc++) = F.row(i);
    }
    F_tmp.conservativeResize(fc, 3);

    if (F_tmp.rows() == 0) return false;

    Eigen::VectorXd dblA;
    igl::doublearea(V, F_tmp, dblA);

    Eigen::MatrixXi F_out(F_tmp.rows(), 3);
    fc = 0;
    for (int i = 0; i < F_tmp.rows(); ++i)
    {
        if (dblA(i) > 1e-12)
            F_out.row(fc++) = F_tmp.row(i);
    }
    F_out.conservativeResize(fc, 3);

    Eigen::VectorXi I;
    igl::remove_unreferenced(V, F_out, V, F, I);
    return (F.rows() > 0 && V.rows() > 0);
}

static bool ValidateMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    if (!V.allFinite()) return false;

    const int vcount = static_cast<int>(V.rows());
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx = F(i, j);
            if (idx < 0 || idx >= vcount) return false;
        }
    }
    return true;
}

namespace RetopoProcessor
{
    bool processRetopology(const std::vector<Vertex>& inVertices,
        const std::vector<unsigned int>& inIndices,
        std::vector<Vertex>& outVertices,
        std::vector<unsigned int>& outIndices,
        double scale)
    {
        if (inVertices.empty() || inIndices.empty()) return Fail("empty input");

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

        if (V.rows() == 0 || F.rows() == 0) return Fail("empty V/F");

        for (int i = 0; i < F.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                if (F(i, j) < 0 || F(i, j) >= V.rows()) F(i, j) = 0;
            }
        }

        Eigen::MatrixXd V_merged;
        Eigen::MatrixXi F_merged;
        Eigen::VectorXi SVI, SVJ;
        igl::remove_duplicate_vertices(V, F, 1e-7, V_merged, SVI, SVJ, F_merged);

        Eigen::MatrixXi F_valid(F_merged.rows(), 3);
        int valid_count = 0;
        for (int i = 0; i < F_merged.rows(); ++i) {
            if (F_merged(i, 0) != F_merged(i, 1) &&
                F_merged(i, 1) != F_merged(i, 2) &&
                F_merged(i, 2) != F_merged(i, 0)) {
                F_valid.row(valid_count++) = F_merged.row(i);
            }
        }
        F_valid.conservativeResize(valid_count, 3);

        Eigen::MatrixXi F_resolved;
        Eigen::VectorXi J_faces;
        igl::resolve_duplicated_faces(F_valid, F_resolved, J_faces);

        Eigen::MatrixXd V_clean;
        Eigen::MatrixXi F_clean;
        Eigen::VectorXi I_unref;
        igl::remove_unreferenced(V_merged, F_resolved, V_clean, F_clean, I_unref);

        if (V_clean.rows() < 10 || F_clean.rows() < 10) return Fail("too few verts/faces after resolve");

        // --- 5. Island Killer ---
        /*std::vector<std::vector<int>> adj(V_clean.rows());
        for (int i = 0; i < F_clean.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                adj[F_clean(i, j)].push_back(F_clean(i, (j + 1) % 3));
                adj[F_clean(i, (j + 1) % 3)].push_back(F_clean(i, j));
            }
        }

        std::vector<int> comp_id(V_clean.rows(), -1);
        std::vector<int> comp_size;
        int current_comp = 0;

        for (int i = 0; i < V_clean.rows(); ++i) {
            if (comp_id[i] == -1) {
                int size = 0;
                std::vector<int> q;
                q.push_back(i);
                comp_id[i] = current_comp;
                int head = 0;
                while (head < q.size()) {
                    int curr = q[head++];
                    size++;
                    for (int n : adj[curr]) {
                        if (comp_id[n] == -1) {
                            comp_id[n] = current_comp;
                            q.push_back(n);
                        }
                    }
                }
                comp_size.push_back(size);
                current_comp++;
            }
        }

        int max_comp = 0, max_size = 0;
        for (int i = 0; i < comp_size.size(); ++i) {
            if (comp_size[i] > max_size) {
                max_size = comp_size[i];
                max_comp = i;
            }
        }

        Eigen::MatrixXi F_largest(F_clean.rows(), 3);
        int l_count = 0;
        for (int i = 0; i < F_clean.rows(); ++i) {
            if (comp_id[F_clean(i, 0)] == max_comp) {
                F_largest.row(l_count++) = F_clean.row(i);
            }
        }
        F_largest.conservativeResize(l_count, 3);

        Eigen::MatrixXd V_temp;
        Eigen::MatrixXi F_temp;
        igl::remove_unreferenced(V_clean, F_largest, V_temp, F_temp, I_unref);
        V_clean = V_temp;
        F_clean = F_temp;

        if (V_clean.rows() < 10) return Fail("too few verts after island removal");*/

        // --- 5. Island Bouncer (Check for disconnected components) ---
        std::vector<std::vector<int>> adj(V_clean.rows());
        for (int i = 0; i < F_clean.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                adj[F_clean(i, j)].push_back(F_clean(i, (j + 1) % 3));
                adj[F_clean(i, (j + 1) % 3)].push_back(F_clean(i, j));
            }
        }

        std::vector<int> comp_id(V_clean.rows(), -1);
        int current_comp = 0;

        for (int i = 0; i < V_clean.rows(); ++i) {
            if (comp_id[i] == -1) {
                // If we find a second disconnected island, MIQ will crash. Abort safely.
                if (current_comp > 0) {
                    return Fail("Multiple disconnected islands detected; MIQ not safe");
                }

                std::vector<int> q;
                q.push_back(i);
                comp_id[i] = current_comp;
                int head = 0;
                while (head < q.size()) {
                    int curr = q[head++];
                    for (int n : adj[curr]) {
                        if (comp_id[n] == -1) {
                            comp_id[n] = current_comp;
                            q.push_back(n);
                        }
                    }
                }
                current_comp++;
            }
        }

        // --- EXTRA CLEANUP + MANIFOLD SAFETY ---
        const int vcount = static_cast<int>(V_clean.rows());

        Eigen::MatrixXi F_noDeg(F_clean.rows(), 3);
        int fcount = 0;
        for (int i = 0; i < F_clean.rows(); ++i)
        {
            int a = F_clean(i, 0), b = F_clean(i, 1), c = F_clean(i, 2);
            if (a < 0 || b < 0 || c < 0 || a >= vcount || b >= vcount || c >= vcount)
                return Fail("invalid index after island removal");
            if (a == b || b == c || c == a) continue;
            F_noDeg.row(fcount++) = F_clean.row(i);
        }
        F_noDeg.conservativeResize(fcount, 3);

        Eigen::MatrixXd V_final;
        Eigen::MatrixXi F_final;
        Eigen::VectorXi I_final;
        igl::remove_unreferenced(V_clean, F_noDeg, V_final, F_final, I_final);

        V_clean = V_final;
        F_clean = F_final;

        if (V_clean.rows() < 10 || F_clean.rows() < 10) return Fail("too few verts/faces after cleanup");

        // --- 6. TOPOLOGY SAFETY FILTER ---
        int fin_count = 0;
        std::map<std::pair<int, int>, int> edge_counts;
        for (int i = 0; i < F_clean.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                int v1 = F_clean(i, j);
                int v2 = F_clean(i, (j + 1) % 3);
                if (v1 > v2) std::swap(v1, v2);
                edge_counts[{v1, v2}]++;
            }
        }
        for (const auto& edge : edge_counts) {
            if (edge.second > 2) fin_count++;
        }
        if (fin_count > (F_clean.rows() * 0.3)) return Fail("too many non-manifold edges");

        std::vector<std::vector<int>> v_to_f(V_clean.rows());
        for (int i = 0; i < F_clean.rows(); ++i) {
            v_to_f[F_clean(i, 0)].push_back(i);
            v_to_f[F_clean(i, 1)].push_back(i);
            v_to_f[F_clean(i, 2)].push_back(i);
        }

        // Split bowtie vertices instead of deleting faces
        const int original_vcount = static_cast<int>(V_clean.rows());
        std::vector<Eigen::Vector3d> V_vec(original_vcount);
        for (int i = 0; i < original_vcount; ++i)
            V_vec[i] = V_clean.row(i);

        bool split_any = false;

        for (int v = 0; v < original_vcount; ++v) {
            const auto& incident_faces = v_to_f[v];
            if (incident_faces.empty()) continue;

            std::map<int, std::vector<int>> face_adj;
            for (size_t i = 0; i < incident_faces.size(); ++i) {
                for (size_t j = i + 1; j < incident_faces.size(); ++j) {
                    int fi = incident_faces[i];
                    int fj = incident_faces[j];
                    for (int k = 0; k < 3; ++k) {
                        for (int l = 0; l < 3; ++l) {
                            if (F_clean(fi, k) != v && F_clean(fi, k) == F_clean(fj, l)) {
                                face_adj[fi].push_back(fj);
                                face_adj[fj].push_back(fi);
                            }
                        }
                    }
                }
            }

            std::map<int, bool> visited;
            std::map<int, int> face_comp;
            int components = 0;

            for (int fi : incident_faces) {
                if (!visited[fi]) {
                    std::vector<int> q = { fi };
                    visited[fi] = true;
                    face_comp[fi] = components;
                    int head = 0;
                    while (head < q.size()) {
                        int curr = q[head++];
                        for (int nbr : face_adj[curr]) {
                            if (!visited[nbr]) {
                                visited[nbr] = true;
                                face_comp[nbr] = components;
                                q.push_back(nbr);
                            }
                        }
                    }
                    components++;
                }
            }

            if (components > 1)
            {
                OutputDebugStringA("[Retopo] bowtie vertex detected -> splitting vertex\n");
                std::vector<int> comp_to_new(components, v);
                for (int c = 1; c < components; ++c)
                {
                    V_vec.push_back(V_vec[v]);
                    comp_to_new[c] = static_cast<int>(V_vec.size() - 1);
                }

                for (int fi : incident_faces)
                {
                    int comp = face_comp[fi];
                    int new_v = comp_to_new[comp];
                    if (new_v == v) continue;
                    for (int k = 0; k < 3; ++k)
                    {
                        if (F_clean(fi, k) == v) F_clean(fi, k) = new_v;
                    }
                }
                split_any = true;
            }
        }

        if (split_any)
        {
            Eigen::MatrixXd V_new(static_cast<int>(V_vec.size()), 3);
            for (int i = 0; i < static_cast<int>(V_vec.size()); ++i)
                V_new.row(i) = V_vec[i].transpose();
            V_clean = V_new;
        }

        // --- VALENCE SAFETY FILTER (pre-curvature) ---
        std::vector<int> valence(V_clean.rows(), 0);
        for (int i = 0; i < F_clean.rows(); ++i)
        {
            valence[F_clean(i, 0)]++;
            valence[F_clean(i, 1)]++;
            valence[F_clean(i, 2)]++;
        }

        // Remove faces connected to low-valence vertices
        Eigen::MatrixXi F_valence(F_clean.rows(), 3);
        int fcount3 = 0;
        for (int i = 0; i < F_clean.rows(); ++i)
        {
            int a = F_clean(i, 0), b = F_clean(i, 1), c = F_clean(i, 2);
            if (valence[a] < 3 || valence[b] < 3 || valence[c] < 3) continue;
            F_valence.row(fcount3++) = F_clean.row(i);
        }
        F_valence.conservativeResize(fcount3, 3);

        Eigen::MatrixXd V_final3;
        Eigen::MatrixXi F_final3;
        Eigen::VectorXi I_final3;
        igl::remove_unreferenced(V_clean, F_valence, V_final3, F_final3, I_final3);

        V_clean = V_final3;
        F_clean = F_final3;

        if (V_clean.rows() < 10 || F_clean.rows() < 10)
            return Fail("too few verts/faces after valence filter");

        // --- EDGE MANIFOLD CLEANUP (drop faces around non-manifold edges) ---
        bool changed = true;
        while (changed)
        {
            changed = false;

            std::map<std::pair<int, int>, std::vector<int>> edge_faces;
            for (int i = 0; i < F_clean.rows(); ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    int a = F_clean(i, j);
                    int b = F_clean(i, (j + 1) % 3);
                    if (a > b) std::swap(a, b);
                    edge_faces[{a, b}].push_back(i);
                }
            }

            std::vector<bool> face_keep(F_clean.rows(), true);
            for (const auto& kv : edge_faces)
            {
                if (kv.second.size() > 2)
                {
                    for (int fi : kv.second)
                        face_keep[fi] = false;
                    changed = true;
                }
            }

            if (changed)
            {
                Eigen::MatrixXi F_tmp(F_clean.rows(), 3);
                int fc = 0;
                for (int i = 0; i < F_clean.rows(); ++i)
                {
                    if (face_keep[i]) F_tmp.row(fc++) = F_clean.row(i);
                }
                F_tmp.conservativeResize(fc, 3);

                Eigen::MatrixXd V_tmp;
                Eigen::MatrixXi F_tmp2;
                Eigen::VectorXi I_tmp;
                igl::remove_unreferenced(V_clean, F_tmp, V_tmp, F_tmp2, I_tmp);

                V_clean = V_tmp;
                F_clean = F_tmp2;

                if (V_clean.rows() < 10 || F_clean.rows() < 10)
                    return Fail("too few verts/faces after edge-manifold cleanup");
            }
        }

        // --- VERTEX MANIFOLD CLEANUP (drop faces around non-manifold vertices) ---
        bool v_changed = true;
        while (v_changed)
        {
            v_changed = false;

            std::vector<std::vector<int>> v_to_f2(V_clean.rows());
            for (int i = 0; i < F_clean.rows(); ++i)
            {
                v_to_f2[F_clean(i, 0)].push_back(i);
                v_to_f2[F_clean(i, 1)].push_back(i);
                v_to_f2[F_clean(i, 2)].push_back(i);
            }

            std::vector<bool> face_keep(F_clean.rows(), true);
            for (int v = 0; v < V_clean.rows(); ++v)
            {
                const auto& incident_faces = v_to_f2[v];
                if (incident_faces.size() < 2) continue;

                std::map<int, std::vector<int>> face_adj;
                for (size_t i = 0; i < incident_faces.size(); ++i) {
                    for (size_t j = i + 1; j < incident_faces.size(); ++j) {
                        int fi = incident_faces[i];
                        int fj = incident_faces[j];
                        for (int k = 0; k < 3; ++k) {
                            for (int l = 0; l < 3; ++l) {
                                if (F_clean(fi, k) != v && F_clean(fi, k) == F_clean(fj, l)) {
                                    face_adj[fi].push_back(fj);
                                    face_adj[fj].push_back(fi);
                                }
                            }
                        }
                    }
                }

                std::map<int, bool> visited;
                int components = 0;
                for (int fi : incident_faces) {
                    if (!visited[fi]) {
                        components++;
                        std::vector<int> q = { fi };
                        visited[fi] = true;
                        int head = 0;
                        while (head < q.size()) {
                            int curr = q[head++];
                            for (int nbr : face_adj[curr]) {
                                if (!visited[nbr]) {
                                    visited[nbr] = true;
                                    q.push_back(nbr);
                                }
                            }
                        }
                    }
                }

                if (components > 1)
                {
                    for (int fi : incident_faces) face_keep[fi] = false;
                    v_changed = true;
                }
            }

            if (v_changed)
            {
                Eigen::MatrixXi F_tmp(F_clean.rows(), 3);
                int fc = 0;
                for (int i = 0; i < F_clean.rows(); ++i)
                {
                    if (face_keep[i]) F_tmp.row(fc++) = F_clean.row(i);
                }
                F_tmp.conservativeResize(fc, 3);

                Eigen::MatrixXd V_tmp;
                Eigen::MatrixXi F_tmp2;
                Eigen::VectorXi I_tmp;
                igl::remove_unreferenced(V_clean, F_tmp, V_tmp, F_tmp2, I_tmp);

                V_clean = V_tmp;
                F_clean = F_tmp2;

                if (V_clean.rows() < 10 || F_clean.rows() < 10)
                    return Fail("too few verts/faces after vertex-manifold cleanup");
            }
        }

        // --- MANIFOLD VALIDATION (required before MIQ) ---
        if (!SanitizeFaces(V_clean, F_clean))
            return Fail("invalid/degenerate faces after sanitize");

        if (!igl::is_edge_manifold(F_clean))
            return Fail("edge non-manifold after cleanup");

        if (!igl::is_vertex_manifold(F_clean))
            return Fail("vertex non-manifold after cleanup");

        // DEBUG COUNTS (add here)
        OutputDebugStringA(("[Retopo] V=" + std::to_string(V_clean.rows()) +
            " F=" + std::to_string(F_clean.rows()) + "\n").c_str());

        // --- Boundary guard: MIQ is unstable on open meshes ---
        std::vector<std::vector<int>> loops;
        igl::boundary_loop(F_clean, loops);

        // DEBUG: boundary info (add here)
        OutputDebugStringA(("[Retopo] boundary loops=" + std::to_string(loops.size()) + "\n").c_str());
        for (size_t i = 0; i < loops.size(); ++i)
        {
            OutputDebugStringA(("[Retopo] loop[" + std::to_string(i) + "] size=" +
                std::to_string(loops[i].size()) + "\n").c_str());
        }

        if (!loops.empty())
            return Fail("mesh has boundaries; MIQ not safe");

        // --- Final validity guard ---
        if (!ValidateMesh(V_clean, F_clean))
            return Fail("non-finite verts or invalid indices");

        // --- SIZE GUARD (prevent std::bad_alloc in MIQ) ---
        constexpr int kMaxVerts = 200000;
        constexpr int kMaxFaces = 400000;
        if (V_clean.rows() > kMaxVerts || F_clean.rows() > kMaxFaces)
            return Fail("mesh too large for MIQ; reduce target vertex count");

        // --- 7. THE FINAL MATH ---
        Eigen::MatrixXd PD1(V_clean.rows(), 3);
        Eigen::MatrixXd PD2(V_clean.rows(), 3);
        Eigen::MatrixXd PV1(V_clean.rows(), 1);
        Eigen::MatrixXd PV2(V_clean.rows(), 1);

        if (V_clean.rows() < 10 || F_clean.rows() < 10) return Fail("too few verts/faces before curvature");

        for (int i = 0; i < F_clean.rows(); ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                int idx = F_clean(i, j);
                if (idx < 0 || idx >= V_clean.rows()) return Fail("invalid face index before curvature");
            }
        }

        Eigen::MatrixXd N;
        igl::per_vertex_normals(V_clean, F_clean, N);

        // Build a stable tangent frame per vertex
        for (int i = 0; i < V_clean.rows(); ++i)
        {
            Eigen::Vector3d n = N.row(i).normalized();
            Eigen::Vector3d t1 = n.unitOrthogonal();
            Eigen::Vector3d t2 = n.cross(t1).normalized();

            PD1.row(i) = t1.transpose();
            PD2.row(i) = t2.transpose();
            PV1(i, 0) = 0.0;
            PV2(i, 0) = 0.0;
        }

        Eigen::MatrixXd UV;
        Eigen::MatrixXi F_UV;

        igl::copyleft::comiso::miq(V_clean, F_clean, PD1, PD2, UV, F_UV, scale, 5.0, false, 1);

        if (UV.rows() == 0 || F_UV.rows() == 0) return Fail("miq produced empty UV");

        outVertices.clear();
        outIndices.clear();

        outVertices.reserve(F_clean.rows() * 3);
        outIndices.reserve(F_clean.rows() * 3);

        for (int i = 0; i < F_clean.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                int v_idx = F_clean(i, j);
                int uv_idx = F_UV(i, j);

                Vertex newV;
                newV.position[0] = V_clean(v_idx, 0);
                newV.position[1] = V_clean(v_idx, 1);
                newV.position[2] = V_clean(v_idx, 2);

                newV.normal[0] = 0;
                newV.normal[1] = 1;
                newV.normal[2] = 0;

                newV.texCoord[0] = static_cast<float>(UV(uv_idx, 0));
                newV.texCoord[1] = static_cast<float>(UV(uv_idx, 1));

                outVertices.push_back(newV);
                outIndices.push_back(static_cast<unsigned int>(outVertices.size() - 1));
            }
        }

        return true;
    }
}