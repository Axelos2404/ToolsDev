#include <CoMISo/Config/config_suitesparse.hh>
#include <CoMISo/Config/config.hh>
#include <CoMISo/Utils/MatrixDecomposition.hh>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

#if COMISO_SUITESPARSE_CHOLMOD_AVAILABLE
#  include <Eigen/CholmodSupport>
#endif

#if COMISO_SUITESPARSE_UMFPACK_AVAILABLE
#  include <Eigen/UmfPackSupport>
#endif

#if COMISO_METIS_AVAILABLE
#  include <iostream> // EigenMetisSupport.h uses std::cerr without including iostream
#  include <Eigen/MetisSupport>
#endif

namespace COMISO {

template<typename T, typename Scalar=double>
class MatrixDecompositionT : public MatrixDecomposition<Scalar>
{
public:
    using Vector = typename MatrixDecomposition<Scalar>::Vector;
    using SMatrix = Eigen::SparseMatrix<Scalar>;
    MatrixDecompositionT() = default;
    MatrixDecompositionT(SMatrix const&m)
        : ldlt_(m)
    {}
    ~MatrixDecompositionT() override = default;

    virtual void analyzePattern(const SMatrix& a) override {
        return ldlt_.analyzePattern(a);
    }
    virtual void compute(const SMatrix& a) override {
        ldlt_.compute(a);
    }
    virtual void factorize(const SMatrix& a) override {
        return ldlt_.factorize(a);
    }
    virtual Eigen::ComputationInfo info() const override { 
        return ldlt_.info();
    }
    virtual Vector solve(Vector const&x) const override {
        return ldlt_.solve(x);
    }
private:
    T ldlt_;
};

// cholesky decomposition of projection

template<typename Scalar>
std::unique_ptr<MatrixDecomposition<Scalar>>
make_decomposition(MatrixDecompositionAlgorithm algo)
{
    using MDA = MatrixDecompositionAlgorithm;
    using M = Eigen::SparseMatrix<Scalar>;
    using Index = typename M::StorageIndex;
    switch(algo) {
        case MDA::Eigen_SimplicialLDLD_MetisOrdering:
#if COMISO_METIS_AVAILABLE
            return std::make_unique<MatrixDecompositionT<
                Eigen::SimplicialLDLT<M, Eigen::Lower, Eigen::MetisOrdering<Index>>,
                Scalar>>();
#else
            throw std::runtime_error("make_decomposition(): CoMISo was compiled without METIS support");
#endif
        case MDA::Eigen_SimplicialLDLT_NaturalOrdering:
            return std::make_unique<MatrixDecompositionT<
                Eigen::SimplicialLDLT<M, Eigen::Lower, Eigen::NaturalOrdering<Index>>,
                Scalar>>();
        case MDA::Eigen_SimplicialLDLT_AMDOrdering:
            return std::make_unique<MatrixDecompositionT<
                Eigen::SimplicialLDLT<M, Eigen::Lower, Eigen::AMDOrdering<Index>>,
                Scalar>>();
        case MDA::Eigen_SimplicialCholesky:
            return std::make_unique<MatrixDecompositionT<
                Eigen::SimplicialCholesky<M, Eigen::Lower, Eigen::AMDOrdering<Index>>,
                Scalar>>();
        case MDA::Cholmod_Supernodal:
#if COMISO_SUITESPARSE_CHOLMOD_AVAILABLE
            return std::make_unique<MatrixDecompositionT<
                Eigen::CholmodSupernodalLLT<M, Eigen::Lower>,
                Scalar>>();
#else
            throw std::runtime_error("make_decomposition(): CoMISo was compiled without CHOLMOD support");
#endif
        case MDA::Cholmod_SimplicialLLT:
#if COMISO_SUITESPARSE_CHOLMOD_AVAILABLE
            return std::make_unique<MatrixDecompositionT<
                Eigen::CholmodSimplicialLLT<M, Eigen::Lower>,
                Scalar>>();
#else
            throw std::runtime_error("make_decomposition(): CoMISo was compiled without CHOLMOD support");
#endif
        case MDA::UmfPack_LU:
#if COMISO_SUITESPARSE_UMFPACK_AVAILABLE
            return std::make_unique<MatrixDecompositionT<
                Eigen::UmfPackLU<M>,
                Scalar>>();
#else
            throw std::runtime_error("make_decomposition(): CoMISo was compiled without UMFPACK support");
#endif
        default:
            return {};
    }
}

} // namespace COMISO
