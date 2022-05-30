#include "rmcl/math/math.cuh"

#include <rmagine/math/math.cuh>

using namespace rmagine;
namespace rm = rmagine;


namespace rmcl 
{

void CorrectionCuda::correction_from_covs(
    const MemoryView<Vector, VRAM_CUDA>& ms,
    const MemoryView<Vector, VRAM_CUDA>& ds,
    const MemoryView<Matrix3x3, VRAM_CUDA>& Cs,
    const MemoryView<unsigned int, VRAM_CUDA>& Ncorr,
    MemoryView<Transform, VRAM_CUDA>& Tdelta) const
{
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Us(Cs.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Vs(Cs.size());

    m_svd->calcUV(Cs, Us, Vs);
    rm::transposeInplace(Vs);

    auto Rs = rm::multNxN(Us, Vs);
    auto ts = rm::subNxN(ds, rm::multNxN(Rs, ms));

    rm::pack(Rs, ts, Tdelta);
}

void CorrectionCuda::correction_from_covs(
    const MemoryView<Vector, VRAM_CUDA>& ms,
    const MemoryView<Vector, VRAM_CUDA>& ds,
    const MemoryView<Matrix3x3, VRAM_CUDA>& Cs,
    const MemoryView<unsigned int, VRAM_CUDA>& Ncorr,
    MemoryView<Quaternion, VRAM_CUDA>& Rdelta,
    MemoryView<Vector, VRAM_CUDA>& tdelta) const
{
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Us(Cs.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Vs(Cs.size());

    m_svd->calcUV(Cs, Us, Vs);
    rm::transposeInplace(Vs);

    rm::multNxN(Us, Vs, Rdelta);
    rm::subNxN(ds, rm::multNxN(Rdelta, ms), tdelta);
}

void CorrectionCuda::correction_from_covs(
    const CorrectionPreResults<VRAM_CUDA>& pre_res,
    MemoryView<Transform, VRAM_CUDA>& Tdelta) const
{
    correction_from_covs(pre_res.ms, pre_res.ds, pre_res.Cs, pre_res.Ncorr, Tdelta);
}

Memory<Transform, VRAM_CUDA> CorrectionCuda::correction_from_covs(
    const CorrectionPreResults<VRAM_CUDA>& pre_res) const
{
    Memory<Transform, VRAM_CUDA> Tdelta(pre_res.ms.size());
    correction_from_covs(pre_res, Tdelta);
    return Tdelta;
}

// weighted average by
// - number of correspondences
// - fixed weights
// TODO: more than two

void weighted_average(
    const MemoryView<Vector, VRAM_CUDA>& ms1,
    const MemoryView<Vector, VRAM_CUDA>& ds1,
    const MemoryView<Matrix3x3, VRAM_CUDA>& Cs1,
    const MemoryView<unsigned int, VRAM_CUDA>& Ncorr1,
    const MemoryView<Vector, VRAM_CUDA>& ms2,
    const MemoryView<Vector, VRAM_CUDA>& ds2,
    const MemoryView<Matrix3x3, VRAM_CUDA>& Cs2,
    const MemoryView<unsigned int, VRAM_CUDA>& Ncorr2,
    MemoryView<Vector, VRAM_CUDA>& ms,
    MemoryView<Vector, VRAM_CUDA>& ds,
    MemoryView<Matrix3x3, VRAM_CUDA>& Cs,
    MemoryView<unsigned int, VRAM_CUDA>& Ncorr)
{

}

void weighted_average(
    const MemoryView<Vector, VRAM_CUDA>& ms1,
    const MemoryView<Vector, VRAM_CUDA>& ds1,
    const MemoryView<Matrix3x3, VRAM_CUDA>& Cs1,
    const MemoryView<unsigned int, VRAM_CUDA>& Ncorr1,
    float w1,
    const MemoryView<Vector, VRAM_CUDA>& ms2,
    const MemoryView<Vector, VRAM_CUDA>& ds2,
    const MemoryView<Matrix3x3, VRAM_CUDA>& Cs2,
    const MemoryView<unsigned int, VRAM_CUDA>& Ncorr2,
    float w2,
    MemoryView<Vector, VRAM_CUDA>& ms,
    MemoryView<Vector, VRAM_CUDA>& ds,
    MemoryView<Matrix3x3, VRAM_CUDA>& Cs,
    MemoryView<unsigned int, VRAM_CUDA>& Ncorr)
{

}

void weighted_average(
    const std::vector<MemoryView<Vector, VRAM_CUDA> >& model_means,
    const std::vector<MemoryView<Vector, VRAM_CUDA> >& dataset_means,
    const std::vector<MemoryView<Matrix3x3, VRAM_CUDA> >& covs,
    const std::vector<MemoryView<unsigned int, VRAM_CUDA> >& Ncorrs,
    MemoryView<Vector, VRAM_CUDA>& ms,
    MemoryView<Vector, VRAM_CUDA>& ds,
    MemoryView<Matrix3x3, VRAM_CUDA>& Cs,
    MemoryView<unsigned int, VRAM_CUDA>& Ncorr)
{

}

void weighted_average(
    const std::vector<MemoryView<Vector, VRAM_CUDA> >& model_means,
    const std::vector<MemoryView<Vector, VRAM_CUDA> >& dataset_means,
    const std::vector<MemoryView<Matrix3x3, VRAM_CUDA> >& covs,
    const std::vector<MemoryView<unsigned int, VRAM_CUDA> >& Ncorrs,
    const std::vector<float>& weights,
    MemoryView<Vector, VRAM_CUDA>& ms,
    MemoryView<Vector, VRAM_CUDA>& ds,
    MemoryView<Matrix3x3, VRAM_CUDA>& Cs,
    MemoryView<unsigned int, VRAM_CUDA>& Ncorr)
{

}

void weighted_average(
    const std::vector<CorrectionPreResults<VRAM_CUDA> >& pre_results,
    CorrectionPreResults<VRAM_CUDA>& pre_results_combined)
{

}

CorrectionPreResults<VRAM_CUDA> weighted_average(
    const std::vector<CorrectionPreResults<VRAM_CUDA> >& pre_results)
{

}

void weighted_average(
    const std::vector<CorrectionPreResults<VRAM_CUDA> >& pre_results,
    const std::vector<float>& weights,
    CorrectionPreResults<VRAM_CUDA>& pre_results_combined)
{

}

CorrectionPreResults<VRAM_CUDA> weighted_average(
    const std::vector<CorrectionPreResults<VRAM_CUDA> >& pre_results,
    const std::vector<float>& weights)
{

}

} // namespace rmcl