#ifndef RMCL_MATH_MATH_CUH
#define RMCL_MATH_MATH_CUH

#include <rmagine/math/types.h>
#include <rmagine/types/MemoryCuda.hpp>
#include <vector>
#include <rmcl/correction/CorrectionResults.hpp>
#include <rmagine/math/SVDCuda.hpp>
#include <memory>

namespace rmcl {

class CorrectionCuda {
public:
    CorrectionCuda();
    CorrectionCuda(rmagine::SVDCudaPtr svd);

    void correction_from_covs(
        const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
        const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr,
        rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tdelta
    ) const;

    void correction_from_covs(
        const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
        const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr,
        rmagine::MemoryView<rmagine::Quaternion, rmagine::VRAM_CUDA>& Rdelta,
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& tdelta
    ) const;

    void correction_from_covs(
        const CorrectionPreResults<rmagine::VRAM_CUDA>& pre_res,
        rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tdelta
    ) const;

    rmagine::Memory<rmagine::Transform, rmagine::VRAM_CUDA> correction_from_covs(
        const CorrectionPreResults<rmagine::VRAM_CUDA>& pre_res
    ) const;

private:
    rmagine::SVDCudaPtr m_svd;
};

using CorrectionCudaPtr = std::shared_ptr<CorrectionCuda>;

void compute_transform(
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Us,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Vs,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& dT);

// weighted average by
// - number of correspondences
// - fixed weights
// TODO: more than two

void weighted_average(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds1,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs1,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds2,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs2,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr2,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
);

void weighted_average(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds1,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs1,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr1,
    float w1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds2,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs2,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr2,
    float w2,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
);

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA> >& Ncorrs,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
);

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA> >& Ncorrs,
    const std::vector<float>& weights,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
);

void weighted_average(
    const std::vector<CorrectionPreResults<rmagine::VRAM_CUDA> >& pre_results,
    CorrectionPreResults<rmagine::VRAM_CUDA>& pre_results_combined
);

CorrectionPreResults<rmagine::VRAM_CUDA> weighted_average(
    const std::vector<CorrectionPreResults<rmagine::VRAM_CUDA> >& pre_results
);

void weighted_average(
    const std::vector<CorrectionPreResults<rmagine::VRAM_CUDA> >& pre_results,
    const std::vector<float>& weights,
    CorrectionPreResults<rmagine::VRAM_CUDA>& pre_results_combined
);

CorrectionPreResults<rmagine::VRAM_CUDA> weighted_average(
    const std::vector<CorrectionPreResults<rmagine::VRAM_CUDA> >& pre_results,
    const std::vector<float>& weights
);

} // namespace rmcl

#endif // RMCL_MATH_MATH_CUH