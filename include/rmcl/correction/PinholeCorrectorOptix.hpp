#ifndef RMCL_Pinhole_CORRECTOR_OPTIX_HPP
#define RMCL_Pinhole_CORRECTOR_OPTIX_HPP

#include <rmagine/map/OptixMap.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/math/types.h>
#include <rmagine/math/SVDCuda.hpp>
#include <rmagine/simulation/PinholeSimulatorOptix.hpp>

#include "CorrectionResults.hpp"
#include "CorrectionParams.hpp"

#include <memory>

namespace rmcl 
{

class PinholeCorrectorOptix 
: public rmagine::PinholeSimulatorOptix
{
public:
    using Base = rmagine::PinholeSimulatorOptix;

    PinholeCorrectorOptix(rmagine::OptixMapPtr map);

    void setParams(
        const CorrectionParams& params);

    void setInputData(
        const rmagine::MemoryView<float, rmagine::VRAM_CUDA>& ranges
    );

    void setOptical(bool optical = true);

    CorrectionResults<rmagine::VRAM_CUDA> correct(
        const rmagine::Memory<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms
    ) const;
    
protected:
    rmagine::Memory<float, rmagine::VRAM_CUDA> m_ranges;
    rmagine::Memory<CorrectionParams, rmagine::VRAM_CUDA> m_params;

    rmagine::SVDCudaPtr m_svd;

    std::vector<rmagine::OptixProgramPtr> programs;

    bool m_optical = false;

private:
    void computeMeansCovsRW(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm,
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& m1, // from, dataset
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& m2, // to, model
        rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
        ) const;

    void computeMeansCovsSW(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm,
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& m1,
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& m2,
        rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
        ) const;
};

using PinholeCorrectorOptixPtr = std::shared_ptr<PinholeCorrectorOptix>;

} // namespace rmcl

#endif // RMCL_Pinhole_CORRECTOR_OPTIX_HPP