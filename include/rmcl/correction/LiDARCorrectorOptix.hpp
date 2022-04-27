#ifndef RMCL_LIDAR_CORRECTOR_OPTIX_HPP
#define RMCL_LIDAR_CORRECTOR_OPTIX_HPP

#include <rmagine/map/OptixMap.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/math/types.h>
#include <rmagine/math/SVDCuda.hpp>
#include <rmagine/simulation/SphereSimulatorOptix.hpp>

#include "CorrectionResults.hpp"
#include "CorrectionParams.hpp"

#include <memory>

namespace rmcl 
{

class LiDARCorrectorOptix 
: public rmagine::SphereSimulatorOptix
{
public:
    using Base = rmagine::SphereSimulatorOptix;

    LiDARCorrectorOptix(rmagine::OptixMapPtr map);

    void setParams(
        const CorrectionParams& params);

    void setInputData(
        const rmagine::Memory<float, rmagine::VRAM_CUDA>& ranges
    );

    CorrectionResults<rmagine::VRAM_CUDA> correct(
        const rmagine::Memory<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms
    ) const;
    
protected:
    rmagine::Memory<float, rmagine::VRAM_CUDA> m_ranges;
    rmagine::Memory<CorrectionParams, rmagine::VRAM_CUDA> m_params;

    rmagine::SVDCudaPtr m_svd;

    std::vector<rmagine::OptixProgramPtr> programs;

private:
    void computeMeansCovsRW(
        const rmagine::Memory<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm,
        rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& m1,
        rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& m2,
        rmagine::Memory<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        rmagine::Memory<unsigned int, rmagine::VRAM_CUDA>& Ncorr
        ) const;

    void computeMeansCovsSW(
        const rmagine::Memory<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm,
        rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& m1,
        rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& m2,
        rmagine::Memory<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        rmagine::Memory<unsigned int, rmagine::VRAM_CUDA>& Ncorr
        ) const;
};

using LiDARCorrectorOptixPtr = std::shared_ptr<LiDARCorrectorOptix>;

} // namespace rmcl

#endif // RMCL_LIDAR_CORRECTOR_OPTIX_HPP