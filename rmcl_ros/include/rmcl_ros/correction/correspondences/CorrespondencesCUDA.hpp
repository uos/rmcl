#ifndef RMCL_CORRECTION_CORRESPONDENCES_CUDA_HPP
#define RMCL_CORRECTION_CORRESPONDENCES_CUDA_HPP

#include <rmagine/types/MemoryCuda.hpp>
#include <rmcl_ros/correction/Correspondences.hpp>

namespace rmcl
{

class CorrespondencesCUDA
: public Correspondences_<rmagine::VRAM_CUDA>
{
public:
  rmagine::CrossStatistics computeCrossStatistics(
    const rmagine::Transform& T_snew_sold,
    double convergence_progress = 0.0) const;
};

} // namespace rmcl

#endif // RMCL_CORRECTION_CORRESPONDENCES_CUDA_HPP