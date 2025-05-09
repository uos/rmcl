#ifndef RMCL_CORRECTION_CORRESPONDENCES_CPU_HPP
#define RMCL_CORRECTION_CORRESPONDENCES_CPU_HPP

#include <rmagine/types/Memory.hpp>
#include <rmcl_ros/correction/Correspondences.hpp>

namespace rmcl
{

class CorrespondencesCPU
: public Correspondences_<rmagine::RAM>
{
public:
  rmagine::CrossStatistics computeCrossStatistics(
    const rmagine::Transform& T_snew_sold) const;
};

} // namespace rmcl

#endif // RMCL_CORRECTION_CORRESPONDENCES_CPU_HPP