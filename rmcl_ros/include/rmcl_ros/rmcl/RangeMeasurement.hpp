#ifndef RMCL_MCL_RANGE_MEASUREMENT_HPP
#define RMCL_MCL_RANGE_MEASUREMENT_HPP

#include <rmagine/math/types.h>

namespace rmcl
{

// TODO: move this to rmagine
struct RangeMeasurement
{
  rmagine::Vector       orig;
  rmagine::Vector       dir;
  float                 range;
  rmagine::Matrix3x3    cov;

  inline rmagine::Vector3 mean() const
  {
    return orig + dir * range;
  }
};

/**
 * @brief this transforms a RangeMeasurement from one space to another
 * Remember: Measurements are only comparable in the same coordinate system!
 * Therefore, this function is very important
 */
RMAGINE_INLINE_FUNCTION
RangeMeasurement transform(
  const rmagine::Transform& T,
  const RangeMeasurement& meas)
{
  RangeMeasurement ret;
  const rmagine::Matrix3x3 R = T.R;
  ret.dir = T.R * meas.dir;
  ret.orig = T * meas.orig;
  ret.range = meas.range;
  // seems the range is not effected by a rigid transformation
  // however, it will be effected by a transformation that include a scaling part
  ret.cov = R * meas.cov * R.T(); // TODO: check this
  return ret;
}

RMAGINE_INLINE_FUNCTION
RangeMeasurement operator*(
  const rmagine::Transform& T,
  const RangeMeasurement& meas)
{
  return transform(T, meas);
}

} // namespace rmcl

#endif // RMCL_MCL_RANGE_MEASUREMENT_HPP
