#ifndef RMCL_ROS_MCL_OPTIX_CUDA_RMAGINE_CONVERSIONS_CUH
#define RMCL_ROS_MCL_OPTIX_CUDA_RMAGINE_CONVERSIONS_CUH

#include <rmagine/math/types.h>

inline __host__ __device__ float3 make_float3(rmagine::Vector3f v)
{
    return make_float3(v.x, v.y, v.z);
}

#endif // RMCL_ROS_MCL_OPTIX_CUDA_RMAGINE_CONVERSIONS_CUH