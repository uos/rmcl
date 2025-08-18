#ifndef RMCL_ROS_MCL_OPTIX_CUDA_MATH_HELPER_CUH
#define RMCL_ROS_MCL_OPTIX_CUDA_MATH_HELPER_CUH

#include <rmagine/math/types.h>

inline __host__ __device__ float3 operator+(float3 a, float3 b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline __host__ __device__ float3 operator-(float3 a, float3 b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline __host__ __device__ float3 operator*(float3 a, float s)
{
    return make_float3(a.x * s, a.y * s, a.z * s);
}

inline __host__ __device__ float3 operator*(float s, float3 a)
{
    return a * s;
}

inline __host__ __device__ float dot(float3 a, float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

#endif // RMCL_ROS_MCL_OPTIX_CUDA_MATH_HELPER_CUH