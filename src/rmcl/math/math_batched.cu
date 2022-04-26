#include "rmcl/math/math_batched.cuh"

namespace rm = rmagine;

namespace rmcl
{

template<unsigned int blockSize>
__global__ void sumFancyBatched_kernel(
    const rm::Vector* data1,
    const rm::Vector* center1,
    const rm::Vector* data2, 
    const rm::Vector* center2,
    const unsigned int* mask,
    unsigned int chunkSize,
    rm::Matrix3x3* Cs)
{
    __shared__ rm::Matrix3x3 sdata[blockSize];

    const unsigned int tid = threadIdx.x;
    const unsigned int globId = chunkSize * blockIdx.x + threadIdx.x;
    const unsigned int rows = (chunkSize + blockSize - 1) / blockSize;

    sdata[tid].setZeros();

    for(unsigned int i=0; i<rows; i++)
    {
        if(tid + blockSize * i < chunkSize)
        {
            if(mask[globId + blockSize * i] > 0)
            {
                const rm::Vector a = data1[globId + blockSize * i] - center1[blockIdx.x];
                const rm::Vector b = data2[globId + blockSize * i] - center2[blockIdx.x];

                sdata[tid](1,0) += a.x * b.y;
                sdata[tid](2,0) += a.x * b.z;
                sdata[tid](0,1) += a.y * b.x;
                sdata[tid](1,1) += a.y * b.y;
                sdata[tid](2,1) += a.y * b.z;
                sdata[tid](0,2) += a.z * b.x;
                sdata[tid](1,2) += a.z * b.y;
                sdata[tid](2,2) += a.z * b.z;
            }
        }
    }
    __syncthreads();
    
    for(unsigned int s=blockSize / 2; s > 0; s >>= 1)
    {
        if(tid < s)
        {
            sdata[tid] += sdata[tid + s];
        }
        __syncthreads();
    }

    if(tid == 0)
    {
        Cs[blockIdx.x] = sdata[0];
    }
}

void sumFancyBatched(
    const rm::Memory<rm::Vector, rm::VRAM_CUDA>& data1,
    const rm::Memory<rm::Vector, rm::VRAM_CUDA>& center1,
    const rm::Memory<rm::Vector, rm::VRAM_CUDA>& data2,
    const rm::Memory<rm::Vector, rm::VRAM_CUDA>& center2,
    const rm::Memory<unsigned int, rm::VRAM_CUDA>& mask,
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA>& Cs)
{
    unsigned int Nchunks = center1.size();
    unsigned int batchSize = data1.size() / Nchunks;
    constexpr unsigned int blockSize = 16;

    sumFancyBatched_kernel<blockSize> <<<Nchunks, blockSize>>>(
        data1.raw(), center1.raw(), 
        data2.raw(), center2.raw(), 
        mask.raw(), batchSize, 
        Cs.raw());
}

rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> sumFancyBatched(
    const rm::Memory<rm::Vector, rm::VRAM_CUDA>& data1,
    const rm::Memory<rm::Vector, rm::VRAM_CUDA>& center1,
    const rm::Memory<rm::Vector, rm::VRAM_CUDA>& data2,
    const rm::Memory<rm::Vector, rm::VRAM_CUDA>& center2,
    const rm::Memory<unsigned int, rm::VRAM_CUDA>& mask)
{
    unsigned int Nchunks = center1.size();
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Nchunks);
    sumFancyBatched(data1, center1, data2, center2, mask, Cs);
    return Cs;
}
    

} // namespace rmcl