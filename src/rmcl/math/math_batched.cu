#include "rmcl/math/math_batched.cuh"

namespace rm = rmagine;

namespace rmcl
{

template<unsigned int blockSize, typename T>
__device__ void warpReduce(volatile T* sdata, unsigned int tid)
{
    if(blockSize >= 64) sdata[tid] += sdata[tid + 32];
    if(blockSize >= 32) sdata[tid] += sdata[tid + 16];
    if(blockSize >= 16) sdata[tid] += sdata[tid + 8];
    if(blockSize >=  8) sdata[tid] += sdata[tid + 4];
    if(blockSize >=  4) sdata[tid] += sdata[tid + 2];
    if(blockSize >=  2) sdata[tid] += sdata[tid + 1];
}

template<unsigned int blockSize>
__global__ void meanBatched_kernel(
    const rm::Vector* data,
    const unsigned int* mask,
    const unsigned int* Ncorr,
    unsigned int chunkSize,
    rm::Vector* res)
{
    __shared__ rm::Vector sdata[blockSize];

    const unsigned int tid = threadIdx.x;
    const unsigned int globId = chunkSize * blockIdx.x + threadIdx.x;
    const unsigned int rows = (chunkSize + blockSize - 1) / blockSize;

    sdata[tid] = {0.0, 0.0, 0.0};

    for(unsigned int i=0; i<rows; i++)
    {
        if(tid + blockSize * i < chunkSize)
        {
            if(mask[globId + blockSize * i] > 0)
            {
                sdata[tid] += data[globId + blockSize * i];
            }
        }
    }
    __syncthreads();
    
    for(unsigned int s = blockSize / 2; s > 32; s >>= 1)
    {
        if(tid < s)
        {
            sdata[tid] += sdata[tid + s];
        }
        __syncthreads();
    }

    if(tid < blockSize / 2 && tid < 32)
    {
        warpReduce<blockSize>(sdata, tid);
    }

    if(tid == 0)
    {
        const unsigned int Ncorr_ = Ncorr[blockIdx.x];
        if(Ncorr_ > 0)
        {
            res[blockIdx.x] = sdata[0] / Ncorr_;
        } else {
            res[blockIdx.x] = {0.0, 0.0, 0.0};
        }
    }
}

template<unsigned int blockSize>
__global__ void sumFancyBatched_kernel(
    const rm::Vector* data1, // to
    const rm::Vector* center1,
    const rm::Vector* data2, // from
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
                sdata[tid] += b.multT(a);
            }
        }
    }
    __syncthreads();
    
    for(unsigned int s = blockSize / 2; s > 32; s >>= 1)
    {
        if(tid < s)
        {
            sdata[tid] += sdata[tid + s];
        }
        __syncthreads();
    }

    if(tid < blockSize / 2 && tid < 32)
    {
        warpReduce<blockSize>(sdata, tid);
    }

    if(tid == 0)
    {
        Cs[blockIdx.x] = sdata[0];
    }
}


template<unsigned int blockSize>
__global__ void covFancyBatched_kernel(
    const rm::Vector* data1, // to
    const rm::Vector* center1,
    const rm::Vector* data2, // from
    const rm::Vector* center2,
    const unsigned int* mask,
    const unsigned int* Ncorr,
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
                sdata[tid] += b.multT(a);
            }
        }
    }
    __syncthreads();
    
    for(unsigned int s = blockSize / 2; s > 32; s >>= 1)
    {
        if(tid < s)
        {
            sdata[tid] += sdata[tid + s];
        }
        __syncthreads();
    }

    if(tid < blockSize / 2 && tid < 32)
    {
        warpReduce<blockSize>(sdata, tid);
    }

    if(tid == 0)
    {
        const unsigned int Ncorr_ = Ncorr[blockIdx.x];
        if(Ncorr_ > 0)
        {
            Cs[blockIdx.x] = sdata[0] / Ncorr_;
        } else {
            Cs[blockIdx.x].setZeros();
        }
    }
}


template<unsigned int blockSize>
__global__ void covFancyBatched_kernel(
    const rm::Vector* data1, // to
    const rm::Vector* data2, // from
    const unsigned int* mask,
    const unsigned int* Ncorr,
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
                // dataset: d, model: m
                const rm::Vector d = data1[globId + blockSize * i];
                const rm::Vector m = data2[globId + blockSize * i];
                sdata[tid] += m.multT(d);
            }
        }
    }
    __syncthreads();
    
    for(unsigned int s = blockSize / 2; s > 32; s >>= 1)
    {
        if(tid < s)
        {
            sdata[tid] += sdata[tid + s];
        }
        __syncthreads();
    }

    if(tid < blockSize / 2 && tid < 32)
    {
        warpReduce<blockSize>(sdata, tid);
    }

    if(tid == 0)
    {
        const unsigned int Ncorr_ = Ncorr[blockIdx.x];
        if(Ncorr_ > 0)
        {
            Cs[blockIdx.x] = sdata[0] / Ncorr_;
        } else {
            Cs[blockIdx.x].setZeros();
        }
    }
}

void meanBatched(
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& res)
{
    unsigned int Nchunks = Ncorr.size();
    unsigned int batchSize = data.size() / Nchunks;
    constexpr unsigned int blockSize = 128; // TODO: get best value for this one

    meanBatched_kernel<blockSize> <<<Nchunks, blockSize>>>(
        data.raw(), mask.raw(), 
        Ncorr.raw(), batchSize, 
        res.raw());
}

rm::Memory<rm::Vector, rm::VRAM_CUDA> meanBatched(
    const rm::MemoryView<rmagine::Vector, rm::VRAM_CUDA>& data,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr)
{
    rm::Memory<rm::Vector, rm::VRAM_CUDA> res(Ncorr.size());
    meanBatched(data, mask, Ncorr, res);
    return res;
}

void sumFancyBatched(
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data1,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& center1,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data2,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& center2,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask,
    rm::MemoryView<rm::Matrix3x3, rm::VRAM_CUDA>& Cs)
{
    unsigned int Nchunks = center1.size();
    unsigned int batchSize = data1.size() / Nchunks;
    constexpr unsigned int blockSize = 64; // TODO: get best value for this one

    sumFancyBatched_kernel<blockSize> <<<Nchunks, blockSize>>>(
        data1.raw(), center1.raw(), 
        data2.raw(), center2.raw(), 
        mask.raw(), batchSize, 
        Cs.raw());
}

rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> sumFancyBatched(
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data1, // from
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& center1,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data2, // to
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& center2,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask)
{
    unsigned int Nchunks = center1.size();
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Nchunks);
    sumFancyBatched(data1, center1, data2, center2, mask, Cs);
    return Cs;
}

void covFancyBatched(
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data1,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& center1,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data2,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& center2,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr,
    rm::MemoryView<rm::Matrix3x3, rm::VRAM_CUDA>& Cs)
{
    unsigned int Nchunks = center1.size();
    unsigned int batchSize = data1.size() / Nchunks;
    constexpr unsigned int blockSize = 64; // TODO: get best value for this one

    covFancyBatched_kernel<blockSize> <<<Nchunks, blockSize>>>(
        data1.raw(), center1.raw(), 
        data2.raw(), center2.raw(), 
        mask.raw(), Ncorr.raw(), 
        batchSize, Cs.raw());
}

rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> covFancyBatched(
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data1, // from
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& center1,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data2, // to
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& center2,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr)
{
    unsigned int Nchunks = center1.size();
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Nchunks);
    covFancyBatched(data1, center1, data2, center2, mask, Ncorr, Cs);
    return Cs;
}


void covFancyBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data1, // from, dataset
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data2, // to, model
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs)
{
    unsigned int Nchunks = Ncorr.size();
    unsigned int batchSize = data1.size() / Nchunks;
    constexpr unsigned int blockSize = 64; // TODO: get best value for this one

    covFancyBatched_kernel<blockSize> <<<Nchunks, blockSize>>>(
        data1.raw(), data2.raw(), 
        mask.raw(), Ncorr.raw(), 
        batchSize, Cs.raw());
}

rmagine::Memory<rmagine::Matrix3x3, rmagine::VRAM_CUDA> covFancyBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data1, // from, dataset
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data2, // to, model
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr)
{
    unsigned int Nchunks = Ncorr.size();
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Nchunks);
    covFancyBatched(data1, data2, mask, Ncorr, Cs);
    return Cs;
}
    

} // namespace rmcl