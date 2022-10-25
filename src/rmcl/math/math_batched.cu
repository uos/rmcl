#include "rmcl/math/math_batched.cuh"

namespace rm = rmagine;

namespace rmcl
{

template<unsigned int blockSize, typename T>
__device__ void warp_reduce(volatile T* sdata, unsigned int tid)
{
    if(blockSize >= 64) sdata[tid] += sdata[tid + 32];
    if(blockSize >= 32) sdata[tid] += sdata[tid + 16];
    if(blockSize >= 16) sdata[tid] += sdata[tid + 8];
    if(blockSize >=  8) sdata[tid] += sdata[tid + 4];
    if(blockSize >=  4) sdata[tid] += sdata[tid + 2];
    if(blockSize >=  2) sdata[tid] += sdata[tid + 1];
}

template<unsigned int blockSize>
__global__ void mean_batched_kernel(
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
        warp_reduce<blockSize>(sdata, tid);
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
__global__ void sum_batched_kernel(
    const rm::Vector* data1, // from, dataset
    const rm::Vector* center1,
    const rm::Vector* data2, // to, model
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
                const rm::Vector d = data1[globId + blockSize * i] - center1[blockIdx.x];
                const rm::Vector m = data2[globId + blockSize * i] - center2[blockIdx.x];
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
        warp_reduce<blockSize>(sdata, tid);
    }

    if(tid == 0)
    {
        Cs[blockIdx.x] = sdata[0];
    }
}


template<unsigned int blockSize>
__global__ void cov_batched_kernel(
    const rm::Vector* dataset_points, // from, dataset
    const rm::Vector* dataset_center,
    const rm::Vector* model_points, // to, model
    const rm::Vector* model_center,
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
                const rm::Vector d = dataset_points[globId + blockSize * i] - dataset_center[blockIdx.x];
                const rm::Vector m = model_points[globId + blockSize * i] - model_center[blockIdx.x];
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
        warp_reduce<blockSize>(sdata, tid);
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
__global__ void cov_batched_kernel(
    const rm::Vector* dataset_points, // from, dataset
    const rm::Vector* model_points, // to, model
    const unsigned int* mask, // valid correspondences
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
                const rm::Vector d = dataset_points[globId + blockSize * i];
                const rm::Vector m = model_points[globId + blockSize * i];
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
        warp_reduce<blockSize>(sdata, tid);
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

// template<unsigned int blockSize>
// __global__ void cov_online_batched_kernel(
//     const rm::Vector* dataset_points, // from, dataset
//     const rm::Vector* model_points, // to, model
//     const unsigned int* mask,
//     const unsigned int* Ncorr,
//     unsigned int chunkSize,
//     rm::Vector* dataset_center,
//     rm::Vector* model_center,
//     rm::Matrix3x3* Cs)
// {
//     __shared__ rm::Matrix3x3 sdata[blockSize];

//     const unsigned int tid = threadIdx.x;
//     const unsigned int globId = chunkSize * blockIdx.x + threadIdx.x;
//     const unsigned int rows = (chunkSize + blockSize - 1) / blockSize;

//     sdata[tid].setZeros();

//     for(unsigned int i=0; i<rows; i++)
//     {
//         if(tid + blockSize * i < chunkSize)
//         {
//             if(mask[globId + blockSize * i] > 0)
//             {
//                 const rm::Vector d = dataset_points[globId + blockSize * i] - dataset_center[blockIdx.x];
//                 const rm::Vector m = model_points[globId + blockSize * i] - model_center[blockIdx.x];
//                 sdata[tid] += m.multT(d);
//             }
//         }
//     }
//     __syncthreads();
    
//     for(unsigned int s = blockSize / 2; s > 32; s >>= 1)
//     {
//         if(tid < s)
//         {
//             sdata[tid] += sdata[tid + s];
//         }
//         __syncthreads();
//     }

//     if(tid < blockSize / 2 && tid < 32)
//     {
//         warp_reduce<blockSize>(sdata, tid);
//     }

//     if(tid == 0)
//     {
//         const unsigned int Ncorr_ = Ncorr[blockIdx.x];
//         if(Ncorr_ > 0)
//         {
//             Cs[blockIdx.x] = sdata[0] / Ncorr_;
//         } else {
//             Cs[blockIdx.x].setZeros();
//         }
//     }
// }

void mean_batched(
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& res)
{
    unsigned int Nchunks = Ncorr.size();
    unsigned int batchSize = data.size() / Nchunks;
    constexpr unsigned int blockSize = 128; // TODO: get best value for this one

    mean_batched_kernel<blockSize> <<<Nchunks, blockSize>>>(
        data.raw(), mask.raw(), 
        Ncorr.raw(), batchSize, 
        res.raw());
}

rm::Memory<rm::Vector, rm::VRAM_CUDA> mean_batched(
    const rm::MemoryView<rmagine::Vector, rm::VRAM_CUDA>& data,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr)
{
    rm::Memory<rm::Vector, rm::VRAM_CUDA> res(Ncorr.size());
    mean_batched(data, mask, Ncorr, res);
    return res;
}

void sum_batched(
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

    sum_batched_kernel<blockSize> <<<Nchunks, blockSize>>>(
        data1.raw(), center1.raw(), 
        data2.raw(), center2.raw(), 
        mask.raw(), batchSize, 
        Cs.raw());
}

rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> sum_batched(
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data1, // from
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& center1,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& data2, // to
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& center2,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask)
{
    unsigned int Nchunks = center1.size();
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Nchunks);
    sum_batched(data1, center1, data2, center2, mask, Cs);
    return Cs;
}

void cov_batched(
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& dataset_points,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& dataset_center,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& model_points,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& model_center,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr,
    rm::MemoryView<rm::Matrix3x3, rm::VRAM_CUDA>& Cs)
{
    unsigned int Nchunks = dataset_center.size();
    unsigned int batchSize = dataset_points.size() / Nchunks;
    constexpr unsigned int blockSize = 64; // TODO: get best value for this one

    cov_batched_kernel<blockSize> <<<Nchunks, blockSize>>>(
        dataset_points.raw(), dataset_center.raw(), 
        model_points.raw(), model_center.raw(), 
        mask.raw(), Ncorr.raw(), 
        batchSize, Cs.raw());
}

rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> cov_batched(
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& dataset_points, // from
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& dataset_center,
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& model_points, // to
    const rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& model_center,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& mask,
    const rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr)
{
    unsigned int Nchunks = dataset_center.size();
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Nchunks);
    cov_batched(dataset_points, dataset_center, model_points, model_center, mask, Ncorr, Cs);
    return Cs;
}


void cov_batched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& dataset_points, // from, dataset
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& model_points, // to, model
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs)
{
    unsigned int Nchunks = Ncorr.size();
    unsigned int batchSize = dataset_points.size() / Nchunks;
    constexpr unsigned int blockSize = 64; // TODO: get best value for this one

    cov_batched_kernel<blockSize> <<<Nchunks, blockSize>>>(
        dataset_points.raw(), model_points.raw(), 
        mask.raw(), Ncorr.raw(), 
        batchSize, Cs.raw());
}

rmagine::Memory<rmagine::Matrix3x3, rmagine::VRAM_CUDA> cov_batched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& dataset_points, // from, dataset
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& model_points, // to, model
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr)
{
    unsigned int Nchunks = Ncorr.size();
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Nchunks);
    cov_batched(dataset_points, model_points, mask, Ncorr, Cs);
    return Cs;
}

} // namespace rmcl