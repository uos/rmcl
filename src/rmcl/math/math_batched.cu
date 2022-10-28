#include "rmcl/math/math_batched.cuh"

#include <rmagine/math/math_batched.cuh>

namespace rm = rmagine;

namespace rmcl
{

template<unsigned int blockSize, typename T>
__device__ void warp_sum(volatile T* sdata, unsigned int tid)
{
    if(blockSize >= 64) sdata[tid] += sdata[tid + 32];
    if(blockSize >= 32) sdata[tid] += sdata[tid + 16];
    if(blockSize >= 16) sdata[tid] += sdata[tid + 8];
    if(blockSize >=  8) sdata[tid] += sdata[tid + 4];
    if(blockSize >=  4) sdata[tid] += sdata[tid + 2];
    if(blockSize >=  2) sdata[tid] += sdata[tid + 1];
}

// TODO
// template<unsigned int blockSize, typename T>
// __device__ void warp_mean(
//     volatile T* sdata,
//     volatile unsigned int* sN, 
//     unsigned int tid)
// {
//     if(blockSize >= 64) 
//     {
//         volatile unsigned int N1 = sN[tid];
//         volatile unsigned int N2 = sN[tid + 32];
//         volatile unsigned int Ntot = N1 + N2;
//         volatile float w1 = static_cast<volatile float>(N1) / static_cast<volatile float>(Ntot);
//         volatile float w2 = static_cast<volatile float>(N2) / static_cast<volatile float>(Ntot);
//         volatile rm::Vector res = sdata[tid].mult(w1) + sdata[tid + 32].mult(w2);
//         sdata[tid].x = res.x;
//         sdata[tid].y = res.y;
//         sdata[tid].z = res.z;
//     }
//     if(blockSize >= 32)
//     {
//         const unsigned int N1 = sN[tid];
//         const unsigned int N2 = sN[tid + 16];
//         const unsigned int Ntot = N1 + N2;
//         const float w1 = static_cast<float>(N1) / static_cast<float>(Ntot);
//         const float w2 = static_cast<float>(N2) / static_cast<float>(Ntot);
//         sdata[tid] = sdata[tid] * w1 + sdata[tid + 16] * w2;
//     }
//     if(blockSize >= 16) 
//     {
//         const unsigned int N1 = sN[tid];
//         const unsigned int N2 = sN[tid + 8];
//         const unsigned int Ntot = N1 + N2;
//         const float w1 = static_cast<float>(N1) / static_cast<float>(Ntot);
//         const float w2 = static_cast<float>(N2) / static_cast<float>(Ntot);
//         sdata[tid] = sdata[tid] * w1 + sdata[tid + 8] * w2;
//     }
//     if(blockSize >= 8) 
//     {
//         const unsigned int N1 = sN[tid];
//         const unsigned int N2 = sN[tid + 4];
//         const unsigned int Ntot = N1 + N2;
//         const float w1 = static_cast<float>(N1) / static_cast<float>(Ntot);
//         const float w2 = static_cast<float>(N2) / static_cast<float>(Ntot);
//         sdata[tid] = sdata[tid] * w1 + sdata[tid + 4] * w2;
//     }
//     if(blockSize >= 4) 
//     {
//         const unsigned int N1 = sN[tid];
//         const unsigned int N2 = sN[tid + 2];
//         const unsigned int Ntot = N1 + N2;
//         const float w1 = static_cast<float>(N1) / static_cast<float>(Ntot);
//         const float w2 = static_cast<float>(N2) / static_cast<float>(Ntot);
//         sdata[tid] = sdata[tid] * w1 + sdata[tid + 2] * w2;
//     }
//     if(blockSize >= 2) 
//     {
//         const unsigned int N1 = sN[tid];
//         const unsigned int N2 = sN[tid + 1];
//         const unsigned int Ntot = N1 + N2;
//         const float w1 = static_cast<float>(N1) / static_cast<float>(Ntot);
//         const float w2 = static_cast<float>(N2) / static_cast<float>(Ntot);
//         sdata[tid] = sdata[tid] * w1 + sdata[tid + 1] * w2;
//     }
// }

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
        warp_sum<blockSize>(sdata, tid);
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
        warp_sum<blockSize>(sdata, tid);
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
        // s <= blockSize / 2 and tid < s so tid + s < blockSize. Good
        //  -> add '&& tid + s < chunkSize' to reduce this even more
        if(tid < s)
        {
            sdata[tid] += sdata[tid + s];
        }
        __syncthreads();
    }

    if(tid < blockSize / 2 && tid < 32)
    {
        warp_sum<blockSize>(sdata, tid);
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
        warp_sum<blockSize>(sdata, tid);
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
__global__ void means_covs_online_batched_kernel(
    const rm::Vector* dataset_points, // from, dataset
    const rm::Vector* model_points, // to, model
    const unsigned int* mask,
    unsigned int chunkSize,
    rm::Vector* dataset_center,
    rm::Vector* model_center,
    rm::Matrix3x3* Cs,
    unsigned int* Ncorr)
{
    // Online update: Covariance and means 
    // - https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    // 
    // Comment:
    // I dont think the Wikipedia algorithm is completely correct (Covariances).
    // But it is used in a lot of code -> Let everything be equally incorrect

    __shared__ rm::Vector sD[blockSize];
    __shared__ rm::Vector sM[blockSize];
    __shared__ rm::Matrix3x3 sC[blockSize];
    __shared__ unsigned int sN[blockSize];
    // __shared__ bool sMask[blockSize];
    

    const unsigned int tid = threadIdx.x;
    const unsigned int globId = chunkSize * blockIdx.x + threadIdx.x;
    const unsigned int rows = (chunkSize + blockSize - 1) / blockSize;

    // if(tid == 0)
    // {
    //     printf("blockSize: %u, chunkSize %u\n", blockSize, chunkSize);
    // }

    sD[tid] = {0.0f, 0.0f, 0.0f};
    sM[tid] = {0.0f, 0.0f, 0.0f};
    sC[tid].setZeros();
    sN[tid] = 0;

    for(unsigned int i=0; i<rows; i++)
    {
        if(tid + blockSize * i < chunkSize)
        {
            if(mask[globId + blockSize * i] > 0)
            {
                // global read
                const rm::Vector Di = dataset_points[globId + blockSize * i];
                const rm::Vector Mi = model_points[globId + blockSize * i];

                // shared read 
                const rm::Vector D      = sD[tid];
                const rm::Vector M      = sM[tid];
                const rm::Matrix3x3 C   = sC[tid];
                const unsigned int N_1    = sN[tid];
                const float N_1f = static_cast<float>(N_1);
                const unsigned int N = N_1 + 1;
                const float Nf = static_cast<float>(N);

                // compute
                // rm::Vector dD = Di - D;
                // rm::Vector dM = Mi - M;

                const rm::Vector Dnew = D + (Di - D) / Nf;
                const rm::Vector Mnew = M + (Mi - M) / Nf;

                // shared write
                // auto Cnew = P1 + P2 + P3;
                // sC[tid] = C * N_1f / Nf + (Mi - Mnew).multT(Di - Dnew) * 1.0 / Nf
                //     + (M - Mnew).multT(D - Dnew) * N_1f / Nf;
                
                const float w1 = N_1f / Nf;
                const float w2 = 1.0 / Nf;

                auto P1 = (Mi - Mnew).multT(Di - Dnew);
                auto P2 = (M - Mnew).multT(D - Dnew);                
                
                auto Cnew = C * w1 + P1 * w2 + P2 * w1;



                sC[tid] = Cnew;
                sD[tid] = Dnew;
                sM[tid] = Mnew;
                sN[tid] = N;
            }
        }
    }
    __syncthreads();
    
    for(unsigned int s = blockSize / 2; s > 0; s >>= 1)
    {
        if(tid < s && tid < chunkSize / 2)
        {
            // if(tid < s)
            // shared read 
            const rm::Vector D      = sD[tid];
            const rm::Vector M      = sM[tid];
            const rm::Matrix3x3 C   = sC[tid];
            const unsigned int N    = sN[tid];

            const rm::Vector Dnext      = sD[tid + s];
            const rm::Vector Mnext      = sM[tid + s];
            const rm::Matrix3x3 Cnext   = sC[tid + s];
            const unsigned int Nnext    = sN[tid + s];

            // compute
            const unsigned int Ntot = N + Nnext;

            if(Ntot > 0)
            {
                const float w1 = static_cast<float>(N) / static_cast<float>(Ntot);
                const float w2 = static_cast<float>(Nnext) / static_cast<float>(Ntot);

                const rm::Vector Dtot = D * w1 + Dnext * w2;
                const rm::Vector Mtot = M * w1 + Mnext * w2;
                
                auto P1 = C * w1 + Cnext * w2;
                auto P2 = (M - Mtot).multT(D - Dtot) * w1 + (Mnext - Mtot).multT(Dnext - Dtot) * w2;
                auto Cnew = P1 + P2;

                // shared write
                sD[tid] = Dtot;
                sM[tid] = Mtot;
                sC[tid] = Cnew; // currently only a sum
                sN[tid] = Ntot;
            }
        }
        __syncthreads();   
    }

    if(tid == 0)
    {
        const unsigned int N = sN[0];
        if(N > 0)
        {
            dataset_center[blockIdx.x] = sD[0];
            model_center[blockIdx.x] = sM[0];
            Cs[blockIdx.x] = sC[0];
            Ncorr[blockIdx.x] = N;
        } else {
            dataset_center[blockIdx.x].setZeros();
            model_center[blockIdx.x].setZeros();
            Cs[blockIdx.x].setZeros();
            Ncorr[blockIdx.x] = 0;
        }
    }
}


template<unsigned int blockSize>
__global__ void means_covs_online_approx_batched_kernel(
    const rm::Vector* dataset_points, // from, dataset
    const rm::Vector* model_points, // to, model
    const unsigned int* mask,
    unsigned int chunkSize,
    rm::Vector* dataset_center,
    rm::Vector* model_center,
    rm::Matrix3x3* Cs,
    unsigned int* Ncorr)
{
    // Online update: Covariance and means 
    // - https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    // 
    // Comment:
    // I dont think the Wikipedia algorithm is completely correct (Covariances).
    // But it is used in a lot of code -> Let everything be equally incorrect

    __shared__ rm::Vector sD[blockSize];
    __shared__ rm::Vector sM[blockSize];
    __shared__ rm::Matrix3x3 sC[blockSize];
    __shared__ unsigned int sN[blockSize];
    // __shared__ bool sMask[blockSize];
    

    const unsigned int tid = threadIdx.x;
    const unsigned int globId = chunkSize * blockIdx.x + threadIdx.x;
    const unsigned int rows = (chunkSize + blockSize - 1) / blockSize;

    // if(tid == 0)
    // {
    //     printf("blockSize: %u, chunkSize %u\n", blockSize, chunkSize);
    // }

    sD[tid] = {0.0f, 0.0f, 0.0f};
    sM[tid] = {0.0f, 0.0f, 0.0f};
    sC[tid].setZeros();
    sN[tid] = 0;

    for(unsigned int i=0; i<rows; i++)
    {
        if(tid + blockSize * i < chunkSize)
        {
            if(mask[globId + blockSize * i] > 0)
            {
                // shared read 
                const rm::Vector D      = sD[tid];
                const rm::Vector M      = sM[tid];
                const rm::Matrix3x3 C   = sC[tid];
                const unsigned int N    = sN[tid];

                // compute
                const rm::Vector dD = dataset_points[globId + blockSize * i] - D;
                const rm::Vector dM = model_points[globId + blockSize * i] - M;

                // shared write
                sD[tid] = D + dD / static_cast<float>(N + 1);
                sM[tid] = M + dM / static_cast<float>(N + 1);
                sC[tid] = C + dM.multT(dD);
                sN[tid] = N + 1;

                // printf("(%u,G) -> (%u,S) = %u\n", globId + blockSize * i, tid, N + 1);
            }
        }
    }
    __syncthreads();
    
    for(unsigned int s = blockSize / 2; s > 0; s >>= 1)
    {
        if(tid < s && tid < chunkSize / 2)
        {
            // if(tid < s)
            // shared read 
            const rm::Vector D      = sD[tid];
            const rm::Vector M      = sM[tid];
            const rm::Matrix3x3 C   = sC[tid];
            const unsigned int N    = sN[tid];

            const rm::Vector Dnext      = sD[tid + s];
            const rm::Vector Mnext      = sM[tid + s];
            const rm::Matrix3x3 Cnext   = sC[tid + s];
            const unsigned int Nnext    = sN[tid + s];

            // compute
            const unsigned int Ntot = N + Nnext;

            if(Ntot > 0)
            {
                const float w1 = static_cast<float>(N) / static_cast<float>(Ntot);
                const float w2 = static_cast<float>(Nnext) / static_cast<float>(Ntot);

                const rm::Vector Dtot = D * w1 + Dnext * w2;
                // printf("(%u,S) %u, (%f, %f, %f) -> (%u,S) %u, (%f, %f, %f) = %u, (%f, %f, %f)\n", 
                //     tid + s, Nnext, Dnext.x, Dnext.y, Dnext.z,
                //     tid, N, D.x, D.y, D.z,
                //     Ntot, Dtot.x, Dtot.y, Dtot.z);
                
                // shared write
                sD[tid] = Dtot;
                sM[tid] = M * w1 + Mnext * w2;
                sC[tid] = C + Cnext; // currently only a sum
                sN[tid] = Ntot;
            }
        }
        __syncthreads();   
    }

    if(tid == 0)
    {
        const unsigned int N = sN[0];
        if(N > 0)
        {
            dataset_center[blockIdx.x] = sD[0];
            model_center[blockIdx.x] = sM[0];
            Cs[blockIdx.x] = sC[0] / static_cast<float>(N);
            Ncorr[blockIdx.x] = N;
        } else {
            dataset_center[blockIdx.x].setZeros();
            model_center[blockIdx.x].setZeros();
            Cs[blockIdx.x].setZeros();
            Ncorr[blockIdx.x] = 0;
        }
    }
}

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

void means_covs_batched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& dataset_points, // from
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& model_points, // to
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& dataset_center,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& model_center,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr)
{
    rm::sumBatched(mask, Ncorr);

    mean_batched(dataset_points, mask, Ncorr, dataset_center);
    mean_batched(model_points, mask, Ncorr, model_center);

    cov_batched(dataset_points, dataset_center,
            model_points, model_center,
            mask, Ncorr, Cs);
}

void means_covs_online_batched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& dataset_points, // from
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& model_points, // to
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& dataset_center,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& model_center,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr)
{
    unsigned int Nbatches = Ncorr.size();
    unsigned int batchSize = dataset_points.size() / Nbatches;
    constexpr unsigned int blockSize = 64; // TODO: get best value for this one

    means_covs_online_batched_kernel<blockSize> <<<Nbatches, blockSize>>>(
        dataset_points.raw(), model_points.raw(), mask.raw(), batchSize, 
        dataset_center.raw(), model_center.raw(), Cs.raw(), Ncorr.raw());
}


void means_covs_online_approx_batched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& dataset_points, // from
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& model_points, // to
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& dataset_center,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& model_center,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr)
{
    unsigned int Nbatches = Ncorr.size();
    unsigned int batchSize = dataset_points.size() / Nbatches;
    constexpr unsigned int blockSize = 64; // TODO: get best value for this one

    means_covs_online_approx_batched_kernel<blockSize> <<<Nbatches, blockSize>>>(
        dataset_points.raw(), model_points.raw(), mask.raw(), batchSize, 
        dataset_center.raw(), model_center.raw(), Cs.raw(), Ncorr.raw());
}

} // namespace rmcl