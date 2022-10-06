#ifndef RMCL_MATH_BATCHED_CUH
#define RMCL_MATH_BATCHED_CUH

#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/math/types.h>


namespace rmcl
{

/**
 * @brief 
 * 
 * - if Ncorr[i] == 0: res[i] get only zeros entries
 * 
 * @param data 
 * @param mask 
 * @param Ncorr 
 * @param res 
 */
void meanBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& res);

rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA> meanBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr);

void sumFancyBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center2,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs);

rmagine::Memory<rmagine::Matrix3x3, rmagine::VRAM_CUDA> sumFancyBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data1, // from
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data2, // to
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center2,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask);

/**
 * @brief Returns covariance matrices of model and dataset points including their centers
 *  - masked version. better for GPU
 *  - if Ncorr[i] == 0: Cs[i] get only zeros entries
 * 
 * @param data1 size = points * poses
 * @param center1 size = poses
 * @param data2 size = points * poses
 * @param center2 size = poses
 * @param mask size = points * poses
 * @param Ncorr size = poses
 * @param Cs size = poses
 */
void covFancyBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center2,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs);

rmagine::Memory<rmagine::Matrix3x3, rmagine::VRAM_CUDA> covFancyBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data1, // from
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data2, // to
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center2,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr);


/**
 * @brief computes covariances of correspondences that are already centered 
 * and in the same coordinate system
 * 
 * data1[0] -> data2[0] if mask[0] == 1
 * data1[1] -> data2[1] if mask[1] == 1
 * ...
 * data1[N] -> data2[N] if mask[N] == 1
 * 
 * 
 * @param data1 
 * @param data2 
 * @param mask 
 * @param Ncorr 
 * @param Cs 
 * @return * compute 
 */
void covFancyBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data1, // from, dataset
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data2, // to, model
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs);

rmagine::Memory<rmagine::Matrix3x3, rmagine::VRAM_CUDA> covFancyBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data1, // from, dataset
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data2, // to, model
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr);

} // namespace rmcl

#endif // RMCL_MATH_BATCHED_CUH