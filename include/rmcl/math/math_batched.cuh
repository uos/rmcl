#ifndef RMCL_MATH_BATCHED_CUH
#define RMCL_MATH_BATCHED_CUH

#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/math/types.h>


namespace rmcl
{

void sumFancyBatched(
    const rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& data1,
    const rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& center1,
    const rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& data2,
    const rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& center2,
    const rmagine::Memory<unsigned int, rmagine::VRAM_CUDA>& mask,
    rmagine::Memory<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs
);

rmagine::Memory<rmagine::Matrix3x3, rmagine::VRAM_CUDA> sumFancyBatched(
    const rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& data1,
    const rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& center1,
    const rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& data2,
    const rmagine::Memory<rmagine::Vector, rmagine::VRAM_CUDA>& center2,
    const rmagine::Memory<unsigned int, rmagine::VRAM_CUDA>& mask);

} // namespace rmcl

#endif // RMCL_MATH_BATCHED_CUH