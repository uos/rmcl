#ifndef RMCL_MATH_BATCHED_CUH
#define RMCL_MATH_BATCHED_CUH

#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/math/types.h>


namespace rmcl
{

void sumFancyBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center2,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs
);

rmagine::Memory<rmagine::Matrix3x3, rmagine::VRAM_CUDA> sumFancyBatched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data1, // from
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& data2, // to
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& center2,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& mask);

} // namespace rmcl

#endif // RMCL_MATH_BATCHED_CUH