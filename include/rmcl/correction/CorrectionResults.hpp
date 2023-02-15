/**
 * Copyright (c) 2021, University Osnabrück
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University Osnabrück nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL University Osnabrück BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * CorrectionResults.hpp
 *
 *  Created on: Jul 17, 2021
 *      Author: Alexander Mock
 */

#ifndef RMCL_CORRECTION_CORRECTION_RESULTS_HPP
#define RMCL_CORRECTION_CORRECTION_RESULTS_HPP

#include <rmagine/types/Memory.hpp>
#include <rmagine/math/types.h>

namespace rmcl {

template<typename MemT>
struct PointToPlaneCorrespondencesView
{
    rmagine::MemoryView<rmagine::Vector, MemT>  dataset_points;
    rmagine::MemoryView<rmagine::Vector, MemT>  model_points;
    rmagine::MemoryView<rmagine::Vector, MemT>  model_normals;
    rmagine::MemoryView<unsigned int, MemT>     corr_valid;
};

template<typename MemT>
struct PointToPlaneCorrespondences
{
    rmagine::Memory<rmagine::Vector, MemT>  dataset_points;
    rmagine::Memory<rmagine::Vector, MemT>  model_points;
    rmagine::Memory<rmagine::Vector, MemT>  model_normals;
    rmagine::Memory<unsigned int, MemT>     corr_valid;
};

template<typename MemT>
struct PointToPointCorrespondencesView
{
    rmagine::MemoryView<rmagine::Vector, MemT>  dataset_points;
    rmagine::MemoryView<rmagine::Vector, MemT>  model_points;
    rmagine::MemoryView<unsigned int, MemT>     corr_valid;
};

template<typename MemT>
struct PointToPointCorrespondences
{
    rmagine::Memory<rmagine::Vector, MemT>  dataset_points;
    rmagine::Memory<rmagine::Vector, MemT>  model_points;
    rmagine::Memory<unsigned int, MemT>     corr_valid;

    
    inline PointToPointCorrespondencesView<MemT> slice(
        unsigned int start, unsigned int end) const
    {
        return {
            dataset_points.slice(start, end),
            model_points.slice(start, end),
            corr_valid.slice(start, end)
        };
    }

    inline PointToPointCorrespondencesView<MemT> view() const
    {
        return {
            dataset_points,
            model_points,
            corr_valid
        };
    }


    // Enable this function?
    // inline std::vector<CorrespondencesView<MemT> > split(
    //     unsigned int N) const
    // {
    //     std::vector<CorrespondencesView<MemT> > ret;

    //     unsigned int batch_size = dataset_points.size() / N;

    //     for(unsigned int i=0; i<N; i++)
    //     {
    //         ret.push_back({
    //             dataset_points(i * batch_size, (i+1) * batch_size),
    //             model_points(i * batch_size, (i+1) * batch_size),
    //             corr_valid(i * batch_size, (i+1) * batch_size)
    //         });
    //     }

    //     return ret;
    // }
};

template<typename MemT>
using Correspondences = PointToPointCorrespondences<MemT>;

template<typename MemT>
using CorrespondencesView = PointToPointCorrespondencesView<MemT>;


template<typename MemT>
struct CorrectionPreResults 
{
    // dataset means (from)
    rmagine::Memory<rmagine::Vector, MemT>      ds;
    // model means (to)
    rmagine::Memory<rmagine::Vector, MemT>      ms;
    // covs
    rmagine::Memory<rmagine::Matrix3x3, MemT>   Cs;
    // number of correspondences
    rmagine::Memory<unsigned int, MemT>         Ncorr;
};

template<typename MemT>
struct CorrectionResults
{
    rmagine::Memory<rmagine::Transform, MemT>    Tdelta;
    // Scan specific? 
    rmagine::Memory<unsigned int,       MemT>    Ncorr;
};

} // namespace rmcl

#endif // RMCL_CORRECTION_CORRECTION_RESULTS_HPP