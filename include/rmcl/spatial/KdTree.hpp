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
 * KdTree.hpp
 *
 *  Created on: Jul 17, 2021
 *      Author: Alexander Mock
 */

#ifndef MAMCL_SPATIAL_KDTREE_HPP
#define MAMCL_SPATIAL_KDTREE_HPP


#include <rmagine/types/Memory.hpp>
#include <rmagine/math/types.h>
#include <nanoflann.hpp>
#include <memory>

#include <vector>

namespace rmcl {

class KdPoints
{
public:
    KdPoints(const rmagine::Memory<rmagine::Vector, rmagine::RAM>& mem)
    :m_mem(mem)
    {

    }

    inline size_t kdtree_get_point_count() const { 
        return m_mem.size();
    }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
        if(dim == 0) {
            return m_mem[idx].x;
        } else if(dim == 1) {
            return m_mem[idx].y;
        } else if(dim == 2) {
            return m_mem[idx].z;
        } else {
            throw std::runtime_error("Tried to access dim 4 on vector");
            return 0.0;
        }
	}

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

    const rmagine::Memory<rmagine::Vector, rmagine::RAM>& m_mem;
};

using KdPointsPtr = std::shared_ptr<KdPoints>;

class KdTree : public nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<float, KdPoints > ,
		KdPoints, 3 /* dim */>
{
public:
    using Super = nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<float, KdPoints > ,
		KdPoints, 3 /* dim */ >;

    KdTree(KdPointsPtr points);

    rmagine::Vector nearest(
        const rmagine::Vector& query_point,
        bool query_in_index = false) const;

    float nearestDist(
        const rmagine::Vector& query_point,
        bool query_in_index = false) const;

    size_t nearestId(
        const rmagine::Vector& query_point,
        bool query_in_index = false) const;

    const KdPointsPtr dataset() const
    {
        return m_points;
    }

protected:
    KdPointsPtr m_points;
};

using KdTreePtr = std::shared_ptr<KdTree>;

} // namespace rmcl

#endif // RMCL_SPATIAL_KDTREE_HPP