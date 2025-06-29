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
 * scan_operations.hpp
 *
 *  Created on: Jul 17, 2021
 *      Author: Alexander Mock
 */

#ifndef RMCL_UTIL_SCAN_OPERATIONS_H
#define RMCL_UTIL_SCAN_OPERATIONS_H

#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmcl_msgs/msg/scan.hpp>
#include <rmcl_msgs/msg/o1_dn_stamped.hpp>

namespace rmcl {

void fill(
    rmcl_msgs::msg::Scan& scan, 
    const rmagine::Memory<float, rmagine::RAM>& ranges);

void fillEmpty(rmcl_msgs::msg::Scan& scan);


struct IterationOptions
{
  size_t skip_begin = 0;
  size_t skip_end = 0;
  size_t increment = 0;
};

struct FilterOptions2D
{
  float range_min;
  float range_max;

  IterationOptions width;
  IterationOptions height;
};

struct FilterOptions1D
{
  float range_min;
  float range_max;

  IterationOptions skip;
};

void filter(
  rmcl_msgs::msg::O1Dn& o1dn_out,
  const rmcl_msgs::msg::O1Dn& o1dn_in, 
  const FilterOptions2D& options);


} // namespace rmcl

#endif // RMCL_UTIL_SCAN_OPERATIONS_H