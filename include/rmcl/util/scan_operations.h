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
#include <rmcl_msgs/Scan.h>

namespace rmcl {

static void fill(
    rmcl_msgs::Scan& scan, 
    const rmagine::Memory<float, rmagine::RAM>& ranges)
{
    rmagine::SphericalModel model;
    convert(scan.info, model);
    
    scan.data.ranges.resize(ranges.size());
    for(unsigned int vid = 0; vid < model.getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < model.getWidth(); hid++)
        {
            const unsigned int loc_id = model.getBufferId(vid, hid);
            scan.data.ranges[loc_id] = ranges[loc_id];
        }
    }
}

static void fillEmpty(rmcl_msgs::Scan& scan)
{
    rmagine::SphericalModel model;
    convert(scan.info, model);
    
    scan.data.ranges.resize(model.size());
    for(unsigned int vid = 0; vid < model.getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < model.getWidth(); hid++)
        {
            const unsigned int loc_id = model.getBufferId(vid, hid);
            scan.data.ranges[loc_id] = scan.info.range_max + 1.0;
        }
    }
}

} // namespace rmcl

#endif // RMCL_UTIL_SCAN_OPERATIONS_H