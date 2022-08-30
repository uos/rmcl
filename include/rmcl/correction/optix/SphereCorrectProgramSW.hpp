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
 * ScanCorrectProgramSW.hpp
 *
 *  Created on: Jul 17, 2021
 *      Author: Alexander Mock
 */

#ifndef RMCL_CORRECTION_OPTIX_SPHERE_CORRECT_PROGRAM_SW_HPP
#define RMCL_CORRECTION_OPTIX_SPHERE_CORRECT_PROGRAM_SW_HPP

#include <rmagine/map/OptixMap.hpp>
#include <rmagine/util/optix/optix_modules.h>


namespace rmcl {

rmagine::ProgramModulePtr make_program_module_corr_gen(
    rmagine::OptixScenePtr scene,
    unsigned int sensor_id
);

// typedef rmagine::SbtRecord<rmagine::HitGroupDataNormals>   HitGroupSbtRecord;

/**
 * @brief Scan-wise parallelization
 * 
 */
class SphereCorrectProgramSW
{
public:
    SphereCorrectProgramSW(rmagine::OptixMapPtr map);
private:
    HitGroupSbtRecord m_hg_sbt;
};

} // namespace rmcl

#endif // RMCL_CORRECTION_OPTIX_SPHERE_CORRECT_PROGRAM_SW_HPP