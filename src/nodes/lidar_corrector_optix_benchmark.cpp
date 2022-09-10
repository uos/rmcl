#include <iostream>
#include <rmagine/util/synthetic.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/map/optix/optix_shapes.h>
#include <rmagine/types/sensors.h>
#include <rmagine/math/math.cuh>
#include <rmagine/util/prints.h>

#include <rmcl/correction/SphereCorrectorOptix.hpp>

#include <fstream>


using namespace rmagine;
using namespace rmcl;

bool find_abc(unsigned int& A, unsigned int& B, unsigned int C)
{
    A = static_cast<unsigned int>( sqrt(C) );
    B = A;

    unsigned int res = A * B;

    while(res != C && B > 0 && A <= C)
    {
        if(res > C)
        {
            B--;
        } else {
            A++;
        }
        res = A * B;
    }

    return (B > 0 && A <= C && res == C);
}

OptixMapPtr make_sphere_map(unsigned int Nfaces)
{
    // N = Nfaces / 2

    // A*B = N
    // 
    // A = N / B
    // B = N / A
    // A, B must be integer. N / B must be possible

    if(Nfaces % 2 == 1)
    {
        throw std::runtime_error("Nfaces must be even");
    }


    size_t N = Nfaces / 2;

    unsigned int A, B;
    if(!find_abc(A, B, N) )
    {
        throw std::runtime_error("Could not find integers for A*B=N");
    }

    std::cout << A << "x" << B << " * 2 = " << Nfaces << std::endl;

    OptixMeshPtr sphere_mesh = std::make_shared<OptixSphere>(A, B);
    sphere_mesh->commit();
    OptixScenePtr sphere_scene = std::make_shared<OptixScene>();
    sphere_scene->add(sphere_mesh);
    sphere_scene->commit();
    return std::make_shared<OptixMap>(sphere_scene);
}

int main(int argc, char** argv)
{
    // TODO: somewhere is memory that is not be freed when the scene is overwritten

    size_t Nfaces = atoi(argv[1]);
    size_t Nposes = atoi(argv[2]);
    size_t Nruns = 1000;


    std::cout << "Running benchmark with " << Nfaces << " faces and " << Nposes << " poses" << std::endl;

    OptixMapPtr sphere_map_tmp = make_sphere_map(Nfaces);
    
    SphereCorrectorOptix correct(sphere_map_tmp);
    correct.setTsb(Transform::Identity());

    // return 0;

    auto model = vlp16_900();
    model.range.min = 0.0;
    correct.setModel(model);
    
    using ResultT = Bundle<
        Ranges<VRAM_CUDA>
    >;

    ResultT sim_res;
    sim_res.ranges.resize(model.size());
    
    Memory<Transform, RAM> T_dest(1);
    T_dest[0] = Transform::Identity();


    StopWatch sw;
    double el;

    Memory<Transform, RAM> T_curr(Nposes);
    for(size_t i=0; i<T_curr.size(); i++)
    {
        T_curr[i] = Transform::Identity();
        T_curr[i].t.z += 0.2;
    }
    
    Memory<Transform, VRAM_CUDA> T_dest_ = T_dest;
    Memory<Transform, VRAM_CUDA> T_curr_ = T_curr;

    // simulate the data that would be recorded at destination
    correct.simulate(T_dest_, sim_res);
    correct.setInputData(sim_res.ranges);

    double el_total = 0.0;
    

    for(size_t i=0; i<Nruns; i++)
    {
        sw();
        auto corr_res = correct.correct(T_curr_);
        el = sw();
        el_total += el;
        T_curr_ = multNxN(T_curr_, corr_res.Tdelta);
    }

    std::cout << "- runtime: " << std::endl;
    std::cout << Nfaces << "," << Nposes << "," << el_total/static_cast<double>(Nruns) << std::endl;


    // AMOCKHOME
    // #faces, #poses, correction runtime 
    // 100000,1000,0.0135733
    // 500000,1000,0.0141381
    // 1000000,1000,0.0169034
    // 2000000,1000,0.0212061
    // 3000000,1000,0.0242707
    // 4000000,1000,0.0256912
    // 6000000,1000,0.0278478
    // 8000000,1000,0.0291446
    // 10000000,1000,0.0311896

    


    return 0;
}