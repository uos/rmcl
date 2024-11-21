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


    // timings:
    // - 0: measure outer timings
    // - 1: measure inner timings

    int timings = 0;

    if(timings == 0)
    {
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

    } else if(timings == 1) {
        auto bres = correct.benchmark(T_curr_, Nruns);
        std::cout << "Absolute:"  << std::endl;
        std::cout << "- Sim: " << bres.sim << std::endl;
        std::cout << "- Red: " << bres.red << std::endl;
        std::cout << "- SVD: " << bres.svd << std::endl;

        double total = bres.sim + bres.red + bres.svd;
        std::cout << "- Total: " << total <<  std::endl;
        std::cout << "- Relative: " << std::endl;
        std::cout << bres.sim / total << "," << bres.red / total << "," << bres.svd / total << std::endl;
    }

    // TIMINGS 0

    // DPC (AMOCKHOME)
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

    // NUC
    // #faces, #poses, correction runtime
    // 100000,1000,0.0131995
    // 500000,1000,0.0192545
    // 1000000,1000,0.0231976
    // 2000000,1000,0.0307703
    // 3000000,1000,0.0365178
    // 4000000,1000,0.0360314
    // 6000000,1000,0.0400326
    // 8000000,1000,0.0433546
    // 10000000,1000,0.0449058

    // LX1
    // #faces, #poses, correction runtime
    // 100000,1000,0.133156
    // 500000,1000,0.167014
    // 1000000,1000,0.185449
    // 2000000,1000,0.208977
    // 3000000,1000,0.235557
    // 4000000,1000,0.241927
    // 6000000,1000,0.259021
    // 8000000,1000,0.270712
    // 10000000,1000,0.273286


    // TIMINGS 1
    // args 1000000 1000

    // DPC (AMOCKHOME)
    // 
    // Absolute:
    // - Sim: 0.0119368
    // - Red: 1.32316e-05
    // - SVD: 0.0028495
    // - Total: 0.0147996
    // - Relative: 
    // 0.806567,0.000894055,0.192539

    // NUC
    //
    // Absolute:
    // - Sim: 0.0181216
    // - Red: 8.49177e-05
    // - SVD: 0.00435061
    // - Total: 0.0225571
    // - Relative: 
    // 0.803365,0.00376456,0.192871

    // LX1
    //
    //




    return 0;
}