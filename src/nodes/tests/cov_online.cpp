
#include <rmcl/math/math_batched.h>
#include <random>
#include <iostream>
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmcl/math/math_batched.cuh>

namespace rm = rmagine;

void fill_random(rm::MemoryView<rm::Vector, rm::RAM> points)
{
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0.0, 1.0);
    for(unsigned int i = 0; i < points.size(); i++) 
    {
        points[i].x = dis(gen);
        points[i].y = dis(gen);
        points[i].z = dis(gen);
    }
}

void fill_sequence(rm::MemoryView<rm::Vector, rm::RAM> points)
{
    for(unsigned int i = 0; i < points.size(); i++) 
    {
        points[i].x = (float)i;
        points[i].y = (float)i;
        points[i].z = (float)i;
    }
}

void fill_ones(rm::MemoryView<unsigned int> mask)
{
    for(unsigned int i=0; i < mask.size(); i++)
    {
        mask[i] = i % 2;
    }
}

int main(int argc, char** argv)
{
    rm::Mem<rm::Vector> dataset(10000);
    fill_random(dataset);
    // fill_sequence(dataset);

    rm::Mem<rm::Vector> model(dataset.size());

    rm::Mem<unsigned int> mask(dataset.size());
    fill_ones(mask);


    rm::Transform T;
    T.t = {1.0, 2.0, 3.0};
    T.R.set(rm::EulerAngles{0.0, 0.1, 0.1});

    for(unsigned int i=0; i<dataset.size(); i++)
    {
        model[i] = T * dataset[i];
    }

    rm::StopWatchHR sw;
    double el;
    rm::Mem<rm::Vector> ds(1);
    rm::Mem<rm::Vector> ms(1);
    rm::Mem<rm::Matrix3x3> Cs(1);
    rm::Mem<unsigned int> Ncorr(1);

    std::cout << "CPU" << std::endl;
    {
        std::cout << "means_covs_batched" << std::endl;
        sw();
        rmcl::means_covs_batched(dataset, model, mask, ds, ms, Cs, Ncorr);
        el = sw();
        std::cout << "- runtime: " << el * 1000.0 << " ms" << std::endl;
        std::cout << ds[0] << std::endl;
        std::cout << ms[0] << std::endl;
        std::cout << Cs[0] << std::endl;
        std::cout << Ncorr[0] << std::endl;

        std::cout << "means_covs_online_approx_batched" << std::endl;
        sw();
        rmcl::means_covs_online_approx_batched(dataset, model, mask, ds, ms, Cs, Ncorr);
        el = sw();
        std::cout << "- runtime: " << el * 1000.0 << " ms" << std::endl;
        std::cout << ds[0] << std::endl;
        std::cout << ms[0] << std::endl;
        std::cout << Cs[0] << std::endl;
        std::cout << Ncorr[0] << std::endl;

        std::cout << "means_covs_online_batched" << std::endl;
        sw();
        rmcl::means_covs_online_batched(dataset, model, mask, ds, ms, Cs, Ncorr);
        el = sw();
        std::cout << "- runtime: " << el * 1000.0 << " ms" << std::endl;
        std::cout << ds[0] << std::endl;
        std::cout << ms[0] << std::endl;
        std::cout << Cs[0] << std::endl;
        std::cout << Ncorr[0] << std::endl;
    }

    std::cout << std::endl;
    std::cout << "GPU" << std::endl;
    {
        rm::Mem<rm::Vector, rm::VRAM_CUDA> dataset_ = dataset;
        rm::Mem<rm::Vector, rm::VRAM_CUDA> model_ = model;
        rm::Mem<unsigned int, rm::VRAM_CUDA> mask_ = mask;

        rm::Mem<rm::Vector, rm::VRAM_CUDA> ds_(1);
        rm::Mem<rm::Vector, rm::VRAM_CUDA> ms_(1);
        rm::Mem<rm::Matrix3x3, rm::VRAM_CUDA> Cs_(1);
        rm::Mem<unsigned int, rm::VRAM_CUDA> Ncorr_(1);

        std::cout << "means_covs_batched" << std::endl;
        sw();
        rmcl::means_covs_batched(dataset_, model_, mask_, ds_, ms_, Cs_, Ncorr_);
        el = sw();
        std::cout << "- runtime: " << el * 1000.0 << " ms" << std::endl;
        ds = ds_; ms = ms_; Cs = Cs_; Ncorr = Ncorr_;
        std::cout << ds[0] << std::endl;
        std::cout << ms[0] << std::endl;
        std::cout << Cs[0] << std::endl;
        std::cout << Ncorr[0] << std::endl;

        std::cout << "means_covs_online_approx_batched" << std::endl;
        sw();
        rmcl::means_covs_online_approx_batched(dataset_, model_, mask_, ds_, ms_, Cs_, Ncorr_);
        el = sw();
        std::cout << "- runtime: " << el * 1000.0 << " ms" << std::endl;
        ds = ds_; ms = ms_; Cs = Cs_; Ncorr = Ncorr_;
        std::cout << ds[0] << std::endl;
        std::cout << ms[0] << std::endl;
        std::cout << Cs[0] << std::endl;
        std::cout << Ncorr[0] << std::endl;

        std::cout << "means_covs_online_batched" << std::endl;
        sw();
        rmcl::means_covs_online_batched(dataset_, model_, mask_, ds_, ms_, Cs_, Ncorr_);
        el = sw();
        std::cout << "- runtime: " << el * 1000.0 << " ms" << std::endl;
        ds = ds_; ms = ms_; Cs = Cs_; Ncorr = Ncorr_;
        std::cout << ds[0] << std::endl;
        std::cout << ms[0] << std::endl;
        std::cout << Cs[0] << std::endl;
        std::cout << Ncorr[0] << std::endl;
    }

    return 0;
}