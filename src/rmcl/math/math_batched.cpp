#include "rmcl/math/math_batched.h"

#include <rmagine/math/math_batched.h>

namespace rm = rmagine;

namespace rmcl
{

void means_covs_batched(
    const rm::MemoryView<rm::Vector, rm::RAM>& dataset_points, // from
    const rm::MemoryView<rm::Vector, rm::RAM>& model_points, // to
    const rm::MemoryView<unsigned int, rm::RAM>& mask,
    rm::MemoryView<rm::Vector, rm::RAM>& dataset_center,
    rm::MemoryView<rm::Vector, rm::RAM>& model_center,
    rm::MemoryView<rm::Matrix3x3, rm::RAM>& Cs,
    rm::MemoryView<unsigned int, rm::RAM>& Ncorr)
{
    unsigned int Nbatches = Ncorr.size();
    unsigned int batchSize = dataset_points.size() / Nbatches;

    #pragma omp parallel for default(shared) if(Nbatches > 4)
    for(size_t i=0; i<Nbatches; i++)
    {
        const rm::MemoryView<rm::Vector> data_batch = dataset_points(i * batchSize, (i+1) * batchSize);
        const rm::MemoryView<rm::Vector> model_batch = model_points(i * batchSize, (i+1) * batchSize);
        const rm::MemoryView<unsigned int> mask_batch = mask(i * batchSize, (i+1) * batchSize);

        rm::Vector d_mean = {0.0f, 0.0f, 0.0f};
        rm::Vector m_mean = {0.0f, 0.0f, 0.0f};
        unsigned int n_corr = 0;

        for(size_t j=0; j<batchSize; j++)
        {
            if(mask_batch[j] > 0)
            {
                const rm::Vector dD = data_batch[j] - d_mean;
                const rm::Vector dM = model_batch[j] - m_mean;
                float N = static_cast<float>(n_corr + 1);

                // reduction
                n_corr++;
                d_mean += dD / N;
                m_mean += dM / N;
            }
        }

        Ncorr[i] = n_corr;

        if(n_corr > 0)
        {
            rm::Matrix3x3 C = rm::Matrix3x3::Zeros();

            for(size_t j = 0; j < batchSize; j++)
            {
                C += (model_batch[j] - m_mean).multT(data_batch[j] - d_mean);
            }

            const float Ncorr_f = static_cast<float>(n_corr);
            C /= Ncorr_f;

            dataset_center[i] = d_mean;
            model_center[i] = m_mean;
            Cs[i] = C;
        } else {
            dataset_center[i] = {0.0f, 0.0f, 0.0f};
            model_center[i] = {0.0f, 0.0f, 0.0f};
            Cs[i].setZeros();
        }
    }
}

void means_covs_online_batched(
    const rm::MemoryView<rm::Vector, rm::RAM>& dataset_points, // from
    const rm::MemoryView<rm::Vector, rm::RAM>& model_points, // to
    const rm::MemoryView<unsigned int, rm::RAM>& mask,
    rm::MemoryView<rm::Vector, rm::RAM>& dataset_center,
    rm::MemoryView<rm::Vector, rm::RAM>& model_center,
    rm::MemoryView<rm::Matrix3x3, rm::RAM>& Cs,
    rm::MemoryView<unsigned int, rm::RAM>& Ncorr)
{
    unsigned int Nbatches = Ncorr.size();
    unsigned int batchSize = dataset_points.size() / Nbatches;

    #pragma omp parallel for default(shared) if(Nbatches > 4)
    for(size_t i=0; i<Nbatches; i++)
    {
        const rm::MemoryView<rm::Vector> data_batch = dataset_points(i * batchSize, (i+1) * batchSize);
        const rm::MemoryView<rm::Vector> model_batch = model_points(i * batchSize, (i+1) * batchSize);
        const rm::MemoryView<unsigned int> mask_batch = mask(i * batchSize, (i+1) * batchSize);

        rm::Vector d_mean = {0.0f, 0.0f, 0.0f};
        rm::Vector m_mean = {0.0f, 0.0f, 0.0f};
        rm::Matrix3x3 C = rm::Matrix3x3::Zeros();
        unsigned int n_corr = 0;

        for(size_t j=0; j<batchSize; j++)
        {
            if(mask_batch[j] > 0)
            {
                const rm::Vector dD = data_batch[j] - d_mean;
                const rm::Vector dM = model_batch[j] - m_mean;
                float N = static_cast<float>(n_corr + 1);

                // reduction
                n_corr++;
                d_mean += dD / N;
                m_mean += dM / N;
                C += dM.multT(dD);
            }
        }

        Ncorr[i] = n_corr;

        if(n_corr > 0)
        {
            const float Ncorr_f = static_cast<float>(n_corr);
            C /= Ncorr_f;

            dataset_center[i] = d_mean;
            model_center[i] = m_mean;
            Cs[i] = C;
        } else {
            dataset_center[i] = {0.0f, 0.0f, 0.0f};
            model_center[i] = {0.0f, 0.0f, 0.0f};
            Cs[i].setZeros();
        }
    }
}

} // namespace rmcl