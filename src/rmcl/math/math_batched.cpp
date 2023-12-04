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
                // sum
                n_corr++;
                d_mean += data_batch[j];
                m_mean += model_batch[j];
            }
        }

        Ncorr[i] = n_corr;

        if(n_corr > 0)
        {
            const float Ncorr_f = static_cast<float>(n_corr);
            d_mean /= Ncorr_f;
            m_mean /= Ncorr_f;

            rm::Matrix3x3 C = rm::Matrix3x3::Zeros();

            for(size_t j = 0; j < batchSize; j++)
            {
                if(mask_batch[j] > 0)
                {
                    C += (model_batch[j] - m_mean).multT(data_batch[j] - d_mean);
                }
            }
            
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
                const float N_1 = static_cast<float>(n_corr);
                const float N = static_cast<float>(n_corr + 1);
                const float w1 = N_1/N;
                const float w2 = 1.0/N;

                const rm::Vector Di = data_batch[j]; // read
                const rm::Vector Mi = model_batch[j]; // read
                const rm::Vector d_mean_old = d_mean; // read
                const rm::Vector m_mean_old = m_mean; // read

                // update means
                
                // save old means for covariance
                
                const rm::Vector d_mean_new = d_mean_old * w1 + Di * w2; 
                const rm::Vector m_mean_new = m_mean_old * w1 + Mi * w2;

                auto P1 = (Mi - m_mean_new).multT(Di - d_mean_new);
                auto P2 = (m_mean_old - m_mean_new).multT(d_mean_old - d_mean_new);

                // Write
                C = C * w1 + P1 * w2 + P2 * w1;
                d_mean = d_mean_new; // write
                m_mean = m_mean_new; // write
                n_corr = n_corr + 1;
            }
        }

        Ncorr[i] = n_corr;
        dataset_center[i] = d_mean;
        model_center[i] = m_mean;
        Cs[i] = C;
    }
}


// Poses: N
// Scan size: M
void means_covs_p2l_online_batched(
    const rm::MemoryView<rm::Transform, rm::RAM>& pre_transforms, // N
    const rm::MemoryView<rm::Vector, rm::RAM>& dataset_points, // from, M
    const rm::MemoryView<unsigned int, rm::RAM>& dataset_mask, // M
    const rm::MemoryView<rm::Vector, rm::RAM>& model_points, // to, NxM
    const rm::MemoryView<rm::Vector, rm::RAM>& model_normals, // to, NxM
    const rm::MemoryView<unsigned int, rm::RAM>& model_mask, // NxM
    float max_corr_dist,
    rm::MemoryView<rm::Vector, rm::RAM>& dataset_center, // N
    rm::MemoryView<rm::Vector, rm::RAM>& model_center, // N
    rm::MemoryView<rm::Matrix3x3, rm::RAM>& Cs, // N
    rm::MemoryView<unsigned int, rm::RAM>& Ncorr // N
    )
{
    unsigned int Nbatches = pre_transforms.size();
    unsigned int batchSize = dataset_points.size() / Nbatches;

    #pragma omp parallel for default(shared) if(Nbatches > 4)
    for(size_t i=0; i<Nbatches; i++)
    {
        const rm::Transform Tpre = pre_transforms[i];
        const rm::MemoryView<rm::Vector> data_batch = dataset_points(i * batchSize, (i+1) * batchSize);
        const rm::MemoryView<rm::Vector> model_batch = model_points(i * batchSize, (i+1) * batchSize);
        const rm::MemoryView<rm::Vector> model_normal_batch = model_normals(i * batchSize, (i+1) * batchSize);
        const rm::MemoryView<unsigned int> model_mask_batch = model_mask(i * batchSize, (i+1) * batchSize);

        rm::Vector d_mean = {0.0f, 0.0f, 0.0f};
        rm::Vector m_mean = {0.0f, 0.0f, 0.0f};
        rm::Matrix3x3 C = rm::Matrix3x3::Zeros();
        unsigned int n_corr = 0;

        for(size_t j=0; j<batchSize; j++)
        {
            // figure out if distance is too high

            if(dataset_mask[j] > 0 && model_mask_batch[j] > 0)
            {
                const rm::Vector Di = Tpre * data_batch[j]; // read
                const rm::Vector Ii = model_batch[j]; // read

                const rm::Vector Ni = model_normal_batch[j];

                // 2. project new dataset point on plane -> new model point
                const float signed_plane_dist = (Ii - Di).dot(Ni);


                if(fabs(signed_plane_dist) < max_corr_dist)
                {
                    // nearest point on model
                    const rm::Vector Mi = Di + Ni * signed_plane_dist;  


                    const rm::Vector d_mean_old = d_mean; // read
                    const rm::Vector m_mean_old = m_mean; // read

                    const float N_1 = static_cast<float>(n_corr);
                    const float N = static_cast<float>(n_corr + 1);
                    const float w1 = N_1/N;
                    const float w2 = 1.0/N;

                    

                    // update means
                    
                    // save old means for covariance
                    const rm::Vector d_mean_new = d_mean_old * w1 + Di * w2; 
                    const rm::Vector m_mean_new = m_mean_old * w1 + Mi * w2;

                    auto P1 = (Mi - m_mean_new).multT(Di - d_mean_new);
                    auto P2 = (m_mean_old - m_mean_new).multT(d_mean_old - d_mean_new);

                    // Write
                    C = C * w1 + P1 * w2 + P2 * w1;
                    d_mean = d_mean_new; // write
                    m_mean = m_mean_new; // write
                    n_corr = n_corr + 1;
                }

            }
        }

        Ncorr[i] = n_corr;
        dataset_center[i] = d_mean;
        model_center[i] = m_mean;
        Cs[i] = C;
    }
}

void means_covs_online_approx_batched(
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