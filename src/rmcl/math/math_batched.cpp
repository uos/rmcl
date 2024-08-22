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
    rm::MemoryView<unsigned int, rm::RAM>& Ncorr, // N
    int scene_id,
    int object_id,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& scene_mask, // NxM
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& object_mask // NxM
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
        const rm::MemoryView<unsigned int> scene_mask_batch = scene_mask(i * batchSize, (i+1) * batchSize);
        const rm::MemoryView<unsigned int> object_mask_batch = object_mask(i * batchSize, (i+1) * batchSize);



        rm::Vector d_mean = {0.0f, 0.0f, 0.0f};
        rm::Vector m_mean = {0.0f, 0.0f, 0.0f};
        rm::Matrix3x3 C = rm::Matrix3x3::Zeros();
        unsigned int n_corr = 0;

        for(size_t j=0; j<batchSize; j++)
        {
            // figure out if distance is too high

            if(dataset_mask[j] > 0 && model_mask_batch[j] > 0
               && ((scene_id < 0 && object_id < 0) || (scene_mask_batch[j] == static_cast<unsigned int>(scene_id) && object_mask_batch[j] == static_cast<unsigned int>(object_id)))
            )
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

void incremental_covariance_object_wise(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& dataset_points, // from, M
    const rmagine::MemoryView<unsigned int, rmagine::RAM>&    dataset_mask, // from, M
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_points, // to, M
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_normals, // to, M
    const rmagine::MemoryView<unsigned int, rmagine::RAM>&    model_object_ids,  // to, M
    const rmagine::MemoryView<unsigned int, rmagine::RAM>&    model_mask, // to, M
    const float max_corr_dist,
    rmagine::MemoryView<rmagine::Vector>& dataset_centers,      // per object id
    rmagine::MemoryView<rmagine::Vector>& model_centers,        // per object
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,  // per object
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr     // per object
)
{
    // in contrast to other functions in this file this function is especially 
    // designed to correct a set of objects in a frame at once
    // 
    // we fill the partition variables (mu_d, mu_m, C, N) by object id:
    // - dataset_centers, model_centers, Cs, Ncorr
    // therefore, it is required for the memory objects to have a size of the largest object_id+1
    // 
    // therefore, it is most efficient for the object ids to have
    // an increasing unique order, e.g model_object_ids could look like this:
    // [-,-,-,0,2,-,-,-,1,-,-,-,-,3]
    // but not like this
    // [-,-,-,1000,-,1]
    // 
    // TODO: check if we can meet these constraints. Otherwise I will change some things

    unsigned int batch_size = dataset_points.size();
    unsigned int n_objects = dataset_centers.size();

    // initialize all partitions
    for(size_t i=0; i<n_objects; i++)
    {
      dataset_centers[i] = {0.0f, 0.0f, 0.0f};
      model_centers[i] = {0.0f, 0.0f, 0.0f};
      Cs[i] = rm::Matrix3x3::Zeros();
      Ncorr[i] = 0;
    }

    for(size_t i=0; i<batch_size; i++)
    {
        // figure out if distance is too high
        if(dataset_mask[i] > 0 && model_mask[i] > 0) // valid
        {
            // 1. read a few things we need to make a iterative
            const rm::Vector Di   = dataset_points[i];
            const rm::Vector Ii   = model_points[i];
            const rm::Vector Ni   = model_normals[i];
            const unsigned int Oi = model_object_ids[i];

            // 2. project new dataset point on plane -> new model point
            const float signed_plane_dist = (Ii - Di).dot(Ni);

            if(fabs(signed_plane_dist) < max_corr_dist)
            {
                // nearest point on model
                const rm::Vector Mi = Di + Ni * signed_plane_dist;

                // read old partition (mu_d, mu_m, C_dm, N)
                const rm::Vector d_mean_old = dataset_centers[Oi];
                const rm::Vector m_mean_old = model_centers[Oi];
                const rm::Matrix3x3 C = Cs[i];
                const unsigned int N_old = Ncorr[Oi];

                const float N_1 = static_cast<float>(N_old);
                const float N = static_cast<float>(N_old + 1);
                const float w1 = N_1/N;
                const float w2 = 1.0/N;
                // update means
                
                // save old means for covariance
                const rm::Vector d_mean_new = d_mean_old * w1 + Di * w2; 
                const rm::Vector m_mean_new = m_mean_old * w1 + Mi * w2;

                auto P1 = (Mi - m_mean_new).multT(Di - d_mean_new);
                auto P2 = (m_mean_old - m_mean_new).multT(d_mean_old - d_mean_new);

                // Write
                dataset_centers[Oi] = d_mean_new; // write
                model_centers[Oi] = m_mean_new; // write
                Cs[Oi] = C * w1 + P1 * w2 + P2 * w1;
                Ncorr[Oi] = N_old + 1;
            }
        }
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
