#include "rmcl/clustering/clustering.h"

namespace rmcl 
{

void sort_clusters(std::vector<std::vector<size_t> >& clusters)
{
    std::sort(clusters.begin(), clusters.end(), 
        [](const std::vector<size_t>& a, const std::vector<size_t>& b) -> bool
        {
            return a.size() > b.size();
        }
    );

    // for(auto& cluster: clusters)
    // {
    //     std::sort(cluster.begin(), cluster.end());
    // }
}

std::vector<std::vector<size_t> > dbscan(
    KdTreePtr index, 
    float search_dist, 
    unsigned int min_pts_in_radius,
    unsigned int min_pts_per_cluster)
{
    size_t Npoints = index->m_size;

    std::vector<std::vector<size_t> > clusters;

    std::vector<bool> visited(Npoints);
    std::vector<std::pair<unsigned int, float> > matches;
    std::vector<std::pair<unsigned int, float> > sub_matches;

    auto data = index->dataset();

    for(size_t i = 0; i < Npoints; i++)
    {
        if (visited[i]) 
        {
            continue;
        }
        
        index->radiusSearch(
            reinterpret_cast<const float*>(&data->m_mem[i].x), 
            search_dist, matches, 
            nanoflann::SearchParams(32, 0.f, false));
        
        if (matches.size() < static_cast<size_t>(min_pts_in_radius)) 
        {
            continue;
        }

        visited[i] = true;

        std::vector<size_t> cluster = {i};

        while (matches.empty() == false)
        {
            auto nb_idx = matches.back().first;
            matches.pop_back();
            if (visited[nb_idx]) continue;
            visited[nb_idx] = true;

            index->radiusSearch(
                reinterpret_cast<const float*>(&data->m_mem[nb_idx].x),
                search_dist, sub_matches, 
                nanoflann::SearchParams(32, 0.f, false));

            if (sub_matches.size() >= static_cast<size_t>(min_pts_in_radius))
            {
                std::copy(sub_matches.begin(), sub_matches.end(), std::back_inserter(matches));
            }
            cluster.push_back(nb_idx);
        }

        if(cluster.size() >= min_pts_per_cluster)
        {
            clusters.emplace_back(std::move(cluster));
        }
    }

    return clusters;
}

} // namespace rmcl