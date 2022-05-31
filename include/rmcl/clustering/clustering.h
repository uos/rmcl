#ifndef RMCL_CLUSTERING_CLUSTERING_H
#define RMCL_CLUSTERING_CLUSTERING_H

#include <vector>
#include <algorithm>
#include <rmcl/spatial/KdTree.hpp>

namespace rmcl
{

void sort_clusters(std::vector<std::vector<size_t> >& clusters);

std::vector<std::vector<size_t> > dbscan(
    KdTreePtr index, 
    float search_dist, 
    unsigned int min_pts_in_radius,
    unsigned int min_pts_per_cluster);

} // namespace rmcl

#endif // RMCL_CLUSTERING_CLUSTERING_H