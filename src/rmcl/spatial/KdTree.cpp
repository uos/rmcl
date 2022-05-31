#include "rmcl/spatial/KdTree.hpp"

using namespace rmagine;

namespace rmcl {

KdTree::KdTree(KdPointsPtr points)
:m_points(points)
,Super(3, *points, nanoflann::KDTreeSingleIndexAdaptorParams(10) )
{
    buildIndex();
}

Vector KdTree::nearest(
    const Vector& query_point,
    bool query_in_index) const
{
    if(query_in_index)
    {
        const size_t num_results = 2;
        nanoflann::KNNResultSet<float> resultSet(num_results);
        size_t ret_index[2];
        float out_dist_sqr[2];
        resultSet.init(ret_index, out_dist_sqr);
        findNeighbors(resultSet, reinterpret_cast<const float*>(&query_point), nanoflann::SearchParams(10));
        return m_points->m_mem.at(ret_index[1]);
    } else {
        const size_t num_results = 1;
        nanoflann::KNNResultSet<float> resultSet(num_results);
        size_t ret_index;
        float out_dist_sqr;
        resultSet.init(&ret_index, &out_dist_sqr);
        findNeighbors(resultSet, reinterpret_cast<const float*>(&query_point), nanoflann::SearchParams(10));
        return m_points->m_mem.at(ret_index);
    }
}

float KdTree::nearestDist(
    const Vector& query_point,
    bool query_in_index) const
{
    if(query_in_index)
    {
        const size_t num_results = 2;
        nanoflann::KNNResultSet<float> resultSet(num_results);
        size_t ret_index[2];
        float out_dist_sqr[2];
        resultSet.init(ret_index, out_dist_sqr);
        findNeighbors(resultSet, reinterpret_cast<const float*>(&query_point), nanoflann::SearchParams(10));
        return out_dist_sqr[1];
    } else {
        const size_t num_results = 1;
        nanoflann::KNNResultSet<float> resultSet(num_results);
        size_t ret_index;
        float out_dist_sqr;
        resultSet.init(&ret_index, &out_dist_sqr);
        findNeighbors(resultSet, reinterpret_cast<const float*>(&query_point), nanoflann::SearchParams(10));
        return out_dist_sqr;
    }
}

size_t KdTree::nearestId(
    const Vector& query_point,
    bool query_in_index) const
{
    if(query_in_index)
    {
        const size_t num_results = 2;
        nanoflann::KNNResultSet<float> resultSet(num_results);
        size_t ret_index[2];
        float out_dist_sqr[2];
        resultSet.init(ret_index, out_dist_sqr);
        findNeighbors(resultSet, reinterpret_cast<const float*>(&query_point), nanoflann::SearchParams(10));
        return ret_index[1];
    } else {
        const size_t num_results = 1;
        nanoflann::KNNResultSet<float> resultSet(num_results);
        size_t ret_index;
        float out_dist_sqr;
        resultSet.init(&ret_index, &out_dist_sqr);
        findNeighbors(resultSet, reinterpret_cast<const float*>(&query_point), nanoflann::SearchParams(10));
        return ret_index;
    }
}

} // namespace rmcl