#pragma once

#include "Clustering.hpp"


ClusteringData outofcore_cluster(std::vector<ClusteringData> incore_results,
    ClusteringMetricConfig metric_error, FloatingNumber max_error);
