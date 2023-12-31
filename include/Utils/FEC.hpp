#pragma once

#include "Objects/Params.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <pcl/io/ply_io.h>
#include <ctime>
#include <omp.h>

/**
* Store index and label information for each point
*/
struct PointIndex_NumberTag
{
    float nPointIndex;
    float nNumberTag;
};

class FEC{
    private:
        /* ---------- Private Attributes ---------- */
        float eps_;
        int minPts_, maxPts_;
        static bool NumberTag(const PointIndex_NumberTag& p0, const PointIndex_NumberTag& p1){
            return p0.nNumberTag < p1.nNumberTag;
        }
    
    public:
        FEC() = default;
        FEC(const Params::ObjectDetector &params);
        ~FEC();
        std::vector<pcl::PointIndices> fastClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float epsilon ,int min_component_size, int max_n);
};



