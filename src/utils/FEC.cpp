#include "utils/FEC.hpp"

FEC::FEC(const Params::Detector &params) {
    eps_ = params.hiperparams.eps;
    maxPts_ = params.hiperparams.maxPts;
    minPts_ = params.hiperparams.minPts;
}

FEC::~FEC() {}

void FEC::fastClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<pcl::PointIndices> &cluster_indices) {
                
    unsigned long i, j;

    pcl::KdTreeFLANN<pcl::PointXYZI> cloud_kdtreeflann;
    cloud_kdtreeflann.setInputCloud(cloud);

    int cloud_size = cloud->size();
    std::vector<int> marked_indices;
    marked_indices.resize(cloud_size);


    memset(marked_indices.data(), 0, sizeof(int) * cloud_size);
    std::vector<int> pointIdx;
    std::vector<float> pointquaredDistance;

    int tag_num = 1, temp_tag_num = -1;
    
    for (i = 0; i < cloud_size; i++)
    {
        // Clustering process
        if (marked_indices[i] == 0) // reset to initial value if this point has not been manipulated
        {
            pointIdx.clear();
            pointquaredDistance.clear();
            cloud_kdtreeflann.radiusSearch(cloud->points[i], this->eps_, pointIdx, pointquaredDistance);
            /**
            * All neighbors closest to a specified point with a query within a given radius
            * para.this->eps_ is the radius of the sphere that surrounds all neighbors
            * pointIdx is the resulting index of neighboring points
            * pointquaredDistance is the final square distance to adjacent points
            * pointIdx.size() is the maximum number of neighbors returned by limit
            */
            int min_tag_num = tag_num;
            for (j = 0; j < pointIdx.size(); j++)
            {
                /**
                 * find the minimum label value contained in the field points, and tag it to this cluster label.
                 */
                if ((marked_indices[pointIdx[j]] > 0) && (marked_indices[pointIdx[j]] < min_tag_num))
                {
                    min_tag_num = marked_indices[pointIdx[j]];
                }
            }
            for (j = 0; j < pointIdx.size(); j++)
            {
                temp_tag_num = marked_indices[pointIdx[j]];

                /*
                 * Each domain point, as well as all points in the same cluster, is uniformly assigned this label
                 */
                if (temp_tag_num > min_tag_num)
                {
                    for (int k = 0; k < cloud_size; k++)
                    {
                        if (marked_indices[k] == temp_tag_num)
                        {
                            marked_indices[k] = min_tag_num;
                        }
                    }
                }
                marked_indices[pointIdx[j]] = min_tag_num;
            }
            tag_num++;
        }
    }

    std::vector<PointIndex_NumberTag> indices_tags;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    indices_tags.resize(cloud_size);

    PointIndex_NumberTag temp_index_tag;


    for (i = 0; i < cloud_size; i++)
    {
        /**
        * Put each point index and the corresponding tag value into the indices_tags
        */
        temp_index_tag.nPointIndex = i;
        temp_index_tag.nNumberTag = marked_indices[i];

        indices_tags[i] = temp_index_tag;
    }

    sort(indices_tags.begin(), indices_tags.end(), NumberTag);

    unsigned long begin_index = 0;
    for (i = 0; i < indices_tags.size(); i++)
    {
        // Relabel each cluster
        if (indices_tags[i].nNumberTag != indices_tags[begin_index].nNumberTag)
        {
            if ((i - begin_index) >= this->minPts_)
            {
                unsigned long m = 0;
                inliers->indices.resize(i - begin_index);
                for (j = begin_index; j < i; j++)
                    inliers->indices[m++] = indices_tags[j].nPointIndex;
                cluster_indices.push_back(*inliers);
            }
            begin_index = i;
        }
    }

    if ((i - begin_index) >= this->minPts_)
    {
        for (j = begin_index; j < i; j++)
        {
            unsigned long m = 0;
            inliers->indices.resize(i - begin_index);
            for (j = begin_index; j < i; j++)
            {
                inliers->indices[m++] = indices_tags[j].nPointIndex;
            }
            cluster_indices.push_back(*inliers);
        }
    }
}