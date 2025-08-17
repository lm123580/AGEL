#include "DBSCANKdtreeCluster.h"

namespace AGEL
{
    DBSCANKdtreeCluster::DBSCANKdtreeCluster()
    {
        eps_ = 0.3;
        minPts_ = 15;
        min_pts_per_cluster_ = 40;
        max_pts_per_cluster_ = 4000;
    }

    void DBSCANKdtreeCluster::run(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_vector)
    {    
        if(cloud->size() == 0)
        {
            // ROS_WARN("CLOUD IS EMPTY, IN DBSCANKdtree.");
            return ;
        }

        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        std::vector<bool> is_noise(cloud->points.size(), false);
        std::vector<int> types(cloud->points.size(), UN_PROCESSED);

        pcl::search::KdTree<pcl::PointXYZ> tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree.setInputCloud(cloud);

        for (int i = 0; i < (int)cloud->points.size(); i++) {
            if (types[i] == PROCESSED) {
                continue;
            }

            int nn_size = tree.radiusSearch(i, eps_, nn_indices, nn_distances);

            if (nn_size < minPts_) {
                is_noise[i] = true;
                continue;
            }
            std::vector<int> seed_queue;
            seed_queue.push_back(i);
            types[i] = PROCESSED;
            
            for (int j = 0; j < nn_size; j++) {
                if (nn_indices[j] != i) {
                    seed_queue.push_back(nn_indices[j]);
                    types[nn_indices[j]] = PROCESSING;
                }
            } // for every point near the chosen core point.
            int sq_idx = 1;
            while (sq_idx < (int)seed_queue.size()) {
                int cloud_index = seed_queue[sq_idx];
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED) {
                    // seed_queue.push_back(cloud_index);
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue; // no need to check neighbors.
                }
                nn_size = tree.radiusSearch(cloud_index, eps_, nn_indices, nn_distances);
                if (nn_size >= minPts_) {
                    for (int j = 0; j < nn_size; j++) {
                        if (types[nn_indices[j]] == UN_PROCESSED) {
                            
                            seed_queue.push_back(nn_indices[j]);
                            types[nn_indices[j]] = PROCESSING;
                        }
                    }
                }
                
                types[cloud_index] = PROCESSED;
                sq_idx++;
            }

            if ((int)seed_queue.size() >= min_pts_per_cluster_ && (int)seed_queue.size() <= max_pts_per_cluster_) {
                // pcl::PointIndices r;
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                // r.indices.resize((int)seed_queue.size());
                for (int j = 0; j < (int)seed_queue.size(); ++j) {
                    tmp_cloud->push_back(cloud->points[seed_queue[j]]);
                    // r.indices[j] = seed_queue[j];
                }
                // These two lines should not be needed: (can anyone confirm?) -FF
                // std::sort(r.indices.begin(), r.indices.end());
                // r.indices.erase(std::unique(r.indices.begin(), r.indices.end()), r.indices.end());

                cloud_vector.push_back(tmp_cloud);
                // r.header = cloud->header;
                // cluster_indices.push_back(r);   // We could avoid a copy by working directly in the vector
            }
        }
    }    
}
