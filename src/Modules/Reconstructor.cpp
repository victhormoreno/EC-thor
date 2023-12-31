#include "Modules/Reconstructor.hpp"


/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */
bool Reconstructor::exists() {
    return this->ikd_Tree->size() > 0;
}

int Reconstructor::size() {
    return this->ikd_Tree->size(); 
}

PointType add_intensity(pcl::PointXYZ point){
    PointType p_prima;
    p_prima.x = point.x;
    p_prima.y = point.y;
    p_prima.z = point.z;
    p_prima.intensity = 0;
    return p_prima;
}

void Reconstructor::add(PointCloud &points, bool downsample) {
    if (not this->exists()){
         this->build_tree(points);
    }else {
        this->generate_increment_cloud(points);
        this->add_points(downsample); // error execució
    } 
    //std::cout << "Reconstructor::add: " << this->size() << std::endl;
}

void Reconstructor::build_tree(PointCloud &points) {
    this->generate_increment_cloud(points);
    this->ikd_Tree->Build(this->increment);
}

void Reconstructor::add_points(bool downsample) {
    this->ikd_Tree->Add_Points(this->increment, downsample);
}

void Reconstructor::delete_points() {
    this->ikd_Tree->Delete_Points(this->decrement);
}

void Reconstructor::generate_increment_cloud(PointCloud cloud){
    PointVector().swap(this->increment);
    for (PointType p : cloud->points) this->increment.push_back(p);        
}

void Reconstructor::generate_decrement_cloud(PointCloud cloud){
    PointVector().swap(this->decrement);
    for (PointType p : cloud->points) this->decrement.push_back(p);   
}

void Reconstructor::reconstruct(pcl::PointXYZI centroid, Cluster &cluster){
    PointVector k_points;
    std::vector<float> distance;
    this->ikd_Tree->Nearest_Search(centroid, this->k_nearest, k_points, distance, this->max_dist);
    for(PointType p : k_points){
        cluster.points_.push_back(p);
    }

}

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

void Reconstructor::init(const Params::Reconstructor &params) {

    this->delete_param = params.delete_param;
    this->balance_param = params.balance_param;
    this->box_length = params.box_length;

    this->k_nearest = 500;
    this->max_dist = 0.5;
    this->radius = 0.35;

    KD_TREE<PointType>::Ptr kdtree_ptr (new KD_TREE<PointType>(this->delete_param,this->balance_param,this->box_length));
    this->ikd_Tree =  kdtree_ptr;
}

void Reconstructor::updateIkdTree(const sensor_msgs::PointCloud2& cloud_msg){
    PointCloud cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(cloud_msg, *cloud);
    this->add(cloud, false);
}

void Reconstructor::reconstruct(std::vector<PointType> centroides, std::vector<Cluster> &clusters){
    
    for(PointType centroide : centroides){
        Cluster clust;
        this->reconstruct(centroide,clust);
        clusters.push_back(clust);
    }

}


