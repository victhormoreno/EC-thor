#include "modules/Reconstructor.hpp"


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
        this->add_points(downsample); // error execució si no hi ha downsampling
    } 
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
    PointVector points;
    std::vector<float> distance;
    if(mode) this->ikd_Tree->Nearest_Search(centroid, this->k_nearest, points, distance, this->max_dist);
    else this->ikd_Tree->Radius_Search(centroid, this->radius, points);

    for(PointType p : points){
        cluster.addPoint(p);
    }

}

/* -------------------------------------------------------------------------- */
/*                                   PUBLIC                                   */
/* -------------------------------------------------------------------------- */

void Reconstructor::init(const Params::Reconstructor &params) {

    this->delete_param = params.delete_param;
    this->balance_param = params.balance_param;
    this->box_length = params.box_length;

    this->k_nearest = params.k_nearest;
    this->max_dist =  params.max_dist;

    this->radius = params.radius;
    this->mode = params.mode;
    this->downsample = params.downsample;

    KD_TREE<PointType>::Ptr kdtree_ptr (new KD_TREE<PointType>(this->delete_param,this->balance_param,this->box_length));
    this->ikd_Tree =  kdtree_ptr;
}

void Reconstructor::updateIkdTree(const sensor_msgs::PointCloud2& cloud_msg){
    PointCloud cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(cloud_msg, *cloud);
    this->add(cloud, this->downsample);
}

void Reconstructor::reconstruct(std::vector<Cluster> &clusters){

    omp_set_num_threads(4); // to set the number of threads
    #pragma omp parallel for 
    for (int i = 0; i < clusters.size(); ++i) {
        pcl::PointXYZI centroide; 
        centroide.x = clusters[i].getCentroid(0);
        centroide.y = clusters[i].getCentroid(1);
        centroide.z = clusters[i].getCentroid(2);

        Cluster reconstruction;
        this->reconstruct(centroide,reconstruction);
        clusters[i] = reconstruction;
    }
}

void Reconstructor::attackZone(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, BoxPointType boxpoint){
    PointVector Searched_Points;
    this->ikd_Tree->Box_Search(boxpoint, Searched_Points);

    for(PointType p : Searched_Points){
        pcl::PointXYZI point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.intensity = 0;
        cloud->points.push_back(point);
    }

}


