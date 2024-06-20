#include "utils/Waggner.hpp"

/* -------------------------------------------------------------------------- */
/*                                   PRIVATE                                  */
/* -------------------------------------------------------------------------- */

Waggner::Waggner() {}
Waggner::~Waggner() {}


void Waggner::updateGrid(Cell cell, CellType type){
    
    if(cell.type == type) return;  
    if(cell.type == OBSTACLE) return;
    cell.type = type;    
}

void Waggner::createWallMask(void){

    for(int i=0; i < num_cells_; i++){
        for(int j=0; j < num_cells_; j++){
            float mean_z = this->grid_(i,j).descriptor.mean(2);
            float std_z = this->grid_(i,j).descriptor.stdev(2);
            if(mean_z + 2*std_z > 1.5)  this->grid_(i,j).type = WALL;
            else this->grid_(i,j).type = UNKNOWN;
        }
    }
}

void Waggner::dilatation(void){

    static std::vector<std::vector<bool>> img(grid_.rows(), std::vector<bool>(grid_.cols(), false));
    for(int i = 0; i < grid_.rows(); i++){
        for (int j = 0; j < grid_.cols(); j++){
            img[i][j] = grid_(i,j).type == WALL;
        }
    }

    // Dilataton
    #pragma omp parallel for num_threads(2)
    for (int i = 0; i < img.size(); ++i) {
        for (int j = 0; j < img.size(); ++j) {
            if(i == 0 || j == 0 || i == img.size()-1 || j == img.size()-1){
                walls_[i][j] == true;
                continue;  
            } 
            bool kernel = img[i][j] || img[i - 1][j] || img[i + 1][j] || img[i][j - 1] || img[i][j + 1];
            kernel = kernel || img[i - 1][j - 1] || img[i - 1][j + 1] || img[i + 1][j - 1] || img[i + 1][j + 1];
            if (kernel) walls_[i][j] = true;
            else walls_[i][j] = false;
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                                 PUBLIC                                   */
/* -------------------------------------------------------------------------- */

void Waggner::init(const Params::Processor &params){

    this->last_x_ = 0.0;
    this->last_y_ = 0.0;
    this->last_yaw_ = 0.0;

    // RANSAC Hyperparameters
    this->max_iter_ = params.waggner.max_iterations;
    this->dist_thresh_ = params.waggner.distance_threshold;
    this->max_slope_ = params.waggner.max_slope;
    this->max_error_ = params.waggner.max_error;
    this->range_ = params.waggner.max_range;
    this->min_z_ = params.waggner.min_z;
    this->max_z_ = params.waggner.max_z;

    // Initialize density map
    this->size_ = params.waggner.grid_size;
    this->cell_size_ = params.waggner.cell_size;
    this->num_cells_ = static_cast<int>(size_ / cell_size_);

    Cell cell;
    cell.type = UNKNOWN;
    cell.confidence = 0;
    cell.descriptor = Descriptor<MyPointType>();

    this->walls_ = std::vector<std::vector<bool>>(num_cells_, std::vector<bool>(num_cells_, false));
    this->surface_ = std::vector<std::vector<std::pair<float,float>>>(
                    num_cells_, std::vector<std::pair<float,float>>(
                    num_cells_, std::make_pair(0.0,1.0)));

    this->grid_ = Grid<Cell>(cell_size_, 0, 0, size_, size_, cell);   
}

void Waggner::updateRemover(const nav_msgs::Odometry::ConstPtr &state){

    this->last_x_ = state->pose.pose.position.x;
    this->last_y_ = state->pose.pose.position.y;

    auto qw = state->pose.pose.orientation.w;
    auto qx = state->pose.pose.orientation.x;
    auto qy = state->pose.pose.orientation.y;
    auto qz = state->pose.pose.orientation.z;

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    this->last_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    int cell_x = static_cast<int>((last_x_ + this->size_ / 2.0) / this->cell_size_);
    int cell_y = static_cast<int>((last_y_ + this->size_/ 2.0) / this->cell_size_);

    if (insideGrid(cell_x, cell_y)) {
        this->grid_(cell_x, cell_y).type = OBSTACLE;
    }
}

void Waggner::updateGrid(pcl::PointCloud<MyPointType>::Ptr &cloud){
    

    for(int i=0; i < cloud->size(); i++){

        // Compute cell index
        int u = grid_.cell_x(cloud->points[i].x);
        int v = grid_.cell_y(cloud->points[i].y);

        // Update cloud with cell index
        cloud->points[i].u = u;
        cloud->points[i].v = v;

        bool inside_grid = u != -1 && v != -1;
        if(inside_grid) grid_(u,v).descriptor.addPoint(cloud->points[i]);
    }

    this->createWallMask();
    this->dilatation();

}


void Waggner::removeWallsGrid(pcl::PointCloud<MyPointType>::Ptr &cloud,
                        pcl::PointCloud<MyPointType>::Ptr &walls, 
                        pcl::PointCloud<MyPointType>::Ptr &filtered){

    for(int i=0; i < cloud->size(); i++){
        MyPointType p = cloud->points[i];
        
        int u = p.u;
        int v = p.v;
        float range = p.range;

        bool inside_grid = u != -1 && v != -1;
        if (inside_grid) {
            if(walls_[u][v]) walls->points.push_back(p);
            else if (range < this->range_) filtered->points.push_back(p);
        } 
    }

}

void Waggner::removeGroundRansac(const pcl::PointCloud<MyPointType>::Ptr &cloud,
                        pcl::PointCloud<MyPointType>::Ptr &ground,
                        pcl::PointCloud<MyPointType>::Ptr &obstacles){

    pcl::PointCloud<pcl::PointXYZI>::Ptr preprocCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr postprocCloud(new pcl::PointCloud<pcl::PointXYZI>);
    auto size = cloud->points.size();
    if(size < 200) return;
    pcl::copyPointCloud(*cloud, *preprocCloud);

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(preprocCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(this->min_z_, this->max_z_);
    pass.filter(*postprocCloud);

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    if (postprocCloud->size() != 0) {
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(this->max_iter_);
        seg.setDistanceThreshold(this->dist_thresh_);
        seg.setInputCloud(postprocCloud);
        seg.setAxis(axis);
        seg.setEpsAngle(this->max_slope_);
        seg.segment(*inliers, *coefficients);

        auto inliers_size = inliers->indices.size();
        float relation = static_cast<float>(inliers_size) / size;

        if(relation > 0.99) return;
        if (inliers->indices.size() < 10) return;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pre_ground(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pre_obstacles(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(postprocCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*pre_ground);

        extract.setNegative(true);
        extract.filter(*pre_obstacles);   

        pcl::copyPointCloud(*pre_ground, *ground);
        pcl::copyPointCloud(*pre_obstacles, *obstacles);
    }
}


void lowpass(std::vector<std::vector<std::pair<float,float>>> &grid){

    int rows = grid.size();
    int cols = grid[0].size();

    static std::vector<std::vector<std::pair<float,float>>> original = std::vector<std::vector<std::pair<float,float>>>(rows, 
                                                                std::vector<std::pair<float,float>>(rows, std::make_pair(0.0,0.0)));
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            original[i][j] = grid[i][j];
        }
    }
    
    // #pragma omp parallel for num_threads(2)
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {

            float sum = 0.0f;
            int num = 0; 
            for (int k = -1; k <= 1; ++k) {
                for (int l = -1; l <= 1; ++l) {
                    int r = i + k;
                    int c = j + l;

                    if (r >= 0 && r < rows && c >= 0 && c < cols) {
                        if(std::isnan(original[r][c].second)) continue;
                        sum += original[r][c].second;
                        num++;
                    }
                }
            }

            grid[i][j].second = sum / num;
            
        }
    }
}


void Waggner::removeGroundSurface(const pcl::PointCloud<MyPointType>::Ptr &cloud,
                    pcl::PointCloud<MyPointType>::Ptr &ground,
                    pcl::PointCloud<MyPointType>::Ptr &obstacles) {

    // #pragma omp parallel for num_threads(4)
    for(int i=0; i < num_cells_; i++){
        for(int j=0; j < num_cells_; j++){
            
            if(grid_(i,j).descriptor.points() < 4) continue;
            if(walls_[i][j]) continue;

            float kx = this->grid_(i,j).descriptor.kurtosis(0);
            float ky = this->grid_(i,j).descriptor.kurtosis(1);
            float mean = this->grid_(i,j).descriptor.mean(2);
            float k = std::min(kx,ky);
            if (k < -0.75) this->surface_[i][j].second = mean;
        }
    }

    lowpass(this->surface_);

    for(int i=0; i < cloud->points.size(); i++){

        auto p = cloud->points[i];
        int u = p.u;
        int v = p.v;
        float z = p.z;
        if(insideGrid(u,v)){
            if(std::isnan(this->surface_[u][v].second)) ground->points.push_back(p);
            else if(z < this->surface_[u][v].second) {
                ground->points.push_back(p);
            } else if(z > 1.0) continue;
            else if(std::fabs(z - this->surface_[u][v].second) < 0.15) ground->points.push_back(p);
            else obstacles->points.push_back(p);
        }

    }



}


void Waggner::getSurface(pcl::PointCloud<MyPointType>::Ptr &cloud) {

    for (int i = 0; i < num_cells_ ; ++i) {
        for (int j = 0; j < num_cells_; ++j) {
                MyPointType p;
                p.x = (i - num_cells_ / 2) * cell_size_;
                p.y = -(j - num_cells_ / 2) * cell_size_;
                p.z = this->surface_[i][num_cells_-j-1].second;
                cloud->points.push_back(p);
        }
    }

}

void Waggner::getGrid(nav_msgs::OccupancyGrid &grid){

    grid.header.frame_id = "global";
    grid.header.stamp = ros::Time::now();
    grid.info.width = num_cells_;
    grid.info.height = num_cells_;
    grid.info.resolution = cell_size_;
    grid.info.origin.position.x = -size_ / 2;
    grid.info.origin.position.y = size_ / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    tf2::Quaternion q;
    tf2::fromMsg(grid.info.origin.orientation, q);
    tf2::Quaternion rot;
    rot.setRPY(0, 0, -M_PI / 2); 
    q *= rot;
    grid.info.origin.orientation = tf2::toMsg(q);

    grid.data.resize(num_cells_ * num_cells_);

    for(int i = 0; i < num_cells_; i++){
        for(int j = 0; j < num_cells_; j++){
            if(grid_(i,num_cells_-j-1).descriptor.points() < 3) {
                grid.data[i * num_cells_ + j] = 50;
                continue;
            }
            auto kx = this->grid_(i,num_cells_-j-1).descriptor.kurtosis(0);
            auto ky = this->grid_(i,num_cells_-j-1).descriptor.kurtosis(1);
            auto k = std::min(kx,ky);
            bool isGround = k < -0.75;
            grid.data[i * num_cells_ + j] = isGround ? 100 : 0;
        }
    }

}