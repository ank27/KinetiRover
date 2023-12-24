#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/recognition/cg/geometric_consistency.h>

std::string cloud_topic;
typedef pcl::PointXYZRGB PointT;
sensor_msgs::PointCloud2ConstPtr point_cloud_input;
std::string path = "/home/ankur/workspace/ros-robotics/cybot/src/arm/px100_perception/src/";
std::string voxel_file = "voxel_pc.pcd";
std::string raw_file = "raw_pc_data.pcd";
std::string seg_cloud_file = "seg_planner.pcd";
std::string radius_removal_cloud_file = "radius_remove.pcd";
std::string cluster_cloud_file = "cluster.pcd";
visualization_msgs::Marker marker_obj;
ros::Publisher pub_marker_obj;

void write_pcd_to_file(pcl::PCDWriter& cloud_writer, std::string file, pcl::PointCloud<PointT>& point_cloud) {
    cloud_writer.write(file, point_cloud);
}


typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::ReferenceFrame RFType;
std::string use_algo = "Hough";
std::string scene_file = "milk_cartoon_all.pcd";
std::string model_file = "milk.pcd";
double model_sampling_radius = 0.01f;
double scene_sampling_radius = 0.03f;
double ref_frame_radius =  0.015f;
double descriptor_radius = 0.02;

void object_detection()
{
    pcl::PointCloud<PointT>::Ptr model_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr model_keypoints_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr scene_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr scene_keypoints_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<NormalType>::Ptr model_normal_cloud(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<NormalType>::Ptr scene_normal_cloud(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptor(new pcl::PointCloud<DescriptorType>());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptor(new pcl::PointCloud<DescriptorType>());

    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    reader.read(path+scene_file, *scene_cloud);
    reader.read(path+model_file, *model_cloud);

    std::cout << "\nScene cloud before filtering width : " << scene_cloud->width << 
    ",height:" << scene_cloud->height << ", total points" << scene_cloud-> width * scene_cloud->height << std::endl;
    
    std::cout << "\nModel cloud before filtering width : " << model_cloud->width << 
    ",height:" << model_cloud->height << ", total points" << model_cloud-> width * model_cloud->height << std::endl;

    // compute normals for model and scene
    pcl::NormalEstimationOMP<PointT, NormalType> norm_estimation;
    norm_estimation.setKSearch(10);
    norm_estimation.setInputCloud(model_cloud);
    norm_estimation.compute(*model_normal_cloud);
    writer.write(path+"model_normal.pcd", *model_normal_cloud);

    norm_estimation.setInputCloud(scene_cloud);
    norm_estimation.compute(*scene_normal_cloud);
    writer.write(path+"scene_normal.pcd", *scene_normal_cloud);
    
    // downsample clouds
    pcl::UniformSampling<PointT> uniform_sampling;
    uniform_sampling.setInputCloud(model_cloud);
    uniform_sampling.setRadiusSearch(model_sampling_radius);
    uniform_sampling.filter(*model_keypoints_cloud);    
    std::cout << "Model total points: " << model_cloud->size() << ";  Selected Keypoints: " << model_keypoints_cloud->size () << std::endl;
    writer.write(path+"model_sampling.pcd", *model_keypoints_cloud);

    uniform_sampling.setInputCloud(scene_cloud);
    uniform_sampling.setRadiusSearch(scene_sampling_radius);
    uniform_sampling.filter(*scene_keypoints_cloud);
    std::cout << "Scene total points: " << scene_cloud->size() << ";  Selected Keypoints: " << scene_keypoints_cloud->size() << std::endl;
    writer.write(path+"scene_sampling.pcd", *scene_keypoints_cloud);

    // compute descriptor for keypoints
    pcl::SHOTEstimationOMP<PointT, NormalType, DescriptorType> descriptor_est;
    descriptor_est.setRadiusSearch(descriptor_radius);
    
    descriptor_est.setInputCloud(model_keypoints_cloud);
    descriptor_est.setInputNormals(model_normal_cloud);
    descriptor_est.setSearchSurface(model_cloud);
    descriptor_est.compute(*model_descriptor);
    std::cout << "Model Descriptor cloud size " << model_descriptor->size() << std::endl;

    descriptor_est.setInputCloud(scene_keypoints_cloud);
    descriptor_est.setInputNormals(scene_normal_cloud);
    descriptor_est.setSearchSurface(scene_cloud);
    descriptor_est.compute(*scene_descriptor);
    std::cout << "Scene Descriptor cloud size " << scene_descriptor->size() << std::endl;

    // model-scene correspondence with kd-tree
    pcl::CorrespondencesPtr model_scene_corr(new pcl::Correspondences());
    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud(model_descriptor);

    // look into scene descriptor to find correspondences with model descriptor
    for (size_t i = 0; i < scene_descriptor->size(); ++i)
    {
        std::vector<int> neighbor_indices(1);
        std::vector<float> neighbor_sqr_dist(1);
        if(!std::isfinite(scene_descriptor->at(i).descriptor[0])) //skip nans
        {
            continue;
        }
        int found_neighbors = match_search.nearestKSearch(scene_descriptor->at(i), 1, neighbor_indices, neighbor_sqr_dist);
        if(found_neighbors == 1 && neighbor_sqr_dist[0] < 0.25)
        {
            pcl::Correspondence corrs(neighbor_indices[0], static_cast<int>(i), neighbor_sqr_dist[0]);
            model_scene_corr->push_back(corrs);
        }
    }
    std::cout << "Correspondence found : " << model_scene_corr->size() << std::endl;

    // Clustering
    // std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rot_translation;    
    // std::vector<pcl::Correspondences> clustered_corrs;

    // // use Hough algo to find clusters
    // pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
    // pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

    // pcl::GeometricConsistencyGrouping<PointT, PointT> gc_clusterer;
    // gc_clusterer.setGCSize(0.01);
    // gc_clusterer.setGCThreshold(5);

    // gc_clusterer.setInputCloud(model_keypoints_cloud);
    // gc_clusterer.setSceneCloud (scene_keypoints_cloud);
    // gc_clusterer.setModelSceneCorrespondences(model_scene_corr);

    // //gc_clusterer.cluster (clustered_corrs);
    // gc_clusterer.recognize (rot_translation, clustered_corrs);
}


/**
 * this function test the pointcloud operations we can perform 
 * to segment, voxel or find the cluster in pointcloud.
*/
void perception_pipeline() 
{
    // create shared pointer for pcl::PCLPointCloud
    // pcl pointcloud2 wraps pointcloud2 object from the ros msg.
    pcl::PointCloud<PointT>::Ptr raw_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr pc_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);

    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    pcl::fromROSMsg(*point_cloud_input, *raw_cloud);
    // reader.read(path+raw_file, *raw_cloud);
    // reader.read(path+"table_scene_lms400.pcd", *raw_cloud);
    std::cerr << "\n PointCloud before filtering width : " << raw_cloud->width << 
    ",height:" << raw_cloud->height << ", total points" << raw_cloud-> width * raw_cloud->height << std::endl;
    writer.write(path+"raw_pc_data.pcd", *raw_cloud);

    pcl::CropBox<PointT> cropbox;
    cropbox.setInputCloud(raw_cloud);
    Eigen::Vector4f min_point = Eigen::Vector4f(-0.25, -0.25, -0.25, 1);
    Eigen::Vector4f max_point = Eigen::Vector4f(0.25, 0.25, 0.50, 1);
    cropbox.setMin(min_point);
    cropbox.setMax(max_point);
    cropbox.filter(*cropped_cloud);
    writer.write(path+"cropped_cloud.pcd", *cropped_cloud);

    // voxel grid to desmaple the pc
    pcl::VoxelGrid<PointT> desampled_cloud;
    desampled_cloud.setInputCloud(cropped_cloud);
    desampled_cloud.setLeafSize(0.005f, 0.005f, 0.005f);
    desampled_cloud.filter(*pc_filtered);

    std::cerr << "\nPointCloud after filtering width : " << pc_filtered->width << 
    ",height:" << pc_filtered->height << ", total points" << pc_filtered-> width * pc_filtered->height << std::endl;
    writer.write(path+voxel_file, *pc_filtered);

    // segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.0045);
    seg.setInputCloud(pc_filtered);
    seg.segment(*inliers, *coefficients);
    
    // Extract the inliers          
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(pc_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_p);
    std::cerr << "PointCloud after the planar component removed : " << 
    cloud_p->width * cloud_p->height << " data points." << std::endl;
    std::stringstream ss;
    ss << path+"seg_planner" << ".pcd";
    writer.write<PointT>(ss.str(), *cloud_p, false);

    if(cloud_p->size() == 0) return;
    std::cout << " after segmentation cloud size : " << cloud_p->size() << std::endl;
    
    // Create kd-tree for object extraction
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_p);

    std::vector<pcl::PointIndices> cluster_ind;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_p);
    ec.extract(cluster_ind);

    std::cout << "extracted cluster size : " << cluster_ind.size() << std::endl;
    int j=0;
    for(const auto& cluster : cluster_ind)
    {
        // for specific cluster we will write pointcloud data in separate pcd file
        // so we will extract pointCloud for each cluster in cloud_cluster
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(const auto& idx : cluster.indices)
        {
            // push the point from the specific cluster to cluster object 
            // so we can write it in pcd file
            cloud_cluster->push_back((*cloud_p)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the cluster size : " << cloud_cluster->size() << std::endl;
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << j;
        writer.write(path+"cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); 

        // find centroid of each cluster
        PointT centroid, min_point;
        pcl::computeCentroid(*cloud_cluster, centroid);
        //find the nearest point based on depth of point
        min_point = cloud_cluster->points[0];
        for (size_t i {1}; i < cloud_cluster->size(); i++)
        {
            if(cloud_cluster->points[i].z < min_point.z){
                min_point = cloud_cluster->points[i];
            }
        }
        std::cout << "Centroid of cluster " << j << " is : " << centroid << std::endl;
        std::cout << "Min point of cluster" << j << " is : " << min_point << std::endl;
        std::cout << "Input frame id : " << point_cloud_input->header.frame_id << std::endl;
        // add marker object at centroid in rviz
        marker_obj.id = j;
        marker_obj.header.frame_id = point_cloud_input->header.frame_id;
        marker_obj.type = visualization_msgs::Marker::SPHERE;
        marker_obj.pose.position.x = centroid.x;
        marker_obj.pose.position.y = centroid.y;
        marker_obj.pose.position.z = centroid.z;
        marker_obj.pose.orientation.x = 0.0;
        marker_obj.pose.orientation.y = 0.0;
        marker_obj.pose.orientation.z = 0.0;
        marker_obj.pose.orientation.w = 1.0;
        marker_obj.color.r = 1.0;
        marker_obj.color.g = 0.0;
        marker_obj.color.b = 0.0;
        marker_obj.color.a = 1.0;
        marker_obj.scale.x = 0.01;
        marker_obj.scale.y = 0.01;
        marker_obj.scale.z = 0.01;
        marker_obj.header.stamp = ros::Time::now();
        pub_marker_obj.publish(marker_obj);
        j++;
    };
}

// manual segementation using manual point cloud entries
// to understand how pcl segmentation works using Sample Consensus
void planner_segmentation()
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    cloud->width = 15;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for(auto& point : *cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1.0;
        std::cout << "Points = " << point  << "\n";
    }

    // few outliers
    (*cloud)[0].z = 2.0;
    (*cloud)[3].z = -2.0;
    (*cloud)[5].z = 4.0;
    
    //planner segmentation 
    // Model ax+by+cz+d =0 ; where a,b,c,d are ModelCoefficients
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    //check size of inliers to get the plane
    if(inliers->indices.size() == 0) 
    {
        std::cerr << "Could not estimate the plane model" << std::endl;
    }

    // print model coefficient
    std::cout << "Model coefficients: " << coefficients->values[0] << " " 

                                        << coefficients->values[1] << " "

                                        << coefficients->values[2] << " " 

                                        << coefficients->values[3] << std::endl;

    // model inliers
    std::cout << "Inliers size " << inliers->indices.size() << std::endl;
    for(const auto& inlier : inliers->indices) {
        std::cout << inlier << cloud->points[inlier].x << " " 
                            << cloud->points[inlier].y << " "
                            << cloud->points[inlier].z << " ";
    }
    std::cout << std::endl;
};


void point_cloud_sub_cb(const sensor_msgs::PointCloud2ConstPtr& pc) {
    point_cloud_input = pc;
    perception_pipeline();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detection_perception");
    ros::NodeHandle nh;

    ros::param::get("~cloud_topic", cloud_topic);
    ROS_INFO("Cloud topic: %s", cloud_topic.c_str());
    std::cout << "cloud topic from param : " << cloud_topic;

    // pub_marker_obj = nh.advertise<visualization_msgs::Marker>("markers/objects", 50);

    // ros::Subscriber pc_sub = nh.subscribe(cloud_topic, 1, point_cloud_sub_cb);
    while (ros::ok())
    {
        object_detection();
    }
    ros::spin();
}
