#include <ros/ros.h>
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
    ros::init(argc, argv, "point_cloud_perception");
    ros::NodeHandle nh;

    ros::param::get("~cloud_topic", cloud_topic);
    ROS_INFO("Cloud topic: %s", cloud_topic.c_str());
    std::cout << "cloud topic from param : " << cloud_topic;

    pub_marker_obj = nh.advertise<visualization_msgs::Marker>("markers/objects", 50);

    ros::Subscriber pc_sub = nh.subscribe(cloud_topic, 1, point_cloud_sub_cb);
    ros::spin();
}
