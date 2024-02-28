#include "Lidar_pkg/pointCloud.h"
#include "Lidar_pkg/pointinfo.h"


#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <vector>

using namespace pcl;
using namespace std;

ros::Publisher pub_clustered;
ros::Publisher pub_custom;
// raw data => vector 


inline float MidPt(float a, float b){
    return (a + b) / 2;
}

inline float cal_dist(float x, float y){return sqrt(x*x+y*y);}

inline float inclination(float x1, float x2, float y1, float y2){return (y2 - y1)/(x2 - x1);}

class parameter {
public:
    float ROI_xMin, ROI_xMax;
    float ROI_yMin, ROI_yMax;
    float clustering_tolerance;
    int MinClusterSize, MaxClusterSize;

    parameter() {
        set_para();
    }

    void set_para() {
        // Parameter initialization with default values
        // ROI_xMin = -1.5;
        ROI_xMin = -1.2;
        ROI_xMax = 0;
        ROI_yMin = -1.0;
        ROI_yMax = 0.7;

        clustering_tolerance = 0.1; // 이웃 포인트 간의 최대 거리
        MinClusterSize = 3;
        MaxClusterSize = 1000;
    }
};

parameter P;

void LaserScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan, PointCloud<PointXYZI>& cloud) {
    cloud.clear(); // Ensure the cloud is empty before processing

    for (unsigned int i = 0; i < scan->ranges.size(); ++i) {
        float angle = scan->angle_min + i * scan->angle_increment;
        float x = scan->ranges[i] * cos(angle);
        float y = scan->ranges[i] * sin(angle);

        // Process points within the Region of Interest (ROI)
        if (x >= P.ROI_xMin && x <= P.ROI_xMax && y >= P.ROI_yMin && y <= P.ROI_yMax) {
            PointXYZI point;
            point.intensity = scan->intensities[i];
            point.x = -x;
            point.y = -y;
            point.z = 0;
            cloud.push_back(point);
        }
    }
}

//const PointCloud<PointXYZI>::Ptr& cloud
void Clustering(const PointCloud<PointXYZI>::Ptr& cloud, PointCloud<PointXYZI>& clusteredCloud, vector<pcl::PointXYZ>& sorted_OBJ,vector<pcl::PointXYZ>& length_OBJ) {
    // 터미널 no kdtree 삭제
    if (cloud->empty()) return;
    

    // Perform clustering on the point cloud
    search::KdTree<PointXYZI>::Ptr tree(new search::KdTree<PointXYZI>);
    tree->setInputCloud(cloud);

    vector<PointIndices> cluster_indices;
    EuclideanClusterExtraction<PointXYZI> ec;
    ec.setClusterTolerance(P.clustering_tolerance);
    ec.setMinClusterSize(P.MinClusterSize);
    ec.setMaxClusterSize(P.MaxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // 터미널 no kdtree 삭제
    if (cluster_indices.empty()) return;
    


    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        
        pair<float,float> x(9999,-9999); //first = min, second = max
        pair<float,float> y(9999,-9999);
        pair<float,float> z(9999,-9999);
        
        
    	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
                
            pcl::PointXYZI pt = cloud->points[*pit];
            if(pt.x < x.first) x.first = pt.x;
            if(pt.x > x.second) x.second = pt.x;
            if(pt.y < y.first) y.first = pt.y;
            if(pt.y > y.second) y.second = pt.y;
            if(pt.z < z.first) z.first = pt.z;
            if(pt.z > z.second) z.second = pt.z;
        
            clusteredCloud.push_back(pt);
        }
        pcl::PointXYZ* tmp = new pcl::PointXYZ(MidPt(x.first,x.second), MidPt(y.first,y.second), MidPt(z.first,z.second));
        pcl::PointXYZ* temp = new pcl::PointXYZ(y.first, y.second, z.first);

        sorted_OBJ.push_back(*tmp);
        length_OBJ.push_back(*temp);
    }
}

void processLiDARData(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Processing LiDAR data
    PointCloud<PointXYZI> rawCloud, clusteredCloud;
    PointCloud<PointXYZI>::Ptr cloudPtr(new PointCloud<PointXYZI>); 
    LaserScanToPointCloud(scan, rawCloud);
    *cloudPtr = rawCloud;


    vector<pcl::PointXYZ> sorted_OBJ;
    vector<pcl::PointXYZ> length_OBJ;
    Clustering(cloudPtr, clusteredCloud, sorted_OBJ, length_OBJ);

    // Publishing the clustered point cloud
    sensor_msgs::PointCloud2 output;
    toROSMsg(clusteredCloud, output);
    output.header.frame_id = scan->header.frame_id;
    pub_clustered.publish(output);

    Lidar_pkg::pointCloud customMsg;
    customMsg.size = sorted_OBJ.size();
    for (size_t i = 0; i < sorted_OBJ.size(); ++i) {
        Lidar_pkg::pointinfo info;
        info.x = sorted_OBJ[i].x;
        info.y = sorted_OBJ[i].y;
        info.y_Min = length_OBJ[i].x;
        info.y_Max = length_OBJ[i].y;
        info.idx = i;  // 클러스터의 인덱스
        info.dist = sqrt(sorted_OBJ[i].x * sorted_OBJ[i].x + sorted_OBJ[i].y * sorted_OBJ[i].y);  // 거리 계산
        customMsg.points.push_back(info);
    }
    pub_custom.publish(customMsg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "processing_2D_LidarData");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan> ("/lidar2D", 100, processLiDARData);
    pub_custom = nh.advertise<Lidar_pkg::pointCloud> ("/custom_cluster_info", 100);
    pub_clustered = nh.advertise<sensor_msgs::PointCloud2> ("/clustered_points", 100);
    ros::spin();
    return 0;
}