#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>


class RadarObjectDetection
{
public:
    RadarObjectDetection()
    {

        pcl_sub_ = nh_.subscribe("radar_detection_pcl_dynamic", 10, &RadarObjectDetection::pointCloudCallback, this);
        // 감지된 객체의 위치 publish
        object_pub_ = nh_.advertise<geometry_msgs::PoseArray>("detected_objects", 10);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    ros::Publisher object_pub_;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // sensor_msgs::PointCloud2 메시지를 pcl::PointCloud로 변환합니다.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        // 다운샘플링을 위한 Voxel Grid 필터 적용
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.1f, 0.1f, 0.1f); // 10cm 단위로 다운샘플링
        vg.filter(*cloud_filtered);

        // Euclidean Cluster Extraction을 위한 KD 트리 설정
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.5); // 포인트 사이의 거리 허용치 (50cm)
        ec.setMinClusterSize(10);    // 최소 포인트 개수
        ec.setMaxClusterSize(250);   // 최대 포인트 개수
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        // 감지된 객체의 위치를 저장할 PoseArray 메시지
        geometry_msgs::PoseArray objects_poses;
        objects_poses.header.frame_id = input->header.frame_id;
        objects_poses.header.stamp = ros::Time::now();

        // 클러스터들을 순회하며 각각의 클러스터에 대해 중심점을 계산
        for (const auto& cluster : cluster_indices)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_filtered, cluster, centroid);

            // 중심점을 Pose로 변환하여 PoseArray에 추가
            geometry_msgs::Pose object_pose;
            object_pose.position.x = centroid[0];
            object_pose.position.y = centroid[1];
            object_pose.position.z = centroid[2];
            object_pose.orientation.w = 1.0; // 회전은 없음

            objects_poses.poses.push_back(object_pose);
        }

        // 감지된 객체들을 퍼블리시
        object_pub_.publish(objects_poses);

        ROS_INFO("Detected %lu objects", objects_poses.poses.size());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_object_detection");
    RadarObjectDetection radarObjectDetection;
    ros::spin();
    return 0;
}
