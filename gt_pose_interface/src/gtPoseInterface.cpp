#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// LiDAR 相对 trunk 的安装偏移（通过 ROS 参数配置，与 URDF laser_livox_joint 一致）
double lidarOffsetX = 0.20;
double lidarOffsetY = 0.0;
double lidarOffsetZ = 0.10;

double trunkX = 0, trunkY = 0, trunkZ = 0;
double trunkQx = 0, trunkQy = 0, trunkQz = 0, trunkQw = 1;
bool poseReceived = false;

ros::Publisher pubRegisteredScan;
ros::Publisher pubOdometry;
ros::Publisher pubCloudRegistered;
tf::TransformBroadcaster* tfBroadcaster;

// 收到 trunk GT pose → 广播 TF: map→odom，转发 /Odometry
// TF 链：map→odom(GT) → odom→base(leg odom, 由 Estimator.cpp 发布) → base→trunk→legs
void stateEstimationHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
    trunkX = odom->pose.pose.position.x;
    trunkY = odom->pose.pose.position.y;
    trunkZ = odom->pose.pose.position.z;
    trunkQx = odom->pose.pose.orientation.x;
    trunkQy = odom->pose.pose.orientation.y;
    trunkQz = odom->pose.pose.orientation.z;
    trunkQw = odom->pose.pose.orientation.w;
    poseReceived = true;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(trunkX, trunkY, trunkZ));
    transform.setRotation(tf::Quaternion(trunkQx, trunkQy, trunkQz, trunkQw));
    tfBroadcaster->sendTransform(
        tf::StampedTransform(transform, odom->header.stamp, "map", "odom")
    );

    pubOdometry.publish(odom);
}

// 收到原始点云（LiDAR 坐标系）→ 变换到 map 系 → 发布 /registered_scan, /cloud_registered
void scanHandler(const sensor_msgs::PointCloud::ConstPtr& cloudIn)
{
    if (!poseReceived) return;

    tf::Quaternion q(trunkQx, trunkQy, trunkQz, trunkQw);
    tf::Matrix3x3 R(q);

    pcl::PointCloud<pcl::PointXYZI> cloudOut;
    cloudOut.reserve(cloudIn->points.size());

    for (const auto& pt : cloudIn->points)
    {
        // Step1: laser_livox → trunk（只有平移，joint rpy=0）
        double px = pt.x + lidarOffsetX;
        double py = pt.y + lidarOffsetY;
        double pz = pt.z + lidarOffsetZ;

        // Step2: trunk → map（旋转 + 平移）
        pcl::PointXYZI p;
        p.x = R[0][0]*px + R[0][1]*py + R[0][2]*pz + trunkX;
        p.y = R[1][0]*px + R[1][1]*py + R[1][2]*pz + trunkY;
        p.z = R[2][0]*px + R[2][1]*py + R[2][2]*pz + trunkZ;
        p.intensity = 0;
        cloudOut.push_back(p);
    }

    sensor_msgs::PointCloud2 rosCloud;
    pcl::toROSMsg(cloudOut, rosCloud);
    rosCloud.header.stamp = cloudIn->header.stamp;
    rosCloud.header.frame_id = "map";
    pubRegisteredScan.publish(rosCloud);
    pubCloudRegistered.publish(rosCloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gtPoseInterface");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    nhPrivate.param("lidar_offset_x", lidarOffsetX, 0.20);
    nhPrivate.param("lidar_offset_y", lidarOffsetY, 0.0);
    nhPrivate.param("lidar_offset_z", lidarOffsetZ, 0.10);

    ROS_INFO("gtPoseInterface: LiDAR offset = (%.3f, %.3f, %.3f)",
             lidarOffsetX, lidarOffsetY, lidarOffsetZ);

    tfBroadcaster = new tf::TransformBroadcaster();

    ros::Subscriber subState = nh.subscribe<nav_msgs::Odometry>(
        "/state_estimation", 10, stateEstimationHandler);
    ros::Subscriber subScan = nh.subscribe<sensor_msgs::PointCloud>(
        "/scan", 5, scanHandler);

    pubRegisteredScan  = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 5);
    pubOdometry        = nh.advertise<nav_msgs::Odometry>("/Odometry", 5);
    pubCloudRegistered = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 5);

    ROS_INFO("gtPoseInterface: started. Waiting for /state_estimation and /scan...");
    ros::spin();

    delete tfBroadcaster;
    return 0;
}
