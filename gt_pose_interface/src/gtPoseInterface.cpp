#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
tf::TransformListener* tfListener;

// 收到 trunk GT pose（P3D 插件，map 系）
// 正确计算 map→odom：
//   TF 链：map→odom→base→trunk
//   P3D 给出 T_map_trunk，Estimator 给出 T_odom_base（base≈trunk，fixed joint）
//   需要：T_map_odom = T_map_trunk * inv(T_odom_base)
//   这样 map→trunk = T_map_odom * T_odom_base = T_map_trunk ✓（RobotModel 正确）
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

    tf::Transform T_map_trunk;
    T_map_trunk.setOrigin(tf::Vector3(trunkX, trunkY, trunkZ));
    T_map_trunk.setRotation(tf::Quaternion(trunkQx, trunkQy, trunkQz, trunkQw));

    tf::StampedTransform T_odom_base;
    tf::Transform T_map_odom;
    try {
        tfListener->lookupTransform("odom", "base", ros::Time(0), T_odom_base);
        // T_map_odom = T_map_trunk * inv(T_odom_base)
        T_map_odom = T_map_trunk * T_odom_base.inverse();
    } catch (tf::TransformException&) {
        // odom→base 还未就绪（启动初期），直接用 trunk GT pose
        T_map_odom = T_map_trunk;
    }

    tfBroadcaster->sendTransform(
        tf::StampedTransform(T_map_odom, odom->header.stamp, "map", "odom")
    );

    pubOdometry.publish(odom);
}

void scanHandler(const sensor_msgs::PointCloud::ConstPtr& cloudIn)
{
    if (!poseReceived) return;

    // Use full TF chain: map→odom→base→trunk→laser_livox
    // This correctly accounts for trunk roll/pitch on terrain.
    tf::StampedTransform T_map_lidar;
    try {
        tfListener->lookupTransform("map", "laser_livox", ros::Time(0), T_map_lidar);
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(5.0, "gtPoseInterface: TF map→laser_livox failed: %s", ex.what());
        return;
    }

    tf::Matrix3x3 R(T_map_lidar.getRotation());
    tf::Vector3    t = T_map_lidar.getOrigin();

    pcl::PointCloud<pcl::PointXYZI> cloudOut;
    cloudOut.reserve(cloudIn->points.size());

    const float bodyFilterRadius = 0.6f; // 过滤机器人自身点云半径(m)
    for (const auto& pt : cloudIn->points)
    {
        // 先在雷达坐标系下过滤自身点（距离雷达原点过近的点）
        float localDis = sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        if (localDis < bodyFilterRadius) continue;

        pcl::PointXYZI p;
        p.x = R[0][0]*pt.x + R[0][1]*pt.y + R[0][2]*pt.z + t.x();
        p.y = R[1][0]*pt.x + R[1][1]*pt.y + R[1][2]*pt.z + t.y();
        p.z = R[2][0]*pt.x + R[2][1]*pt.y + R[2][2]*pt.z + t.z();
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
    tfListener    = new tf::TransformListener();

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
    delete tfListener;
    return 0;
}
