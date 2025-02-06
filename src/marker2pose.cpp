#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <vector>
#include <cmath>

ros::Publisher laser_pub;

tf::StampedTransform m_transform;

void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (!msg->detections.empty())
    {
        double scan_angle_min = -M_PI / 2.0;
        double scan_angle_max = M_PI / 2.0;
        double scan_angle_increment = M_PI / 180.0;

        sensor_msgs::LaserScan laser_scan;

        laser_scan.header.frame_id = "base_link";
        laser_scan.range_min = 0.1;
        laser_scan.range_max = 30;

        laser_scan.angle_min = scan_angle_min;
        laser_scan.angle_max = scan_angle_max;
        laser_scan.angle_increment = scan_angle_increment;

        int num_readings = static_cast<int>((scan_angle_max - scan_angle_min) / scan_angle_increment) + 1;
        laser_scan.ranges.resize(num_readings, 0);
        laser_scan.intensities.resize(num_readings, 0);

        for (const auto &detection : msg->detections)
        {
            const geometry_msgs::Point &tag_position = detection.pose.pose.pose.position;

            tf::Vector3 point = {tag_position.x, tag_position.y, tag_position.z};
            point = m_transform * point;

            try
            {
                double dx = point.x();
                double dy = point.y();
                double distance = std::sqrt(dx * dx + dy * dy);

                double angle_to_tag = std::atan2(dy, dx);

                if (angle_to_tag >= scan_angle_min && angle_to_tag <= scan_angle_max)
                {
                    int index = static_cast<int>((angle_to_tag - scan_angle_min) / scan_angle_increment);
                    laser_scan.ranges[index] = static_cast<float>(distance);
                }

                laser_scan.header.stamp = detection.pose.header.stamp;
                laser_pub.publish(laser_scan);
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_pose_to_ekf");
    ros::NodeHandle nh;

    tf::TransformListener tf_listener;
    std::string base_link = "base_link";
    std::string camera_realsense_link_gazebo = "camera_realsense_link_gazebo";
    tf_listener.waitForTransform(base_link, camera_realsense_link_gazebo, ros::Time(0), ros::Duration(10));
    tf_listener.lookupTransform(base_link, camera_realsense_link_gazebo, ros::Time(0), m_transform);

    ros::Subscriber sub = nh.subscribe("/tag_detections", 10, callback);

    laser_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);

    // Spin do ROS
    ros::spin();

    return 0;
}
