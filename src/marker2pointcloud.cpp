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

ros::Publisher pose_pub;

ros::Publisher laser_pub;

struct Point3D
{
    double x, y, z;
};
std::vector<Point3D> generateSpherePoints(int numPoints, Point3D center, double radius)
{
    std::vector<Point3D> points;
    double phi = M_PI * (3.0 - std::sqrt(5.0)); // ângulo de ouro

    for (int i = 0; i < numPoints; ++i)
    {
        double y = 1 - (i / (double)(numPoints - 1)) * 2; // distribui os pontos de -1 a 1
        double r = std::sqrt(1 - y * y) * radius;         // raio do círculo para esse nível
        double theta = phi * i;                           // ângulo baseado no ângulo de ouro

        double x = center.x + std::cos(theta) * r;
        double z = center.z + std::sin(theta) * r;
        double y_coord = center.y + y * radius;

        points.push_back({x, y_coord, z});
    }
    return points;
}

void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (!msg->detections.empty())
    {
        const apriltag_ros::AprilTagDetection &detection = msg->detections[0];

        geometry_msgs::Pose marker_pose = detection.pose.pose.pose;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = marker_pose;

        double scan_angle_min = -M_PI / 2.0;
        double scan_angle_max = M_PI / 2.0;
        double scan_angle_increment = M_PI / 180.0;

        sensor_msgs::LaserScan laser_scan;

        laser_scan.header.stamp = ros::Time::now();
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

            // tf::StampedTransform transform;
            try
            {

                for (auto id : detection.id)
                {
                    tf::StampedTransform transform;
                    tf::TransformListener tf_listener;
                    std::string tag_id = "tag_" + std::to_string(id);
                    ROS_WARN_STREAM("id: " << id);
                    ROS_WARN_STREAM("tag_id: " << tag_id);

                    try
                    {
                        std::string base_link = "base_link";
                        std::string camera_realsense_link_gazebo = "camera_realsense_link_gazebo";
                        tf_listener.waitForTransform(base_link, tag_id, ros::Time(0), ros::Duration(0.2));
                        tf_listener.lookupTransform(base_link, tag_id, ros::Time(0), transform);
                        ROS_WARN_STREAM("id: " << id);
                        ROS_WARN_STREAM("x: " << transform.getOrigin().getX());
                        ROS_WARN_STREAM("y: " << transform.getOrigin().getY());
                        ROS_WARN_STREAM("z: " << transform.getOrigin().getZ());

                        int numPoints = 1;                                                                                          // número de pontos
                        Point3D center = {transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ()}; // centro da esfera
                        double radius = 0.0;
                        auto points = generateSpherePoints(numPoints, center, radius);

                        for (const auto &p : points)
                        {
                            tf::Vector3 tag_position_transformed = tf::Vector3(p.x, p.y, p.z);

                            double dx = tag_position_transformed.x();
                            double dy = tag_position_transformed.y();
                            double distance = std::sqrt(dx * dx + dy * dy);

                            ROS_WARN_STREAM("distance: " << distance);

                            double angle_to_tag = std::atan2(dy, dx);

                            ROS_WARN_STREAM("angle_to_tag: " << angle_to_tag);

                            // Verificar se o ângulo está dentro do intervalo do LaserScan
                            if (angle_to_tag >= scan_angle_min && angle_to_tag <= scan_angle_max)
                            {

                                // Converter o ângulo para o índice correspondente no vetor ranges
                                int index = static_cast<int>((angle_to_tag - scan_angle_min) / scan_angle_increment);

                                ROS_WARN_STREAM("dentro do if- index: " << index);

                                // Atualizar a distância mínima no índice correspondente
                                laser_scan.ranges[index] = static_cast<float>(distance);
                            }
                        }
                    }
                    catch (const std::exception &e)
                    {
                        std::cerr << e.what() << '\n';
                    }
                }

                ROS_WARN_STREAM("detection.pose.header.frame_id: " << detection.pose.header.frame_id);

                // tf::Vector3 tag_position_transformed = tf::Vector3(tag_position.z, -tag_position.x, -tag_position.y);

                // double dx = tag_position_transformed.x();
                // double dy = tag_position_transformed.y();
                // double distance = std::sqrt(dx * dx + dy * dy);
                // ROS_WARN_STREAM("distance: " << distance);

                // double angle_to_tag = std::atan2(dy, dx);

                // ROS_WARN_STREAM("angle_to_tag: " << angle_to_tag);

                // if (angle_to_tag >= scan_angle_min && angle_to_tag <= scan_angle_max)
                // {

                //     int index = static_cast<int>((angle_to_tag - scan_angle_min) / scan_angle_increment);

                //     ROS_WARN_STREAM("dentro do if- index: " << index);

                //     laser_scan.ranges[index] = static_cast<float>(distance);
                // }
                laser_scan.header.stamp = ros::Time::now();
                laser_pub.publish(laser_scan);
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN("tudo falhou: %s", ex.what());
                continue;
            }
        }

        // geometry_msgs::PoseWithCovarianceStamped pose_cov_stamped;
        // pose_cov_stamped.header = msg->header;
        // pose_cov_stamped.pose.pose = marker_pose;
        // pose_cov_stamped.pose.covariance = detection.pose.pose.covariance;

        // static tf2_ros::Buffer tf_buffer;
        // static tf2_ros::TransformListener tf_listener(tf_buffer);

        try
        {
            //     geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
            //         "base_link",
            //         pose_stamped.header.frame_id,
            //         ros::Time(0),
            //         ros::Duration(1.0));

            //     geometry_msgs::PoseWithCovarianceStamped pose_transformed;
            //     tf2::doTransform(pose_cov_stamped, pose_transformed, transform);

            // ROS_INFO_STREAM("Pose do marcador transformada: " << pose_transformed.pose);
            // pose_pub.publish(pose_cov_stamped);
        }
        catch (const tf2::TransformException &e)
        {
            ROS_WARN_STREAM("Erro ao tentar obter a transformação: " << e.what());
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_pose_to_ekf");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/tag_detections", 10, callback);

    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/marker_pose_for_ekf", 10);

    laser_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);

    ros::spin();

    return 0;
}
