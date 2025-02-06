#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

ros::Publisher pose_pub;

void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (!msg->detections.empty())
    {
        const apriltag_ros::AprilTagDetection &detection = msg->detections[0];

        geometry_msgs::Pose marker_pose = detection.pose.pose.pose;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = marker_pose;

        geometry_msgs::PoseWithCovarianceStamped pose_cov_stamped;
        pose_cov_stamped.header = msg->header;
        pose_cov_stamped.pose.pose = marker_pose;
        pose_cov_stamped.pose.covariance = detection.pose.pose.covariance;

        static tf2_ros::Buffer tf_buffer;
        static tf2_ros::TransformListener tf_listener(tf_buffer);

        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
                "base_link",
                pose_stamped.header.frame_id,
                ros::Time(0),
                ros::Duration(1.0));

            geometry_msgs::PoseWithCovarianceStamped pose_transformed;
            tf2::doTransform(pose_cov_stamped, pose_transformed, transform);

            pose_pub.publish(pose_cov_stamped);
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

    ros::spin();

    return 0;
}
