#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
// #include <message_filters/subscriber.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/synchronizer.h>
#include <Eigen/Dense>
#include <tf/tf.h>

class SignLocalization
{
private:
    ros::NodeHandle nh;
    ros::Subscriber visMarker_sub;
    ros::Subscriber odom_sub;
    Eigen::Vector3d object;
    Eigen::Vector3d object_to_map;
    Eigen::Matrix3d transformation_matrix;
    double yaw;
    double pose_xy;
    double pose_x;
    double pose_y;
    double angle;

public:
    SignLocalization();
    void visMarkerCallBack(const visualization_msgs::MarkerArray::ConstPtr& marker_array_msg);
    void odomCallBack(const nav_msgs::Odometry::ConstPtr& odom_msg);
    double quaterion2euler(const nav_msgs::Odometry::ConstPtr& odom_msg);
    Eigen::Vector3d transformPoint(double degree, Eigen::Vector3d sign_position, double position_xy, double position_x, double position_y);
    // double convert(double radians);
};   

void SignLocalization::odomCallBack(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    yaw = quaterion2euler(odom_msg);
    // angle = convert(yaw);  // I DON'T CONVERT RADIAN TO DEGREE HERE, BECAUSE COS AND SIN FUNCTION USE RADIAN
    pose_x = odom_msg->pose.pose.position.x;
    pose_y = odom_msg->pose.pose.position.y;
    pose_xy = sqrt((odom_msg->pose.pose.position.x * odom_msg->pose.pose.position.x) + (odom_msg->pose.pose.position.y * odom_msg->pose.pose.position.y)); 
}

void SignLocalization::visMarkerCallBack(const visualization_msgs::MarkerArray::ConstPtr& marker_array_msg)
{
    if (marker_array_msg->markers.size() == 0)
    {
        std::cout << "No object detected!" << std::endl;
        return;
    }
    else
    {
        object << marker_array_msg->markers[0].pose.position.x, 
            marker_array_msg->markers[0].pose.position.y,
                1.0;

        object_to_map = transformPoint(yaw, object, pose_xy, pose_x, pose_y);  // transform object position to map frame
        std::cout << "Object x|y location corresponding to map frame: " << object_to_map[0] << " | " << object_to_map[1] << std::endl;
    }
}

SignLocalization::SignLocalization()
{
    std::cout << "SignLocalization Node Started!" << std::endl;

    visMarker_sub = nh.subscribe<visualization_msgs::MarkerArray> ("/detection_marker", 1, &SignLocalization::visMarkerCallBack, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry> ("/odom", 1, &SignLocalization::odomCallBack, this);
    object = Eigen::Vector3d::Zero();
    transformation_matrix = Eigen::Matrix3d::Zero();
    yaw = 0.0;
    pose_xy= 0.0;
    pose_x= 0.0;
    pose_y= 0.0  ;
    angle = 0.0;
}

double SignLocalization::quaterion2euler(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, 
        odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

Eigen::Vector3d SignLocalization::transformPoint(double degree, Eigen::Vector3d sign_position, double position_xy, double position_x, double position_y)
{
    std::cout << "----------------------------------------------------" << std::endl;
    std::cout << "Car Angle(radian): " << degree << std::endl;   // angle of the car relative to the map frame
    std::cout << "Object x | y corresponding to car: " << sign_position[0] << " | " << sign_position[1] << std::endl;         // object position relative to the car frame

    transformation_matrix << cos(degree), -sin(degree), position_x, 
                             sin(degree), cos(degree) , position_y,
                             0.0     , 0.0      , 1.0;

    std::cout << transformation_matrix << std::endl;

    object_to_map = transformation_matrix * sign_position;  // 3x3 * 3x1 = 3x1

    return object_to_map;
}

// double SignLocalization::convert(double radians)
// {
//     double pi = 3.14159;
//     return(radians * (180 / pi));
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "SignLocalizationNode");
    SignLocalization node;
    ros::spin();

}