
#include <floor_nav/SimTasksEnv.h>
#include <boost/algorithm/string.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace floor_nav;
using std::placeholders::_1;

SimTasksEnv::SimTasksEnv(std::shared_ptr<rclcpp::Node> n) : task_manager_lib::TaskEnvironment(n),
    joystick_topic("/teleop/twistCommand"), auto_topic("/mux/autoCommand"), 
    base_frame("bubbleRob"), reference_frame("world")
{
    manualControl = false; 
    joystick_topic = n->declare_parameter<std::string>("joystick_topic", joystick_topic);
    auto_topic = n->declare_parameter<std::string>("auto_topic", auto_topic);
    base_frame = n->declare_parameter<std::string>("base_frame", base_frame);
    reference_frame = n->declare_parameter<std::string>("reference_frame", reference_frame);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(n->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    rclcpp::QoS be = rclcpp::QoS(1).best_effort();
    pointCloudSub = n->create_subscription<sensor_msgs::msg::PointCloud2>("~/cloud3d",be,
            std::bind(&SimTasksEnv::pointCloudCallback,this,std::placeholders::_1));
    pointCloud2DSub = n->create_subscription<sensor_msgs::msg::PointCloud2>("~/cloud2d",be,
            std::bind(&SimTasksEnv::pointCloud2DCallback,this,std::placeholders::_1));
    laserscanSub = n->create_subscription<sensor_msgs::msg::LaserScan>("~/scan",be,
            std::bind(&SimTasksEnv::laserScanCallback,this,std::placeholders::_1));
    velPub = n->create_publisher<geometry_msgs::msg::Twist>(auto_topic,1);
}



geometry_msgs::msg::Pose2D SimTasksEnv::getPose2D() const {
    geometry_msgs::msg::Pose pose = getPose();
    geometry_msgs::msg::Pose2D pose2;
    pose2.theta = tf2::getYaw(pose.orientation);
    pose2.x = pose.position.x;
    pose2.y = pose.position.y;
    return pose2;
}

geometry_msgs::msg::Pose SimTasksEnv::getPose() const {
    return getPoseStamped().pose;
}

geometry_msgs::msg::PoseStamped SimTasksEnv::getPoseStamped() const {
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::TransformStamped transformStamped;
    try{
        transformStamped = tf_buffer->lookupTransform(reference_frame, base_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException & ex){
        RCLCPP_ERROR(this->getNode()->get_logger(),"%s",ex.what());
    }
    pose.pose.orientation = transformStamped.transform.rotation;
    pose.pose.position.x = transformStamped.transform.translation.x;
    pose.pose.position.y = transformStamped.transform.translation.y;
    pose.pose.position.z = transformStamped.transform.translation.z;
    pose.header = transformStamped.header;
    return pose;
}

void SimTasksEnv::publishVelocity(double linear, double angular) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    velPub->publish(cmd);
}

void SimTasksEnv::publishVelocity(double linear_x, double linear_y, double angular) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.linear.y = linear_y;
    cmd.angular.z = angular;
    velPub->publish(cmd);
}

void SimTasksEnv::publishVelocity(const geometry_msgs::msg::Twist & twist) {
    velPub->publish(twist);
}

void SimTasksEnv::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PCLPointCloud2 cloud2;
    // ROS2 Pointcloud2 to PCL Pointcloud2
    pcl_conversions::toPCL(*msg,cloud2);    
    // PCL Pointcloud2 to templated form
    pcl::fromPCLPointCloud2(cloud2,pointCloud);

    // RCLCPP_INFO(getNode()->get_logger(),"Received 3D point cloud with %d points",int(pointCloud.size()));
}

void SimTasksEnv::pointCloud2DCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PCLPointCloud2 cloud2;
    // ROS2 Pointcloud2 to PCL Pointcloud2
    pcl_conversions::toPCL(*msg,cloud2);    
    // PCL Pointcloud2 to templated form
    pcl::fromPCLPointCloud2(cloud2,pointCloud2D);
    // RCLCPP_INFO(getNode()->get_logger(),"Received 2D point cloud with %d points",int(pointCloud2D.size()));
}

void SimTasksEnv::laserScanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    pointCloud2D.resize(msg->ranges.size());
    for (size_t i=0;i<msg->ranges.size();i++) {
        pointCloud2D[i].x = msg->ranges[i]*cos(msg->angle_min + i*msg->angle_increment);
        pointCloud2D[i].y = msg->ranges[i]*sin(msg->angle_min + i*msg->angle_increment);
    }
    // RCLCPP_INFO(getNode()->get_logger(),"Received laser scan with %d points",int(pointCloud2D.size()));
}

