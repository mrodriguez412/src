#ifndef SIM_TASKS_ENV_H
#define SIM_TASKS_ENV_H

#include <rclcpp/rclcpp.hpp>
#include "task_manager_lib/TaskDefinition.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "cs7630_msgs/msg/roi_array.hpp"
// PRobably moved to the templated class
// #include "topic_tools/srv/MuxSelect.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


namespace floor_nav {
    class SimTasksEnv: public task_manager_lib::TaskEnvironment
    {
        protected:
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2DSub;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscanSub;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub;
            // rclcpp::Client<topic_tools::srv::MuxSelect>::SharedPtr muxClt;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer;

            void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) ;
            void pointCloud2DCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) ;
            void laserScanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg) ;

            bool manualControl;
            std::string joystick_topic;
            std::string auto_topic;
            std::string base_frame;
            std::string reference_frame;
            pcl::PointCloud<pcl::PointXYZ> pointCloud;
            pcl::PointCloud<pcl::PointXYZ> pointCloud2D;

        public:
            SimTasksEnv(std::shared_ptr<rclcpp::Node> node);
            ~SimTasksEnv() {};

            DECLARE_ENV_CHECKSUM;

            geometry_msgs::msg::Pose2D getPose2D() const ; 

            geometry_msgs::msg::Pose getPose() const ;

            geometry_msgs::msg::PoseStamped getPoseStamped() const  ;

            const pcl::PointCloud<pcl::PointXYZ>& getPointCloud() const {return pointCloud;}
            const pcl::PointCloud<pcl::PointXYZ>& getPointCloud2D() const {return pointCloud2D;}

            void publishVelocity(double linear, double angular) ;
            void publishVelocity(double linear_x, double linear_y, double angular) ;
            void publishVelocity(const geometry_msgs::msg::Twist & twist) ;

            // void setManualControl();
            // void setComputerControl();
            // bool getManualControl() const {return manualControl;}

            const std::string & getReferenceFrame() const {return reference_frame;}
            const std::string & getBaseFrame() const {return base_frame;}
            std::unique_ptr<tf2_ros::Buffer> & getTfBuffer() {return tf_buffer;}
    };

    typedef std::shared_ptr<SimTasksEnv> SimTasksEnvPtr;
    typedef std::shared_ptr<SimTasksEnv const> SimTasksEnvConstPtr;

};

#endif // SIM_TASKS_ENV_H
