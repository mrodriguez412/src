#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
// #include <cs7630_msgs/msg/plane_description.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>

// test

class FloorPlaneRegression: public rclcpp::Node {
    protected:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        // rclcpp::Client<topic_tools::srv::MuxSelect>::SharedPtr muxClt;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;


        std::string base_frame_;
        double max_range_;


    protected: // ROS Callbacks

        void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> pc_sensor, pc_baseframe;
            pcl::PCLPointCloud2 cloud2;
            pcl_conversions::toPCL(*msg,cloud2);    
            pcl::fromPCLPointCloud2(cloud2,pc_sensor);

            if (msg->header.frame_id != base_frame_) {
                geometry_msgs::msg::TransformStamped transformStamped;
                try {
                    std::string errStr;
                    // This converts target in the grid frame.
                    if (!tf_buffer->canTransform(base_frame_, msg->header.frame_id, msg->header.stamp,
                                rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                        RCLCPP_ERROR(this->get_logger(),"Cannot transform target: %s",errStr.c_str());
                        return;
                    }
                    transformStamped = tf_buffer->lookupTransform(base_frame_, msg->header.frame_id, msg->header.stamp);
                    sensor_msgs::msg::PointCloud2 pc;
                    tf2::doTransform(*msg,pc,transformStamped);

                    // ROS2 Pointcloud2 to PCL Pointcloud2
                    pcl_conversions::toPCL(pc,cloud2);    
                } catch (const tf2::TransformException & ex){
                    RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
                }
            } else {
                // ROS2 Pointcloud2 to PCL Pointcloud2
                pcl_conversions::toPCL(*msg,cloud2);    
            }
            // PCL Pointcloud2 to templated form
            pcl::fromPCLPointCloud2(cloud2,pc_baseframe);

            //
            unsigned int n = pc_sensor.size();
            std::vector<size_t> pidx;
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
                float x = pc_sensor[i].x;
                float y = pc_sensor[i].y;
                float d = hypot(x,y);
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                x = pc_baseframe[i].x;
                y = pc_baseframe[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                pidx.push_back(i);
            }
            
            //
            //
            //
            // TODO START
            // 
            // Linear regression: z = a*x + b*y + c
            // Update the code below to use Eigen to find the parameters of the
            // linear regression above. 
            //
            // n is the number of useful point in the point cloud
            n = pidx.size();
            // Eigen is a matrix library. The line below create a 3x3 matrix A,
            // and a 3x1 vector B
            Eigen::MatrixXf A(3,3);
            Eigen::MatrixXf B(n,1);
            for (unsigned int i=0;i<n;i++) {
                // Assign x,y,z to the coordinates of the point we are
                // considering.
                double x = pc_baseframe[pidx[i]].x;
                double y = pc_baseframe[pidx[i]].y;
                double z = pc_baseframe[pidx[i]].z;

                // Example of initialisation of the matrices (wrong)
                A(0,0) = x;
                A(1,1) = y;
                A(2,2) = z;

                B(0,0) = x;
                B(1,0) = y;
                B(2,0) = z;
            }
            // Eigen operation on matrices are very natural:
            Eigen::MatrixXf X = A.transpose() * B;
            // Details on linear solver can be found on 
            // http://eigen.tuxfamily.org/dox-devel/group__TutorialLinearAlgebra.html
            
            // Assuming the result is computed in vector X
            RCLCPP_INFO(this->get_logger(),"Extracted floor plane: z = %.2fx + %.2fy + %.2f",
                    X(0),X(1),X(2));

            // END OF TODO

            // Now build an orientation vector to display a marker in rviz
            // First we build a basis of the plane normal to its normal vector
            Eigen::Vector3f O,u,v,w;
            w << X(0), X(1), -1.0;
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X(0)+0.0*X(1)+X(2);
            u << 2.0, 0.0, 2.0*X(0)+0.0*X(1)+X(2);
            u -= O;
            u /= u.norm();
            v = w.cross(u);

            // Then we build a rotation matrix out of it
            tf2::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            // And convert it to a quaternion
            tf2::Quaternion Q;
            R.getRotation(Q);
            
            // Documentation on visualization markers can be found on:
            // http://www.ros.org/wiki/rviz/DisplayTypes/Marker
            visualization_msgs::msg::Marker m;
            m.header.stamp = msg->header.stamp;
            m.header.frame_id = base_frame_;
            m.ns = "floor_plane";
            m.id = 1;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = O(0);
            m.pose.position.y = O(1);
            m.pose.position.z = O(2);
            m.pose.orientation = tf2::toMsg(Q);
            m.scale.x = 1.0;
            m.scale.y = 1.0;
            m.scale.z = 0.01;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 1.0;

            // Finally publish the marker
            marker_pub_->publish(m);
            
        }

    public:
        FloorPlaneRegression() : rclcpp::Node("floor_plane_regression") {
            // TODO START
            // The parameter below described the frame in which the point cloud
            // must be projected to be estimated. You need to understand TF
            // enough to find the correct value to update in the launch file
            this->declare_parameter("~/base_frame",std::string("body"));
            base_frame_ = this->get_parameter("~/base_frame").as_string();
            // This parameter defines the maximum range at which we want to
            // consider points. Experiment with the value in the launch file to
            // find something relevant.
            this->declare_parameter("~/max_range",5.0);
            max_range_ = this->get_parameter("~/max_range").as_double();
            // END OF TODO


            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            // Subscribe to the point cloud and prepare the marker publisher
            auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile();
            scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("~/scans",qos,
                    std::bind(&FloorPlaneRegression::pointCloudCallback,this,std::placeholders::_1));
            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("~/floor_plane",1);

        }

};

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloorPlaneRegression>());
    rclcpp::shutdown();
    return 0;
}


