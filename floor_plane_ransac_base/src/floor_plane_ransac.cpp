#include <cstdio>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>

class FloorPlaneRegression: public rclcpp::Node {
    protected:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inlier_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        // rclcpp::Client<topic_tools::srv::MuxSelect>::SharedPtr muxClt;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;


        std::string base_frame_;
        double max_range_;
        size_t n_samples_;
        double tolerance_;

        std::random_device rd;  // a seed source for the random number engine
        std::mt19937 gen; // mersenne_twister_engine seeded with rd()

    protected: // ROS Callbacks

        void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> pc_sensor, pc_baseframe, pc_inliers;
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
            // BEGIN TODO
            // Finding planes: z = a*x + b*y + c
            // Remember to use the n_samples and the tolerance variable
            n = pidx.size();
            size_t best = 0;
            double X[3] = {0,0,0};
            rclcpp::Time now = this->get_clock()->now();
            RCLCPP_INFO(this->get_logger(),"%d useful points out of %d",(int)n,(int)pc_sensor.size());
            std::vector<size_t> pinliers;
            std::uniform_int_distribution<> dsample(0, n-1);
            for (unsigned int i=0;i<(unsigned)n_samples_;i++) {
                // Implement RANSAC here. Useful commands:
                // Select a random number in in [0,i-1]
                size_t j = dsample(gen);
                // Create a 3D point:
                // Eigen::Vector3f P; P << x,y,z;
                Eigen::Vector3f P; P << pc_baseframe[pidx[0]].x, pc_baseframe[pidx[0]].y, pc_baseframe[pidx[0]].z; 
                // Dot product
                double x = P.dot(P);
                // Cross product
                Eigen::Vector3f Q = P.cross(P); 
                // Vector norm
                double norm = P.norm();

                double a1 = abs(-0.6);
                double a2 = fabs(-0.6);
                double a3 = std::abs<double>(-0.6);


            }
            // At the end, make sure to store the best plane estimate in X
            // X = {a,b,c}. This will be used for display
            rclcpp::Duration dt = this->get_clock()->now() - now;
            // END OF TODO
            //
            RCLCPP_INFO(this->get_logger(),"Extracted floor plane: z = %.2fx + %.2fy + %.2f: %.3fs, score %d",
                    X[0],X[1],X[2],dt.seconds(),(int)best);

            Eigen::Vector3f O,u,v,w;
            w << X[0], X[1], -1.0;
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X[0]+0.0*X[1]+X[2];
            u << 2.0, 0.0, 2.0*X[0]+0.0*X[1]+X[2];
            u -= O;
            u /= u.norm();
            v = w.cross(u);

            tf2::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            tf2::Quaternion Q;
            R.getRotation(Q);
            
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

            marker_pub_->publish(m);
            
        }

    public:
        FloorPlaneRegression() : rclcpp::Node("floor_plane_regression"), gen(rd()) {
            this->declare_parameter("~/base_frame",std::string("body"));
            this->declare_parameter("~/max_range",5.0);
            this->declare_parameter("~/n_samples",1000);
            this->declare_parameter("~/tolerance",1.0);
            base_frame_ = this->get_parameter("~/base_frame").as_string();
            max_range_ = this->get_parameter("~/max_range").as_double();
            n_samples_ = this->get_parameter("~/n_samples").as_int();
            tolerance_ = this->get_parameter("~/tolerance").as_double();

            RCLCPP_INFO(this->get_logger(),"Searching for Plane parameter z = a x + b y + c");
            RCLCPP_INFO(this->get_logger(),"RANSAC: %lu iteration with %f tolerance",n_samples_,tolerance_);
            assert(n_samples_ > 0);
            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile();
            scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("~/scans",qos,
                    std::bind(&FloorPlaneRegression::pointCloudCallback,this,std::placeholders::_1));
            inlier_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/inliers",1);
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


