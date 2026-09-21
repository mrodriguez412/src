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
#include <opencv2/opencv.hpp>

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

        cv::Mat_<int32_t> accumulator;

        int n_a, n_b, n_c;
        double a_min, a_max, b_min, b_max, c_min, c_max;
        double da, db, dc;


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
            
#if 0
            // Discretization in principle
            int ia; float a, delta_a;
            ia = round((a-a_min)/delta_a); 
            if ((ia < 0) || (ia >= n_a)) {
                continue; // ignore
            }
            a = a_min + ia * delta_a;
#endif
            
            //
            // BEGIN TODO
            // Finding planes: z = a*x + b*y + c using the hough transform
            // Remember to use the a_min,a_max,n_a variables (resp. b, c).
            n = pidx.size();
            rclcpp::Time now = this->get_clock()->now();
            RCLCPP_INFO(this->get_logger(),"%d useful points out of %d",(int)n,(int)pc_sensor.size());
            // fill the accumulator with zeros
            accumulator = 0;
            size_t best = 0;
            for (unsigned int i=0;i<n;i++) {
                double x = pc_baseframe[pidx[i]].x;
                double y = pc_baseframe[pidx[i]].y;
                double z = pc_baseframe[pidx[i]].z;
                // Update the accumulator based on current point here
                // individual cells in the accumulator can be accessed as follows
                accumulator(0,0,0) = 1;
                // incrementing the counter, as an example
                best += 1;
            }


            double X[3] = {0,0,0};
            // Use the accumulator to find the best plane parameters and store
            // them in X (this will be used for display later)
            // X = {a,b,c}

            rclcpp::Duration dt = this->get_clock()->now() - now;
            // END OF TODO
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
        FloorPlaneRegression() : rclcpp::Node("floor_plane_regression") {
            this->declare_parameter("~/base_frame",std::string("body"));
            this->declare_parameter("~/max_range",5.0);
            base_frame_ = this->get_parameter("~/base_frame").as_string();
            max_range_ = this->get_parameter("~/max_range").as_double();

            // TODO: Update the launch file with relevant values for your problem 
            this->declare_parameter("~/n_a",10);
            this->declare_parameter("~/a_min",1.0);
            this->declare_parameter("~/a_max",1.0);
            this->declare_parameter("~/n_b",10);
            this->declare_parameter("~/b_min",1.0);
            this->declare_parameter("~/b_max",1.0);
            this->declare_parameter("~/n_c",10);
            this->declare_parameter("~/c_min",1.0);
            this->declare_parameter("~/c_max",1.0);

            n_a = this->get_parameter("~/n_a").as_int();
            a_min = this->get_parameter("~/a_min").as_double();
            a_max = this->get_parameter("~/a_max").as_double();
            n_b = this->get_parameter("~/n_b").as_int();
            b_min = this->get_parameter("~/b_min").as_double();
            b_max = this->get_parameter("~/b_max").as_double();
            n_c = this->get_parameter("~/n_c").as_int();
            c_min = this->get_parameter("~/c_min").as_double();
            c_max = this->get_parameter("~/c_max").as_double();

            assert(n_a > 0);
            assert(n_b > 0);
            assert(n_c > 0);

            RCLCPP_INFO(this->get_logger(),"Searching for Plane parameter z = a x + b y + c");
            RCLCPP_INFO(this->get_logger(),"a: %d value in [%f, %f]",n_a,a_min,a_max);
            RCLCPP_INFO(this->get_logger(),"b: %d value in [%f, %f]",n_b,b_min,b_max);
            RCLCPP_INFO(this->get_logger(),"c: %d value in [%f, %f]",n_c,c_min,c_max);

            // BEGIN TODO
            // Prepare da, db, dc, the steps between values based on min,max,n
            // for each dimension

            da = 1;
            db = 1;
            dc = 1;

            // END TODO

            // the accumulator is created here as a 3D matrix of size n_a x n_b x n_c
            int dims[3] = {n_a,n_b,n_c};
            accumulator = cv::Mat_<int32_t>(3,dims);


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


