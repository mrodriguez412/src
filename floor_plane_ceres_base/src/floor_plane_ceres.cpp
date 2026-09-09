#include <cstdio>
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

#include "ceres/ceres.h"
#include "ceres/solver.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(trust_region_strategy, "levenberg_marquardt",
        "Options are: levenberg_marquardt, dogleg.");
DEFINE_string(dogleg, "traditional_dogleg", "Options are: traditional_dogleg,"
        "subspace_dogleg.");

DEFINE_bool(inner_iterations, false, "Use inner iterations to non-linearly "
        "refine each successful trust region step.");

DEFINE_string(blocks_for_inner_iterations, "automatic", "Options are: "
        "automatic, cameras, points, cameras,points, points,cameras");

DEFINE_string(linear_solver, "sparse_normal_cholesky", "Options are: "
        "sparse_schur, dense_schur, iterative_schur, sparse_normal_cholesky, "
        "dense_qr, dense_normal_cholesky and cgnr.");

DEFINE_string(preconditioner, "jacobi", "Options are: "
        "identity, jacobi, schur_jacobi, cluster_jacobi, "
        "cluster_tridiagonal.");

DEFINE_string(sparse_linear_algebra_library, "suite_sparse",
        "Options are: suite_sparse and cx_sparse.");

DEFINE_string(ordering, "automatic", "Options are: automatic, user.");

DEFINE_bool(robustify, false, "Use a robust loss function.");
DEFINE_double(eta, 1e-2, "Default value for eta. Eta determines the "
        "accuracy of each linear solve of the truncated newton step. "
        "Changing this parameter can affect solve performance.");

DEFINE_int32(num_threads, 1, "Number of threads.");
DEFINE_int32(num_iterations, 50, "Number of iterations.");
DEFINE_double(max_solver_time, 1e32, "Maximum solve time in seconds.");
DEFINE_bool(nonmonotonic_steps, false, "Trust region algorithm can use"
        " nonmonotic steps.");

struct PlaneError {
    PlaneError(double x, double y, double z, double weight=1)
        : x(x), y(y), z(z), weight(weight) { }

    template <typename T>
        bool operator()(const T* const w,
                T* residuals) const {
            // TODO START
            // The error is the difference between the predicted and a position.
            // Update this value to make it a proper measurement error
            // Check the CERES optimizer web-page for the documentation: 
            // http://homes.cs.washington.edu/~sagarwal/ceres-solver/stable/tutorial.html#chapter-tutorial
            residuals[0] = T(0.0);

            // END OF TODO

            return true;
        }

    double x,y,z, weight;
};

class FloorPlaneRegression: public rclcpp::Node {
    protected:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        // rclcpp::Client<topic_tools::srv::MuxSelect>::SharedPtr muxClt;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;


        std::string base_frame_;
        double max_range_;

        ceres::Solver::Options options;

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
            // Linear regression:  z = a*x + b*y + c
            ceres::Problem problem;
            double X[3] = {0,0,0};
            n = pidx.size();
            for (unsigned int i=0;i<n;i++) {
                const pcl::PointXYZ & P = pc_baseframe[pidx[i]];
                ceres::LossFunction* loss_function;
                ceres::CostFunction *cost_function;
                loss_function = FLAGS_robustify ? new ceres::HuberLoss(1.0) : NULL;
                // TODO START
                // Use the PlaneError defined above to build an error term for
                // the ceres optimiser (see documentation link above)
                cost_function = NULL;
                // END OF TODO
                // This cost function is then added to the optimisation
                // problem, with X as a parameter
                problem.AddResidualBlock(cost_function,loss_function,X);
            }
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            RCLCPP_INFO(this->get_logger(),"Extracted floor plane: z = %.2fx + %.2fy + %.2f",
                    X[0],X[1],X[2]);

            Eigen::Vector3f O,u,v,w;
            w << X[0],X[1],-1;
            w /= w.norm();
            // Assuming 
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
            // TODO: update these variable in the launch file, similarly to the
            // linear regression case
            this->declare_parameter("~/base_frame",std::string("body"));
            this->declare_parameter("~/max_range",5.0);
            base_frame_ = this->get_parameter("~/base_frame").as_string();
            max_range_ = this->get_parameter("~/max_range").as_double();

            // Prepare the CERES options based on the command line arguments
            CHECK(StringToLinearSolverType(FLAGS_linear_solver,
                        &options.linear_solver_type));
            CHECK(StringToPreconditionerType(FLAGS_preconditioner,
                        &options.preconditioner_type));
            // CHECK(StringToSparseLinearAlgebraLibraryType(
            //             FLAGS_sparse_linear_algebra_library,
            //             &options.sparse_linear_algebra_library));
            options.num_threads = FLAGS_num_threads;
            options.max_num_iterations = FLAGS_num_iterations;
            options.minimizer_progress_to_stdout = true;
            options.num_threads = FLAGS_num_threads;
            options.eta = FLAGS_eta;
            options.function_tolerance = 1e-6;
            options.max_solver_time_in_seconds = FLAGS_max_solver_time;
            options.use_nonmonotonic_steps = FLAGS_nonmonotonic_steps;
            options.update_state_every_iteration = false; 

            CHECK(StringToTrustRegionStrategyType(FLAGS_trust_region_strategy,
                        &options.trust_region_strategy_type));
            CHECK(StringToDoglegType(FLAGS_dogleg, &options.dogleg_type));
            options.use_inner_iterations = FLAGS_inner_iterations;

            this->declare_parameter("~/function_tolerance",options.function_tolerance);
            this->declare_parameter("~/max_num_iterations",options.max_num_iterations);
            this->declare_parameter("~/num_threads",options.num_threads);
            this->declare_parameter("~/minimizer_progress_to_stdout",options.minimizer_progress_to_stdout);
            options.function_tolerance = this->get_parameter("~/function_tolerance").as_double();
            options.max_num_iterations = this->get_parameter("~/max_num_iterations").as_int();
            options.num_threads = this->get_parameter("~/num_threads").as_int();
            options.minimizer_progress_to_stdout = this->get_parameter("~/minimizer_progress_to_stdout").as_bool();

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

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


