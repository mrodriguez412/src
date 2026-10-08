// 1. Bibliothèques standards C++
#include <cstdio>
#include <random>
#include <map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <opencv2/core.hpp>

class FloorMapBucket: public rclcpp::Node {
    protected:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_pub_;
        rclcpp::TimerBase::SharedPtr map_timer_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string target_frame_;
        
        double min_x_;       
        double max_x_;       
        double min_y_;       
        double max_y_;       
        double resolution_;  
        int width_cells_;    
        int height_cells_;   
        
        std::string metric_; 
        double max_z_diff_; 
        double max_z_sigma_;
        double max_scale_angle_;  
        double max_scale_height_; 
        double min_range_;
        double max_range_;
        int min_points_bucket_;   
        
        cv::Mat map_;
        cv::Mat cost_map_; // Carte de coût continu (0 à 100), -1.0f = inconnu


    protected:
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
            pcl::PointCloud<pcl::PointXYZ> pc_sensor, pc_target, pc_inliers;
            pcl::PCLPointCloud2 cloud2;
            pcl_conversions::toPCL(*msg,cloud2);    
            pcl::fromPCLPointCloud2(cloud2,pc_sensor);

            if (msg->header.frame_id != target_frame_) {
                geometry_msgs::msg::TransformStamped transformStamped;
                try {
                    std::string errStr;
                    if (!tf_buffer_->canTransform(target_frame_, msg->header.frame_id, msg->header.stamp,
                                rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                        RCLCPP_ERROR(this->get_logger(),"Cannot transform target: %s",errStr.c_str());
                        return;
                    }
                    transformStamped = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp);
                    sensor_msgs::msg::PointCloud2 pc;
                    tf2::doTransform(*msg,pc,transformStamped);

                    // ROS2 Pointcloud2 to PCL Pointcloud2
                    pcl_conversions::toPCL(pc,cloud2);    
                } catch (const tf2::TransformException & ex){
                    RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
                    return;
                }
            } else {
                // ROS2 Pointcloud2 to PCL Pointcloud2
                pcl_conversions::toPCL(*msg,cloud2);   
            }
            // PCL Pointcloud2 to templated form
            pcl::fromPCLPointCloud2(cloud2,pc_target);

            // 1. Remplissage des buckets avec filtrage 3D de la portée capteur
            std::map<std::pair<int, int>, std::vector<pcl::PointXYZ>> buckets;
            size_t n = pc_sensor.size();
            for (size_t i = 0; i < n; ++i) {
                const auto & ps = pc_sensor[i];
                const auto & pt = pc_target[i];

                // Rejet des NaN / Inf
                if (!std::isfinite(ps.x) || !std::isfinite(ps.y) || !std::isfinite(ps.z) ||
                    !std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                    continue;
                }

                // Distance 3D euclidienne au centre optique de la caméra Kinect
                float d_sensor = std::sqrt(ps.x * ps.x + ps.y * ps.y + ps.z * ps.z);
                if (d_sensor < min_range_ || d_sensor > max_range_) {
                    continue;
                }

                int gx, gy;
                if (worldToGrid(pt.x, pt.y, gx, gy)) {
                    buckets[{gx, gy}].push_back(pt);
                }
            }
            

            // 2. Évaluation de chaque bucket
            for (const auto & entry : buckets) {
                int gx = entry.first.first;
                int gy = entry.first.second;
                const auto & pts = entry.second;

                if (pts.size() < static_cast<size_t>(min_points_bucket_)) {
                    continue; // Pas assez de points pour conclure
                }

                bool is_obstacle = false;
                float cost = 0.0f;

                if (metric_ == "zdiff") {
                    // Méthode par écart de hauteur max - min
                    double min_z = pts[0].z;
                    double max_z = pts[0].z;
                    for (const auto & pt : pts) {
                        if (pt.z < min_z) min_z = pt.z;
                        if (pt.z > max_z) max_z = pt.z;
                    }
                    double diff_z = max_z - min_z;
                    is_obstacle = (diff_z > max_z_diff_);

                    // Mappage linéaire continu sur [0, 100]
                    double ratio = diff_z / max_scale_height_;
                    cost = static_cast<float>(std::min(100.0, std::max(0.0, ratio * 100.0)));
                } else {
                    // Méthode par écart-type (zvar)
                    double sum_z = 0.0;
                    for (const auto & pt : pts) sum_z += pt.z;
                    double mean_z = sum_z / pts.size();

                    double sum_sq = 0.0;
                    for (const auto & pt : pts) {
                        double d = pt.z - mean_z;
                        sum_sq += d * d;
                    }
                    double sigma_z = std::sqrt(sum_sq / pts.size());
                    is_obstacle = (sigma_z > max_z_sigma_);

                    // Estimation de l'angle sous hypothèse d'un plan lisse :
                    // sigma_z = tan(theta) * resolution_ / sqrt(12) => slope = sigma_z * sqrt(12) / resolution_
                    double slope = (sigma_z * std::sqrt(12.0)) / resolution_;
                    double estimated_angle = std::atan(slope); // en radians
                    double ratio = estimated_angle / max_scale_angle_;
                    cost = static_cast<float>(std::min(100.0, std::max(0.0, ratio * 100.0)));
                }

                // 3. Mise à jour de map_ (Binaire sécuritaire)
                if (is_obstacle) {
                    map_.at<uint8_t>(gy, gx) = 0;   // Obstacle (Noir)
                } else if (map_.at<uint8_t>(gy, gx) == 127) {
                    map_.at<uint8_t>(gy, gx) = 255; // Traversable (Blanc)
                }

                // 4. Mise à jour de cost_map_ (Continu [0, 100])
                if (cost_map_.at<float>(gy, gx) < 0.0f) {
                    cost_map_.at<float>(gy, gx) = cost;
                } else {
                    // Conserve le coût le plus sévère rencontré
                    cost_map_.at<float>(gy, gx) = std::max(cost_map_.at<float>(gy, gx), cost);
                }
            }
        }
        
        bool worldToGrid(double wx, double wy, int & gx, int & gy) const {
            if (wx < min_x_ || wx >= max_x_ || wy < min_y_ || wy >= max_y_) {
                return false; // En dehors de la carte
            }
            gx = static_cast<int>((wx - min_x_) / resolution_);
            gy = static_cast<int>((wy - min_y_) / resolution_);
            return true;
        }
        void publishMap() {
            if (map_.empty()) {
                return;
            }

            nav_msgs::msg::OccupancyGrid grid;
            grid.header.stamp = this->get_clock()->now();
            grid.header.frame_id = target_frame_;

            grid.info.map_load_time = grid.header.stamp;
            grid.info.resolution = static_cast<float>(resolution_);
            grid.info.width = static_cast<uint32_t>(width_cells_);
            grid.info.height = static_cast<uint32_t>(height_cells_);
            grid.info.origin.position.x = min_x_;
            grid.info.origin.position.y = min_y_;
            grid.info.origin.position.z = 0.0;
            grid.info.origin.orientation.w = 1.0;

            grid.data.resize(width_cells_ * height_cells_, -1);

            for (int y = 0; y < height_cells_; ++y) {
                for (int x = 0; x < width_cells_; ++x) {
                    uint8_t val = map_.at<uint8_t>(y, x);
                    int index = y * width_cells_ + x;

                    if (val == 255) {
                        grid.data[index] = 0;   // Traversable (Libre)
                    } else if (val == 0) {
                        grid.data[index] = 100; // Non-traversable (Obstacle)
                    } else {
                        grid.data[index] = -1;  // Inconnu (127)
                    }
                }
            }

            map_pub_->publish(grid);

            // 4. Publication de la carte de coût continue (0 à 100)
            if (!cost_map_.empty()) {
                nav_msgs::msg::OccupancyGrid cost_grid;
                cost_grid.header = grid.header;
                cost_grid.info = grid.info;
                cost_grid.data.resize(width_cells_ * height_cells_, -1);

                for (int y = 0; y < height_cells_; ++y) {
                    for (int x = 0; x < width_cells_; ++x) {
                        float c = cost_map_.at<float>(y, x);
                        int index = y * width_cells_ + x;
                        if (c < 0.0f) {
                            cost_grid.data[index] = -1; // Inconnu
                        } else {
                            cost_grid.data[index] = static_cast<int8_t>(std::round(c));
                        }
                    }
                }
                cost_map_pub_->publish(cost_grid);
            }
        }
    public:
        FloorMapBucket() : rclcpp::Node("floor_map_bucket") {
            // Déclaration et lecture des paramètres ROS
            this->declare_parameter("min_x", -10.0);
            this->declare_parameter("max_x", 10.0);
            this->declare_parameter("min_y", -10.0);
            this->declare_parameter("max_y", 10.0);
            this->declare_parameter("resolution", 0.1);
            this->declare_parameter("target_frame", std::string("world"));
            this->declare_parameter("metric", std::string("zvar"));
            this->declare_parameter("max_z_diff", 0.08);
            this->declare_parameter("max_z_sigma", 0.02);
            this->declare_parameter("max_scale_angle", 30.0 * M_PI / 180.0);
            this->declare_parameter("max_scale_height", 0.5);
            this->declare_parameter("min_range", 0.1);
            this->declare_parameter("max_range", 4.0);
            this->declare_parameter("min_points_bucket", 5);

            min_x_ = this->get_parameter("min_x").as_double();
            max_x_ = this->get_parameter("max_x").as_double();
            min_y_ = this->get_parameter("min_y").as_double();
            max_y_ = this->get_parameter("max_y").as_double();
            resolution_ = this->get_parameter("resolution").as_double();
            target_frame_ = this->get_parameter("target_frame").as_string();
            metric_ = this->get_parameter("metric").as_string();
            max_z_diff_ = this->get_parameter("max_z_diff").as_double();
            max_z_sigma_ = this->get_parameter("max_z_sigma").as_double();
            max_scale_angle_ = this->get_parameter("max_scale_angle").as_double();
            max_scale_height_ = this->get_parameter("max_scale_height").as_double();
            min_range_ = this->get_parameter("min_range").as_double();
            max_range_ = this->get_parameter("max_range").as_double();
            min_points_bucket_ = this->get_parameter("min_points_bucket").as_int();

            // Calcul des dimensions de la grille
            width_cells_ = static_cast<int>(std::round((max_x_ - min_x_) / resolution_));
            height_cells_ = static_cast<int>(std::round((max_y_ - min_y_) / resolution_));

            // Allocation des matrices OpenCV
            map_ = cv::Mat(height_cells_, width_cells_, CV_8UC1, cv::Scalar(127));          // 127 = Inconnu
            cost_map_ = cv::Mat(height_cells_, width_cells_, CV_32F, cv::Scalar(-1.0f));    // -1.0 = Inconnu

            // Initialisation de TF2
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Souscription au nuage de points 3D de la Kinect (/points)
            auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile();
            scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/points", qos,
                std::bind(&FloorMapBucket::pointCloudCallback, this, std::placeholders::_1));

            // Publishers pour RViz2 : carte binaire et carte de coût continue
            map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/traversability_map", 1);
            cost_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/cost_map", 1);

            // Timer à 2 Hz (500 ms) pour rafraîchir et publier les cartes en continu
            //map_timer_ = this->create_wall_timer(
            //    std::chrono::milliseconds(100),
            //    std::bind(&FloorMapBucket::publishMap, this));

            RCLCPP_INFO(this->get_logger(),
                        "FloorMapBucket pret : grille %dx%d (resolution %.2fm), metrique '%s'",
                        width_cells_, height_cells_, resolution_, metric_.c_str());
        }

        virtual ~FloorMapBucket() = default;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloorMapBucket>());
    rclcpp::shutdown();
    return 0;
}