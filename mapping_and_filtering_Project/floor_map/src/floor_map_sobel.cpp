// 1. Bibliothèques standards C++
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <limits>

// 2. ROS 2 et messages
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// 3. PCL et conversions
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// 4. TF2
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// 5. OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class FloorMapSobel : public rclcpp::Node {
protected:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_pub_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string target_frame_;

    // Paramètres géométriques de la grille
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    double resolution_;
    int width_cells_;
    int height_cells_;

    // Paramètres d'évaluation de traversabilité
    double max_slope_angle_;  // Seuil de pente binaire (ex: 15° en radians)
    double max_scale_angle_;  // Échelle max pour le coût continu [0, 100] (ex: 30° en radians)
    double max_z_diff_;       // Seuil de saut vertical dans une cellule (ex: 0.08 m)
    int min_points_bucket_;   // Nombre minimum de points requis par cellule

    // Digital Elevation Model (DEM)
    cv::Mat dem_sum_z_;       // Somme cumulée des Z (CV_32FC1)
    cv::Mat dem_count_;       // Nombre de points accumulés (CV_32SC1)
    cv::Mat dem_mean_z_;      // Hauteur moyenne Z = dem_sum_z_ / dem_count_ (CV_32FC1)
    cv::Mat dem_min_z_;       // Hauteur min observée (CV_32FC1)
    cv::Mat dem_max_z_;       // Hauteur max observée (CV_32FC1)
    cv::Mat valid_mask_;      // Masque des cellules observées (CV_8UC1, 255 si count >= min_points)

    // Cartes publiées
    cv::Mat map_;             // Carte de traversabilité binaire (CV_8UC1 : 0=obs, 255=libre, 127=inconnu)
    cv::Mat cost_map_;        // Carte de coût continue (CV_32FC1 : 0.0f à 100.0f, -1.0f=inconnu)

protected:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> pc_sensor, pc_target;
        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL(*msg, cloud2);
        pcl::fromPCLPointCloud2(cloud2, pc_sensor);

        if (msg->header.frame_id != target_frame_) {
            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                std::string errStr;
                if (!tf_buffer_->canTransform(target_frame_, msg->header.frame_id, msg->header.stamp,
                            rclcpp::Duration(std::chrono::duration<double>(1.0)), &errStr)) {
                    RCLCPP_ERROR(this->get_logger(), "Cannot transform target: %s", errStr.c_str());
                    return;
                }
                transformStamped = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp);
                sensor_msgs::msg::PointCloud2 pc;
                tf2::doTransform(*msg, pc, transformStamped);
                pcl_conversions::toPCL(pc, cloud2);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                return;
            }
        } else {
            pcl_conversions::toPCL(*msg, cloud2);
        }
        pcl::fromPCLPointCloud2(cloud2, pc_target);

        // 1. Accumulation glissante des points dans le DEM (Option A)
        for (const auto & p : pc_target) {
            if (std::hypot(p.x, p.y) < 1e-2) continue; // ignore les points à l'origine du capteur
            int gx, gy;
            if (worldToGrid(p.x, p.y, gx, gy)) {
                dem_sum_z_.at<float>(gy, gx) += p.z;
                int count = ++dem_count_.at<int>(gy, gx);
                dem_mean_z_.at<float>(gy, gx) = dem_sum_z_.at<float>(gy, gx) / count;

                if (p.z < dem_min_z_.at<float>(gy, gx)) dem_min_z_.at<float>(gy, gx) = p.z;
                if (p.z > dem_max_z_.at<float>(gy, gx)) dem_max_z_.at<float>(gy, gx) = p.z;

                if (count >= min_points_bucket_) {
                    valid_mask_.at<uint8_t>(gy, gx) = 255;
                }
            }
        }

        // 2. Gestion des frontières et calcul de gradient par Sobel
        // Érosion 3x3 : seules les cellules entourées de 8 voisines explorées sont évaluées par Sobel
        cv::Mat valid_interior;
        cv::erode(valid_mask_, valid_interior, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        // Échelle normalisée pour Sobel : dZ/dx = (Sobel_x * Z) / (8 * resolution)
        double scale = 1.0 / (8.0 * resolution_);
        cv::Mat grad_x, grad_y, grad_mag;
        cv::Sobel(dem_mean_z_, grad_x, CV_32F, 1, 0, 3, scale);
        cv::Sobel(dem_mean_z_, grad_y, CV_32F, 0, 1, 3, scale);
        cv::magnitude(grad_x, grad_y, grad_mag);

        // 3. Évaluation de la traversabilité cellule par cellule
        for (int gy = 0; gy < height_cells_; ++gy) {
            for (int gx = 0; gx < width_cells_; ++gx) {
                if (valid_mask_.at<uint8_t>(gy, gx) == 0) {
                    continue; // Cellule pas encore assez observée
                }

                // A. Détection des marches verticales abruptes internes (cylindres, murs)
                float dz = dem_max_z_.at<float>(gy, gx) - dem_min_z_.at<float>(gy, gx);
                bool internal_step = (dz > max_z_diff_);

                if (internal_step) {
                    map_.at<uint8_t>(gy, gx) = 0; // Obstacle (Noir)
                    cost_map_.at<float>(gy, gx) = 100.0f;
                    continue;
                }

                // B. Évaluation par pente Sobel pour les cellules à voisinage complet
                if (valid_interior.at<uint8_t>(gy, gx) == 255) {
                    float slope = grad_mag.at<float>(gy, gx); // tan(theta)
                    double theta = std::atan(slope);          // angle de pente en radians

                    bool is_steep = (theta > max_slope_angle_);

                    // Mappage continu sur [0, 100]
                    double ratio = theta / max_scale_angle_;
                    float cost = static_cast<float>(std::min(100.0, std::max(0.0, ratio * 100.0)));

                    // Mise à jour de map_ (Binaire sécuritaire : une fois obstacle, reste obstacle)
                    if (is_steep) {
                        map_.at<uint8_t>(gy, gx) = 0;   // Obstacle (Noir)
                    } else if (map_.at<uint8_t>(gy, gx) == 127) {
                        map_.at<uint8_t>(gy, gx) = 255; // Traversable (Blanc)
                    }

                    // Mise à jour de cost_map_ (Conserve le coût le plus sévère)
                    if (cost_map_.at<float>(gy, gx) < 0.0f) {
                        cost_map_.at<float>(gy, gx) = cost;
                    } else {
                        cost_map_.at<float>(gy, gx) = std::max(cost_map_.at<float>(gy, gx), cost);
                    }
                }
            }
        }
    }

    bool worldToGrid(double wx, double wy, int & gx, int & gy) const {
        if (wx < min_x_ || wx >= max_x_ || wy < min_y_ || wy >= max_y_) {
            return false;
        }
        gx = static_cast<int>((wx - min_x_) / resolution_);
        gy = static_cast<int>((wy - min_y_) / resolution_);
        return true;
    }

    void publishMap() {
        if (map_.empty()) {
            return;
        }

        // 1. Publication de la carte binaire de traversabilité
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
                    grid.data[index] = 100; // Obstacle
                } else {
                    grid.data[index] = -1;  // Inconnu
                }
            }
        }

        map_pub_->publish(grid);

        // 2. Publication de la carte de coût continue (0 à 100)
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
    FloorMapSobel() : rclcpp::Node("floor_map_sobel") {
        // Déclaration des paramètres ROS
        this->declare_parameter("min_x", -10.0);
        this->declare_parameter("max_x", 10.0);
        this->declare_parameter("min_y", -10.0);
        this->declare_parameter("max_y", 10.0);
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("target_frame", std::string("world"));
        this->declare_parameter("max_slope_angle", 15.0 * M_PI / 180.0); // 15° en rad
        this->declare_parameter("max_scale_angle", 30.0 * M_PI / 180.0); // 30° en rad
        this->declare_parameter("max_z_diff", 0.08);                     // 8 cm
        this->declare_parameter("min_points_bucket", 5);

        // Lecture des paramètres
        min_x_ = this->get_parameter("min_x").as_double();
        max_x_ = this->get_parameter("max_x").as_double();
        min_y_ = this->get_parameter("min_y").as_double();
        max_y_ = this->get_parameter("max_y").as_double();
        resolution_ = this->get_parameter("resolution").as_double();
        target_frame_ = this->get_parameter("target_frame").as_string();
        max_slope_angle_ = this->get_parameter("max_slope_angle").as_double();
        max_scale_angle_ = this->get_parameter("max_scale_angle").as_double();
        max_z_diff_ = this->get_parameter("max_z_diff").as_double();
        min_points_bucket_ = this->get_parameter("min_points_bucket").as_int();

        // Calcul des dimensions de la grille
        width_cells_ = static_cast<int>(std::round((max_x_ - min_x_) / resolution_));
        height_cells_ = static_cast<int>(std::round((max_y_ - min_y_) / resolution_));

        // Initialisation des matrices DEM
        dem_sum_z_ = cv::Mat::zeros(height_cells_, width_cells_, CV_32FC1);
        dem_count_ = cv::Mat::zeros(height_cells_, width_cells_, CV_32SC1);
        dem_mean_z_ = cv::Mat::zeros(height_cells_, width_cells_, CV_32FC1);
        dem_min_z_ = cv::Mat(height_cells_, width_cells_, CV_32FC1, cv::Scalar(std::numeric_limits<float>::infinity()));
        dem_max_z_ = cv::Mat(height_cells_, width_cells_, CV_32FC1, cv::Scalar(-std::numeric_limits<float>::infinity()));
        valid_mask_ = cv::Mat::zeros(height_cells_, width_cells_, CV_8UC1);

        // Allocation des matrices de cartes
        map_ = cv::Mat(height_cells_, width_cells_, CV_8UC1, cv::Scalar(127));          // 127 = Inconnu
        cost_map_ = cv::Mat(height_cells_, width_cells_, CV_32FC1, cv::Scalar(-1.0f));  // -1.0 = Inconnu

        // Initialisation de TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Souscription au nuage de points
        auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile();
        scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points", qos,
            std::bind(&FloorMapSobel::pointCloudCallback, this, std::placeholders::_1));

        // Publishers pour RViz2
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/traversability_map", 1);
        cost_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/cost_map", 1);

        // Timer à 2 Hz (500 ms) pour rafraîchir et publier les cartes
        map_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&FloorMapSobel::publishMap, this));

        RCLCPP_INFO(this->get_logger(),
                    "FloorMapSobel pret : grille %dx%d (resolution %.2fm), seuil pente %.1f deg",
                    width_cells_, height_cells_, resolution_, max_slope_angle_ * 180.0 / M_PI);
    }

    virtual ~FloorMapSobel() = default;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloorMapSobel>());
    rclcpp::shutdown();
    return 0;
}
