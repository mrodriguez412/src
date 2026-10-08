#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// Detects vertical cylinders in the Kinect point cloud and keeps a map of all the
// cylinders seen so far. In the world frame a vertical cylinder is a circle in (x,y)
// plus a height, so the detection is a 2D circle RANSAC on each obstacle cluster.
class CylinderDetector: public rclcpp::Node {
    protected:
        struct Circle {
            double cx, cy, r;
        };

        struct Cylinder {
            double cx, cy, r;
            double z_min, z_max;
            double weight; // Number of inliers merged into this estimate
            int n_obs;     // Number of detections merged into this estimate
        };

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inliers_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::TimerBase::SharedPtr marker_timer_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string target_frame_;
        double min_range_;
        double max_range_;
        double floor_clearance_;
        double max_height_;
        double voxel_size_;
        double cluster_resolution_;
        int n_samples_;
        double tolerance_;
        double min_radius_;
        double max_radius_;
        int min_inliers_;
        int max_fits_per_cluster_;
        double min_inlier_ratio_;
        double support_margin_;
        double min_arc_angle_;
        double min_height_coverage_;
        double min_height_;
        double assoc_distance_;
        int min_observations_;

        std::random_device rd_;
        std::mt19937 gen_;

        std::vector<Cylinder> cylinders_; // Every cylinder detected so far, in target_frame_

    protected:
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> pc_sensor, pc_target;
            pcl::PCLPointCloud2 cloud2;
            pcl_conversions::toPCL(*msg,cloud2);
            pcl::fromPCLPointCloud2(cloud2,pc_sensor);

            // The transform is always needed: its translation is the sensor position in target_frame_
            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                std::string errStr;
                if (!tf_buffer_->canTransform(target_frame_, msg->header.frame_id, msg->header.stamp,
                            rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                    RCLCPP_ERROR(this->get_logger(),"Cannot transform target: %s",errStr.c_str());
                    return;
                }
                transformStamped = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp);
            } catch (const tf2::TransformException & ex){
                RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
                return;
            }
            sensor_msgs::msg::PointCloud2 pc;
            tf2::doTransform(*msg,pc,transformStamped);
            pcl_conversions::toPCL(pc,cloud2);
            pcl::fromPCLPointCloud2(cloud2,pc_target);
            const double sensor_x = transformStamped.transform.translation.x;
            const double sensor_y = transformStamped.transform.translation.y;

            // 1. Local ground: the lowest point of each 2D cell. Upright objects stand on the
            // ground, so this also holds under obstacles, on ramps and on every level.
            std::vector<size_t> valid;
            std::unordered_map<int64_t, float> ground;
            for (size_t i = 0; i < pc_sensor.size(); ++i) {
                const auto & ps = pc_sensor[i];
                const auto & pt = pc_target[i];

                if (!std::isfinite(ps.x) || !std::isfinite(ps.y) || !std::isfinite(ps.z) ||
                    !std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                    continue;
                }

                float d_sensor = std::sqrt(ps.x * ps.x + ps.y * ps.y + ps.z * ps.z);
                if (d_sensor < min_range_ || d_sensor > max_range_) {
                    continue;
                }

                valid.push_back(i);
                float & g = ground.emplace(gridKey(pt.x, pt.y, 0.0, cluster_resolution_), pt.z).first->second;
                g = std::min(g, pt.z);
            }

            // Keep the points high enough above their local ground, one point per voxel
            pcl::PointCloud<pcl::PointXYZ> obstacles;
            std::unordered_set<int64_t> voxels;
            for (size_t i : valid) {
                const auto & pt = pc_target[i];
                float h = pt.z - ground[gridKey(pt.x, pt.y, 0.0, cluster_resolution_)];
                if (h > floor_clearance_ && h < max_height_ &&
                        voxels.insert(gridKey(pt.x, pt.y, pt.z, voxel_size_)).second) {
                    obstacles.push_back(pt);
                }
            }

            // 2. and 3. Cluster the obstacles and fit cylinders in each cluster
            pcl::PointCloud<pcl::PointXYZ> pc_inliers;
            std::vector<Cylinder> detections = detectCylinders(obstacles, sensor_x, sensor_y, pc_inliers);

            // 4. Merge the detections into the map of cylinders
            for (const auto & d : detections) {
                size_t k = integrate(d);
                if (cylinders_[k].n_obs == min_observations_) {
                    RCLCPP_INFO(this->get_logger(), "Cylinder #%zu confirmed at (%.2f, %.2f), r = %.2f",
                            k, cylinders_[k].cx, cylinders_[k].cy, cylinders_[k].r);
                }
            }

            publishCloud(obstacles, msg->header, obstacles_pub_);
            publishCloud(pc_inliers, msg->header, inliers_pub_);
        }

        // Key of the grid cell holding (x,y,z): 21 bits per axis, unique within +/- 2^20 cells
        static int64_t gridKey(double x, double y, double z, double res) {
            int64_t ix = static_cast<int64_t>(std::floor(x / res));
            int64_t iy = static_cast<int64_t>(std::floor(y / res));
            int64_t iz = static_cast<int64_t>(std::floor(z / res));
            return ((ix & 0x1FFFFF) << 42) | ((iy & 0x1FFFFF) << 21) | (iz & 0x1FFFFF);
        }

        std::vector<Cylinder> detectCylinders(const pcl::PointCloud<pcl::PointXYZ> & obstacles,
                double sensor_x, double sensor_y, pcl::PointCloud<pcl::PointXYZ> & pc_inliers) {
            std::vector<Cylinder> detections;
            if (obstacles.empty()) {
                return detections;
            }

            // 2. Cluster the obstacles: rasterise them on a 2D grid and take the connected components
            double min_x = obstacles[0].x, max_x = min_x;
            double min_y = obstacles[0].y, max_y = min_y;
            for (const auto & p : obstacles) {
                min_x = std::min<double>(min_x, p.x);
                max_x = std::max<double>(max_x, p.x);
                min_y = std::min<double>(min_y, p.y);
                max_y = std::max<double>(max_y, p.y);
            }
            int width = static_cast<int>((max_x - min_x) / cluster_resolution_) + 1;
            int height = static_cast<int>((max_y - min_y) / cluster_resolution_) + 1;
            cv::Mat occupied = cv::Mat::zeros(height, width, CV_8UC1);
            std::vector<cv::Point> cells(obstacles.size());
            for (size_t i = 0; i < obstacles.size(); ++i) {
                cells[i] = cv::Point(static_cast<int>((obstacles[i].x - min_x) / cluster_resolution_),
                                     static_cast<int>((obstacles[i].y - min_y) / cluster_resolution_));
                occupied.at<uint8_t>(cells[i]) = 255;
            }
            cv::Mat labels;
            int n_labels = cv::connectedComponents(occupied, labels, 8, CV_32S);
            std::vector<std::vector<size_t>> clusters(n_labels);
            for (size_t i = 0; i < obstacles.size(); ++i) {
                clusters[labels.at<int>(cells[i])].push_back(i);
            }

            // 3. Look for a cylinder in each cluster (label 0 is the empty background). A cluster
            // can hold several objects (e.g. a cylinder against a wall), so the inliers of each
            // fit are removed and the search is repeated on the remaining points.
            for (int l = 1; l < n_labels; ++l) {
                std::vector<size_t> remaining = clusters[l];
                for (int attempt = 0; attempt < max_fits_per_cluster_; ++attempt) {
                    if (remaining.size() < static_cast<size_t>(min_inliers_)) {
                        break;
                    }
                    Circle circle;
                    std::vector<size_t> inliers;
                    if (!fitCircle(obstacles, remaining, circle, inliers)) {
                        break;
                    }
                    Cylinder cylinder;
                    if (isCylinder(obstacles, remaining, circle, inliers, sensor_x, sensor_y, cylinder)) {
                        detections.push_back(cylinder);
                        for (size_t i : inliers) pc_inliers.push_back(obstacles[i]);
                    }
                    // Both lists are sorted, since inliers are collected in the order of remaining
                    std::vector<size_t> outliers;
                    std::set_difference(remaining.begin(), remaining.end(), inliers.begin(), inliers.end(),
                            std::back_inserter(outliers));
                    remaining.swap(outliers);
                }
            }
            return detections;
        }

        // RANSAC on the (x,y) coordinates, followed by a least-squares refinement on the inliers
        bool fitCircle(const pcl::PointCloud<pcl::PointXYZ> & pts, const std::vector<size_t> & idx,
                Circle & best, std::vector<size_t> & inliers) {
            std::uniform_int_distribution<size_t> dsample(0, idx.size() - 1);
            size_t best_score = 0;
            for (int s = 0; s < n_samples_; ++s) {
                const pcl::PointXYZ & a = pts[idx[dsample(gen_)]];
                const pcl::PointXYZ & b = pts[idx[dsample(gen_)]];
                const pcl::PointXYZ & c = pts[idx[dsample(gen_)]];
                Circle candidate;
                if (!circleFrom3Points(a, b, c, candidate)) {
                    continue; // Collinear or repeated points
                }
                if (candidate.r < min_radius_ || candidate.r > max_radius_) {
                    continue;
                }
                size_t score = findInliers(pts, idx, candidate, nullptr);
                if (score > best_score) {
                    best_score = score;
                    best = candidate;
                }
            }
            if (best_score < static_cast<size_t>(min_inliers_)) {
                return false;
            }

            inliers.clear();
            findInliers(pts, idx, best, &inliers);
            Circle refined;
            std::vector<size_t> refined_inliers;
            if (leastSquaresCircle(pts, inliers, refined) &&
                    refined.r >= min_radius_ && refined.r <= max_radius_ &&
                    findInliers(pts, idx, refined, &refined_inliers) >= inliers.size()) {
                best = refined;
                inliers.swap(refined_inliers);
            }
            return true;
        }

        size_t findInliers(const pcl::PointCloud<pcl::PointXYZ> & pts, const std::vector<size_t> & idx,
                const Circle & c, std::vector<size_t> * inliers) const {
            size_t count = 0;
            for (size_t i : idx) {
                double dx = pts[i].x - c.cx;
                double dy = pts[i].y - c.cy;
                if (std::fabs(std::sqrt(dx * dx + dy * dy) - c.r) < tolerance_) {
                    count++;
                    if (inliers) {
                        inliers->push_back(i);
                    }
                }
            }
            return count;
        }

        // Circle through three points in the (x,y) plane. With a as the origin, the centre u
        // satisfies 2 u.b = |b|^2 and 2 u.c = |c|^2.
        static bool circleFrom3Points(const pcl::PointXYZ & a, const pcl::PointXYZ & b,
                const pcl::PointXYZ & c, Circle & out) {
            const double bx = static_cast<double>(b.x) - a.x, by = static_cast<double>(b.y) - a.y;
            const double cx = static_cast<double>(c.x) - a.x, cy = static_cast<double>(c.y) - a.y;
            const double det = 2.0 * (bx * cy - by * cx);
            if (std::fabs(det) < 1e-9) {
                return false;
            }
            const double b2 = bx * bx + by * by;
            const double c2 = cx * cx + cy * cy;
            const double ux = (cy * b2 - by * c2) / det;
            const double uy = (bx * c2 - cx * b2) / det;
            out.cx = a.x + ux;
            out.cy = a.y + uy;
            out.r = std::hypot(ux, uy);
            return true;
        }

        // Algebraic (Kasa) circle fit: x^2 + y^2 + D x + E y + F = 0 is linear in (D,E,F).
        // The coordinates are centred on their mean to keep the system well conditioned.
        static bool leastSquaresCircle(const pcl::PointCloud<pcl::PointXYZ> & pts,
                const std::vector<size_t> & idx, Circle & out) {
            if (idx.size() < 3) {
                return false;
            }
            double mx = 0.0, my = 0.0;
            for (size_t i : idx) {
                mx += pts[i].x;
                my += pts[i].y;
            }
            mx /= idx.size();
            my /= idx.size();

            Eigen::Matrix3d AtA = Eigen::Matrix3d::Zero();
            Eigen::Vector3d Atb = Eigen::Vector3d::Zero();
            for (size_t i : idx) {
                double x = pts[i].x - mx;
                double y = pts[i].y - my;
                Eigen::Vector3d a(x, y, 1.0);
                AtA += a * a.transpose();
                Atb -= a * (x * x + y * y);
            }
            Eigen::Vector3d X = AtA.ldlt().solve(Atb);

            // Centre (-D/2, -E/2), radius^2 = |centre|^2 - F
            double ux = -X(0) / 2.0;
            double uy = -X(1) / 2.0;
            double r2 = ux * ux + uy * uy - X(2);
            if (!std::isfinite(r2) || r2 <= 0.0) {
                return false;
            }
            out.cx = mx + ux;
            out.cy = my + uy;
            out.r = std::sqrt(r2);
            return true;
        }

        // Checks that a circle fitted on a cluster is the visible side of a vertical cylinder
        bool isCylinder(const pcl::PointCloud<pcl::PointXYZ> & pts, const std::vector<size_t> & cluster,
                const Circle & c, const std::vector<size_t> & inliers,
                double sensor_x, double sensor_y, Cylinder & out) const {
            size_t n_ring = 0;
            double disc_z_min = std::numeric_limits<double>::infinity();
            double disc_z_max = -std::numeric_limits<double>::infinity();
            for (size_t i : cluster) {
                double d = std::hypot(pts[i].x - c.cx, pts[i].y - c.cy);
                if (d > c.r - tolerance_ && d < c.r + support_margin_) {
                    n_ring++;
                }
                if (d < c.r + tolerance_) {
                    disc_z_min = std::min<double>(disc_z_min, pts[i].z);
                    disc_z_max = std::max<double>(disc_z_max, pts[i].z);
                }
            }

            std::vector<double> angles;
            angles.reserve(inliers.size());
            double mx = 0.0, my = 0.0;
            double z_min = pts[inliers[0]].z, z_max = z_min;
            for (size_t i : inliers) {
                angles.push_back(std::atan2(pts[i].y - c.cy, pts[i].x - c.cx));
                mx += pts[i].x;
                my += pts[i].y;
                z_min = std::min<double>(z_min, pts[i].z);
                z_max = std::max<double>(z_max, pts[i].z);
            }
            mx /= inliers.size();
            my /= inliers.size();

            // a. Nothing but the cylinder just outside the circle. The ring excludes the inside
            // of the disc, so a visible top cap does not count. Rejects walls and box corners,
            // which continue past the part that matches the circle.
            if (inliers.size() < min_inlier_ratio_ * n_ring) {
                return false;
            }

            // b. The inliers cover a wide enough arc. Rejects flat surfaces, which a large circle
            // can approximate over a short stretch.
            std::sort(angles.begin(), angles.end());
            double max_gap = 2 * M_PI - (angles.back() - angles.front());
            for (size_t k = 1; k < angles.size(); ++k) {
                max_gap = std::max(max_gap, angles[k] - angles[k-1]);
            }
            if (2 * M_PI - max_gap < min_arc_angle_) {
                return false;
            }

            // c. The sensor only sees the outside of a cylinder, so the axis is behind the observed
            // surface. Rejects concave shapes.
            if (std::hypot(c.cx - sensor_x, c.cy - sensor_y) < std::hypot(mx - sensor_x, my - sensor_y)) {
                return false;
            }

            // d. Same radius over the whole height of the object. Rejects spheres and cones,
            // where only a horizontal band matches the circle.
            if (z_max - z_min < min_height_ ||
                    z_max - z_min < min_height_coverage_ * (disc_z_max - disc_z_min)) {
                return false;
            }

            out.cx = c.cx;
            out.cy = c.cy;
            out.r = c.r;
            out.z_min = z_min;
            out.z_max = z_max;
            out.weight = inliers.size();
            out.n_obs = 1;
            return true;
        }

        // Merges a detection into the closest known cylinder, or adds it as a new one, and
        // returns its index. Each estimate is the average of its detections weighted by their
        // number of inliers, so close-range detections (more points, less noise) count more.
        size_t integrate(const Cylinder & d) {
            int closest = -1;
            double closest_dist = assoc_distance_;
            for (size_t k = 0; k < cylinders_.size(); ++k) {
                double dist = std::hypot(cylinders_[k].cx - d.cx, cylinders_[k].cy - d.cy);
                if (dist < closest_dist) {
                    closest = static_cast<int>(k);
                    closest_dist = dist;
                }
            }
            if (closest < 0) {
                cylinders_.push_back(d);
                return cylinders_.size() - 1;
            }

            Cylinder & c = cylinders_[closest];
            double w = c.weight + d.weight;
            c.cx = (c.weight * c.cx + d.weight * d.cx) / w;
            c.cy = (c.weight * c.cy + d.weight * d.cy) / w;
            c.r = (c.weight * c.r + d.weight * d.r) / w;
            c.z_min = std::min(c.z_min, d.z_min);
            c.z_max = std::max(c.z_max, d.z_max);
            c.weight = w;
            c.n_obs += 1;
            return closest;
        }

        void publishCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const std_msgs::msg::Header & header,
                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub) {
            sensor_msgs::msg::PointCloud2 out;
            pcl::toROSMsg(cloud, out);
            out.header.stamp = header.stamp;
            out.header.frame_id = target_frame_;
            pub->publish(out);
        }

        void publishMarkers() {
            visualization_msgs::msg::MarkerArray markers;
            rclcpp::Time now = this->get_clock()->now();
            for (size_t k = 0; k < cylinders_.size(); ++k) {
                const Cylinder & c = cylinders_[k];
                if (c.n_obs < min_observations_) {
                    continue; // Not confirmed yet
                }
                // The ground filter cuts the bottom floor_clearance of the cylinders
                double z_bottom = c.z_min - floor_clearance_;

                visualization_msgs::msg::Marker m;
                m.header.stamp = now;
                m.header.frame_id = target_frame_;
                m.ns = "cylinders";
                m.id = static_cast<int>(k);
                m.type = visualization_msgs::msg::Marker::CYLINDER;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = c.cx;
                m.pose.position.y = c.cy;
                m.pose.position.z = (z_bottom + c.z_max) / 2.0;
                m.pose.orientation.w = 1.0;
                m.scale.x = 2.0 * c.r;
                m.scale.y = 2.0 * c.r;
                m.scale.z = c.z_max - z_bottom;
                m.color.a = 0.8;
                m.color.r = 1.0;
                m.color.g = 0.5;
                m.color.b = 0.0;
                markers.markers.push_back(m);

                visualization_msgs::msg::Marker label = m;
                label.ns = "cylinder_labels";
                label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                label.pose.position.z = c.z_max + 0.15;
                label.scale.x = 0.0;
                label.scale.y = 0.0;
                label.scale.z = 0.12; // Text height
                label.color.r = 1.0;
                label.color.g = 1.0;
                label.color.b = 1.0;
                char text[64];
                snprintf(text, sizeof(text), "#%zu r=%.2f n=%d", k, c.r, c.n_obs);
                label.text = text;
                markers.markers.push_back(label);
            }
            marker_pub_->publish(markers);
        }

    public:
        CylinderDetector() : rclcpp::Node("cylinder_detector"), gen_(rd_()) {
            this->declare_parameter("target_frame", std::string("world"));
            this->declare_parameter("min_range", 0.4);
            this->declare_parameter("max_range", 3.5);
            this->declare_parameter("floor_clearance", 0.05);
            this->declare_parameter("max_height", 2.0);
            this->declare_parameter("voxel_size", 0.02);
            this->declare_parameter("cluster_resolution", 0.05);
            this->declare_parameter("n_samples", 200);
            this->declare_parameter("tolerance", 0.015);
            this->declare_parameter("min_radius", 0.05);
            this->declare_parameter("max_radius", 0.5);
            this->declare_parameter("min_inliers", 50);
            this->declare_parameter("max_fits_per_cluster", 3);
            this->declare_parameter("min_inlier_ratio", 0.7);
            this->declare_parameter("support_margin", 0.1);
            this->declare_parameter("min_arc_angle", M_PI / 2);
            this->declare_parameter("min_height_coverage", 0.8);
            this->declare_parameter("min_height", 0.1);
            this->declare_parameter("assoc_distance", 0.3);
            this->declare_parameter("min_observations", 3);

            target_frame_ = this->get_parameter("target_frame").as_string();
            min_range_ = this->get_parameter("min_range").as_double();
            max_range_ = this->get_parameter("max_range").as_double();
            floor_clearance_ = this->get_parameter("floor_clearance").as_double();
            max_height_ = this->get_parameter("max_height").as_double();
            voxel_size_ = this->get_parameter("voxel_size").as_double();
            cluster_resolution_ = this->get_parameter("cluster_resolution").as_double();
            n_samples_ = this->get_parameter("n_samples").as_int();
            tolerance_ = this->get_parameter("tolerance").as_double();
            min_radius_ = this->get_parameter("min_radius").as_double();
            max_radius_ = this->get_parameter("max_radius").as_double();
            min_inliers_ = std::max<int>(3, this->get_parameter("min_inliers").as_int());
            max_fits_per_cluster_ = this->get_parameter("max_fits_per_cluster").as_int();
            min_inlier_ratio_ = this->get_parameter("min_inlier_ratio").as_double();
            support_margin_ = this->get_parameter("support_margin").as_double();
            min_arc_angle_ = this->get_parameter("min_arc_angle").as_double();
            min_height_coverage_ = this->get_parameter("min_height_coverage").as_double();
            min_height_ = this->get_parameter("min_height").as_double();
            assoc_distance_ = this->get_parameter("assoc_distance").as_double();
            min_observations_ = this->get_parameter("min_observations").as_int();

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile();
            scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/points", qos,
                std::bind(&CylinderDetector::pointCloudCallback, this, std::placeholders::_1));

            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/cylinders", 1);
            obstacles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/obstacles", 1);
            inliers_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/inliers", 1);

            marker_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&CylinderDetector::publishMarkers, this));

            RCLCPP_INFO(this->get_logger(),
                        "CylinderDetector ready: radius in [%.2f, %.2f] m, tolerance %.3f m, %d RANSAC samples",
                        min_radius_, max_radius_, tolerance_, n_samples_);
        }

        virtual ~CylinderDetector() = default;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderDetector>());
    rclcpp::shutdown();
    return 0;
}
