// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>

class FloorPlaneExtract: public rclcpp::Node {
    protected:
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        image_transport::Publisher image_pub_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

		using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,sensor_msgs::msg::Image>;
		using TS = message_filters::Synchronizer<SyncPolicy>;

        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> imgSub;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pclSub;
        std::shared_ptr<TS> sync;


        std::string base_frame_;
        std::string world_frame_;
        std::string outdir_;
        double max_range_;
        double height_threshold_;
        double min_displacement_;
        double min_rotation_;
        int thumb_size_;
        int max_image_per_type_;
        unsigned long image_counter_;
        unsigned long traversable_counter_;
        unsigned long untraversable_counter_;
        int joystick_button_;
        bool learning_;
        rclcpp::Time last_joy_time_;

        bool has_info;
        double fx,fy,cx,cy;
        geometry_msgs::msg::Pose2D last_pose;

        typedef enum {
            SATURATED,
            UNUSABLE,
            UNTRAVERSABLE,
            TRAVERSABLE
        } ThumbType;

        ThumbType check_thumb(const cv::Mat_<cv::Vec3b> & /*thumb: unused so far*/,
                const cv::Mat_<float> & thumb_z) {
            // TODO: Modify this function to take a decision on the traversability of the patch of ground 
            // corresponding to this image.
            for (int r=0;r<thumb_z.rows;r++) {
                for (int c=0;c<thumb_z.cols;c++) {
                    if (std::isnan(thumb_z(r,c))) {
                        // ignore this point, it has not been observed from the kinect
                        continue;
                    }
                }
            }
            return UNUSABLE;
        }


    protected: // ROS Callbacks
        void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
            if ((joystick_button_<int(msg->buttons.size())) && (msg->buttons[joystick_button_])) {
				rclcpp::Time now = this->get_clock()->now();
                if ((now-last_joy_time_).seconds()<0.5) {
                    // This is a bounce, ignore
                    return;
                }
                last_joy_time_ = now;
                learning_ = !learning_;
                if (learning_) {
                    RCLCPP_INFO(this->get_logger(),"Learning started, recording images and labels");
                } else {
                    RCLCPP_INFO(this->get_logger(),"Learning interruped");
                }
            }
        }


        void calibration_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            fx=msg->k[0]; cx=msg->k[2];
            fy=msg->k[4]; cy=msg->k[5];
            has_info = true;
        }

        void sync_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcl_msg, const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
            if (!learning_) {
                // If we're not learning, we don't care about this image
                return;
            }
            if (!has_info) return;

			std::string errStr; 
            geometry_msgs::msg::TransformStamped transformStamped;
            if (!tf_buffer->canTransform(base_frame_, world_frame_, pcl_msg->header.stamp,
                        rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                RCLCPP_ERROR(this->get_logger(),"Cannot transform current pose: %s",errStr.c_str());
                return;
            }
            transformStamped = tf_buffer->lookupTransform(base_frame_, world_frame_, pcl_msg->header.stamp);
            // Check if we moved
			geometry_msgs::msg::Pose2D new_pose;
			new_pose.x = transformStamped.transform.translation.x;
			new_pose.y = transformStamped.transform.translation.y;
			new_pose.theta = tf2::getYaw(transformStamped.transform.rotation);
            if ((hypot(last_pose.x-new_pose.x,last_pose.y-new_pose.y)<min_displacement_) &&
                    (fabs(remainder(last_pose.theta-new_pose.theta,2*M_PI))<min_rotation_)) {
                return;
            }
			last_pose = new_pose;
            

            cv::Mat img(cv_bridge::toCvShare(img_msg,"bgr8")->image);

            pcl::PointCloud<pcl::PointXYZ> pc_sensor, pc_baseframe;
            pcl::PCLPointCloud2 cloud2;
            pcl_conversions::toPCL(*pcl_msg,cloud2);    
            pcl::fromPCLPointCloud2(cloud2,pc_sensor);
            // Make sure the point cloud is in the base-frame
            if (pcl_msg->header.frame_id != base_frame_) {
                geometry_msgs::msg::TransformStamped transformStamped;
                try {
                    std::string errStr;
                    // This converts target in the grid frame.
                    if (!tf_buffer->canTransform(base_frame_, pcl_msg->header.frame_id, pcl_msg->header.stamp,
                                rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                        RCLCPP_ERROR(this->get_logger(),"Cannot transform target: %s",errStr.c_str());
                        return;
                    }
                    transformStamped = tf_buffer->lookupTransform(base_frame_, pcl_msg->header.frame_id, pcl_msg->header.stamp);
                    sensor_msgs::msg::PointCloud2 pc;
                    tf2::doTransform(*pcl_msg,pc,transformStamped);

                    // ROS2 Pointcloud2 to PCL Pointcloud2
                    pcl_conversions::toPCL(pc,cloud2);    
                } catch (const tf2::TransformException & ex){
                    RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
                }
            } else {
                // ROS2 Pointcloud2 to PCL Pointcloud2
                pcl_conversions::toPCL(*pcl_msg,cloud2);    
            }
            // PCL Pointcloud2 to templated form
            pcl::fromPCLPointCloud2(cloud2,pc_baseframe);



            cv::Mat_<float> img_z(img.size(),NAN);


            // First build an image of ground height
            unsigned int n = pc_sensor.size();
            for (unsigned int i=0;i<n;i++) {
                float x = pc_sensor[i].x;
                float y = pc_sensor[i].y;
                float z = pc_sensor[i].z;
                if (z > max_range_) {
                    // Ignoring points too far out.
                    continue;
                }
                int ix = round(cx + x*fx/z); // Sign error?, replace
                int iy = round(cy + y*fy/z); // plus with minus
                if ((ix < 0) || (ix >= img_z.cols) || (iy < 0) || (iy >= img_z.rows)) {
                    // Outside of the image. This is not possible, but may
                    // happen due to numerical uncertainties.
                    continue;
                }
                img_z(iy,ix) = pc_baseframe[i].z;
            }

            for (int r=0;r<img.rows;r+=thumb_size_) {
                if (r+thumb_size_>img.rows) { continue; }
                for (int c=0;c<img.cols;c+=thumb_size_) {
                    // printf("r %d c %d\n",r,c);
                    if (c+thumb_size_>img.cols) { continue; }
                    cv::Rect roi(c,r,thumb_size_,thumb_size_);
                    cv::Mat thumb = img(roi);
                    cv::Mat_<float> thumb_z = img_z(roi);
                    ThumbType type = check_thumb(thumb,thumb_z);
                    // printf("type : %d\n",type);
                    if ((type == TRAVERSABLE) && (traversable_counter_>=(unsigned)max_image_per_type_)) {
                        // We have enough images of this type
                        type = SATURATED;
                    }
                    if ((type == UNTRAVERSABLE) && (untraversable_counter_>=(unsigned)max_image_per_type_)) {
                        // We have enough images of this type
                        type = SATURATED;
                    }
                    if ((type != UNUSABLE) && (type != SATURATED)) { 
                        // Old fashion formatting
                        char dirname[1024],filename[1024],labelname[1024];
                        sprintf(dirname,"%s/%04ld",outdir_.c_str(),image_counter_/1000);
                        mkdir(dirname,0700);// may already exist but it is OK
                        sprintf(filename,"%s/%04ld/%04ld.png",outdir_.c_str(),image_counter_/1000,image_counter_%1000);
                        cv::imwrite(filename,thumb);
                        sprintf(labelname,"%s/labels.txt",outdir_.c_str());
                        FILE * fp = fopen(labelname,"a");
                        fprintf(fp,"%04ld/%04ld.png %d\n",image_counter_/1000,image_counter_%1000,(type==TRAVERSABLE)?1:0);
                        fclose(fp);
                        image_counter_ ++ ;
                        if (type == TRAVERSABLE) {
                            traversable_counter_ ++;
                        } else {
                            untraversable_counter_ ++;
                        }
                    }
                    // Now for display (could be disabled to save CPU)
                    switch (type) {
                        case UNTRAVERSABLE: 
                            for (int tr=0;tr<thumb.rows;tr++) {
                                for (int tc=0;tc<thumb.cols;tc++) {
                                    thumb.at<cv::Vec3b>(tr,tc)[2] = 255;
                                }
                            }
                            break;
                        case TRAVERSABLE: 
                            for (int tr=0;tr<thumb.rows;tr++) {
                                for (int tc=0;tc<thumb.cols;tc++) {
                                    thumb.at<cv::Vec3b>(tr,tc)[1] = 255;
                                }
                            }
                            break;
                        case UNUSABLE: 
                            for (int tr=0;tr<thumb.rows;tr++) {
                                for (int tc=0;tc<thumb.cols;tc++) {
                                    thumb.at<cv::Vec3b>(tr,tc)[0] = 255;
                                }
                            }
                            break;
                        default:
                            for (int tr=0;tr<thumb.rows;tr++) {
                                for (int tc=0;tc<thumb.cols;tc++) {
                                    thumb.at<cv::Vec3b>(tr,tc)[1] = 255;
                                    thumb.at<cv::Vec3b>(tr,tc)[2] = 255;
                                }
                            }
                            break;
                    }
                    // getchar();
                }
            }
            RCLCPP_INFO(this->get_logger(),"Image counter at %ld (%ld / %ld)",image_counter_,traversable_counter_,untraversable_counter_);
            
            cv_bridge::CvImage br(img_msg->header,"bgr8",img);
            image_pub_.publish(br.toImageMsg());
        }

    public:
        FloorPlaneExtract() : rclcpp::Node("floor_plane_extract"){
            learning_ = true;
            has_info = false;
			this->declare_parameter<std::string>("image_transport", "raw");
            this->declare_parameter("~/base_frame",std::string("body"));
            this->declare_parameter("~/world_frame",std::string("world"));
            this->declare_parameter("~/max_range",5.0);
            this->declare_parameter("~/thumb_size",32);
            this->declare_parameter("~/out_dir",std::string("."));
            this->declare_parameter("~/height_threshold",0.02);
            this->declare_parameter("~/min_displacement",0.1);
            this->declare_parameter("~/min_rotation",0.1);
            this->declare_parameter("~/max_image_per_type",1000);
            this->declare_parameter("~/joystick_button",3);
            base_frame_ = this->get_parameter("~/base_frame").as_string();
            world_frame_ = this->get_parameter("~/world_frame").as_string();
            max_range_ = this->get_parameter("~/max_range").as_double();
            thumb_size_ = this->get_parameter("~/thumb_size").as_int();
            outdir_ = this->get_parameter("~/out_dir").as_string();
            height_threshold_ = this->get_parameter("~/height_threshold").as_double();
            min_displacement_ = this->get_parameter("~/min_displacement").as_double();
            min_rotation_ = this->get_parameter("~/min_rotation").as_double();
            max_image_per_type_ = this->get_parameter("~/max_image_per_type").as_int();
            joystick_button_ = this->get_parameter("~/joystick_button").as_int();

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            // Reset label file
            char labelname[1024];
            sprintf(labelname,"%s/labels.txt",outdir_.c_str());
            FILE * fp = fopen(labelname,"w");
            if (!fp) {
                RCLCPP_ERROR(this->get_logger(),"Cannot open label file %s",labelname);
                assert(fp != NULL);
                return;
            }
            fclose (fp);

            image_counter_ = 0;
            traversable_counter_ = 0;
            untraversable_counter_ = 0;
            last_joy_time_ = this->get_clock()->now();

            // Make sure TF is ready
			std::string errStr; 
            geometry_msgs::msg::TransformStamped transformStamped;
            if (!tf_buffer->canTransform(base_frame_, world_frame_, rclcpp::Time(0),
                        rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                RCLCPP_ERROR(this->get_logger(),"Cannot transform current pose: %s",errStr.c_str());
                return;
            }
            transformStamped = tf_buffer->lookupTransform(base_frame_, world_frame_, rclcpp::Time(0));
            // Check if we moved
			last_pose.x = transformStamped.transform.translation.x;
			last_pose.y = transformStamped.transform.translation.y;
			last_pose.theta = tf2::getYaw(transformStamped.transform.rotation);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

            image_pub_ = image_transport::create_publisher(this,"~/image_label",rmw_qos_profile_sensor_data);
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("~/joy",1,std::bind(&FloorPlaneExtract::joy_callback,this,std::placeholders::_1));
            info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("~/info",1,std::bind(&FloorPlaneExtract::calibration_callback,this,std::placeholders::_1));

            // Created a synchronized subscriber for point cloud and image. The
            // need for pointer is suspicious, but it works this way.
            imgSub.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(this,"~/image",qos));
            pclSub.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(this,"~/pointcloud",qos));
            sync.reset(new TS(SyncPolicy(20),*pclSub,*imgSub));
            sync->registerCallback(std::bind(&FloorPlaneExtract::sync_callback,this,
						std::placeholders::_1, std::placeholders::_2));

        }

};

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloorPlaneExtract>());
    rclcpp::shutdown();
    return 0;
}





