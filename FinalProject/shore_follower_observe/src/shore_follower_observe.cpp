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


#include <sys/stat.h>
#include <sys/types.h>


#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>

class ShoreFollowerObserve: public rclcpp::Node {
    protected:
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        image_transport::Subscriber image_sub_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;


        std::string base_frame_;
        std::string world_frame_;
        std::string outdir_;
        double rotation_threshold_;
        double min_displacement_;
        double min_rotation_;
        int max_image_per_type_;
        unsigned long image_counter_;
        unsigned long type_counter_[3]; // left, straight, right
        int joystick_button_;
        bool learning_;

        geometry_msgs::msg::Twist last_command_;
        rclcpp::Time last_joy_time_;
        rclcpp::Time last_command_time_;

        geometry_msgs::msg::Pose2D last_pose;


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

        void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
            last_command_time_ = this->get_clock()->now();
            last_command_ = *msg;
        }

        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg) {
            if (!learning_) {
                // If we're not learning, we don't care about this image
                return;
            }

            if ((rclcpp::Time(img_msg->header.stamp)-last_command_time_).seconds()>0.1) {
                // We can't accept an old command
                return;
            }

			std::string errStr; 
            geometry_msgs::msg::TransformStamped transformStamped;
            if (!tf_buffer->canTransform(base_frame_, world_frame_, img_msg->header.stamp,
                        rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                RCLCPP_ERROR(this->get_logger(),"Cannot transform current pose: %s",errStr.c_str());
                return;
            }
            transformStamped = tf_buffer->lookupTransform(base_frame_, world_frame_, img_msg->header.stamp);
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
            bool save_it = true;
            int label = 1;
            if (last_command_.angular.z<-rotation_threshold_) {
                label = 0;
            } else if (last_command_.angular.z>rotation_threshold_) {
                label = 2;
            }
            save_it = (type_counter_[label] < (unsigned)max_image_per_type_);
            if (save_it) {
                char dirname[1024],filename[1024],labelname[1024];
                sprintf(dirname,"%s/%04ld",outdir_.c_str(),image_counter_/1000);
                mkdir(dirname,0700);// may already exist but it is OK
                sprintf(filename,"%s/%04ld/%04ld.png",outdir_.c_str(),image_counter_/1000,image_counter_%1000);
                cv::imwrite(filename,img);
                sprintf(labelname,"%s/labels.txt",outdir_.c_str());
                FILE * fp = fopen(labelname,"a");
                fprintf(fp,"%04ld/%04ld.png %d\n",image_counter_/1000,image_counter_%1000,label);
                fclose(fp);
                type_counter_[label]++;
                image_counter_++;
            }
            RCLCPP_INFO(this->get_logger(),"Image counter at %ld (%ld %ld %ld)",image_counter_,
                    type_counter_[0],type_counter_[1],type_counter_[2]);
        }

    public:
        ShoreFollowerObserve() : rclcpp::Node("shore_follower_observe"){
            learning_ = true;
			this->declare_parameter<std::string>("~/image_transport", "raw");
            this->declare_parameter("~/base_frame",std::string("body"));
            this->declare_parameter("~/world_frame",std::string("world"));
            this->declare_parameter("~/out_dir",std::string("."));
            this->declare_parameter("~/rotation_threshold",0.3);
            this->declare_parameter("~/min_displacement",0.1);
            this->declare_parameter("~/min_rotation",0.1);
            this->declare_parameter("~/max_image_per_type",1000);
            this->declare_parameter("~/joystick_button",3);
            base_frame_ = this->get_parameter("~/base_frame").as_string();
            world_frame_ = this->get_parameter("~/world_frame").as_string();
            outdir_ = this->get_parameter("~/out_dir").as_string();
            rotation_threshold_ = this->get_parameter("~/rotation_threshold").as_double();
            min_displacement_ = this->get_parameter("~/min_displacement").as_double();
            min_rotation_ = this->get_parameter("~/min_rotation").as_double();
            max_image_per_type_ = this->get_parameter("~/max_image_per_type").as_int();
            joystick_button_ = this->get_parameter("~/joystick_button").as_int();
            std::string transport;
            transport = this->get_parameter("~/image_transport").as_string();

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
            type_counter_[0] = type_counter_[1] = type_counter_[2] = 0;
            last_command_time_ = last_joy_time_ = this->get_clock()->now();

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

            image_sub_ = image_transport::create_subscription(this,"~/image",
                    std::bind(&ShoreFollowerObserve::image_callback,this,std::placeholders::_1),
                    transport, rmw_qos_profile_sensor_data);
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("~/joy",1,std::bind(&ShoreFollowerObserve::joy_callback,this,std::placeholders::_1));
            twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("~/twist",1,std::bind(&ShoreFollowerObserve::twist_callback,this,std::placeholders::_1));


        }

};

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShoreFollowerObserve>());
    rclcpp::shutdown();
    return 0;
}





