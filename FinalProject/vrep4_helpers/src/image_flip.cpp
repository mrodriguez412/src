#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>

using std::placeholders::_1;
using std::placeholders::_2;


class ImageFlipper : public rclcpp::Node
{
    public:
        ImageFlipper() : rclcpp::Node("image_flipper")
        {
            rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
            publisher_ = image_transport::create_publisher(this, "image_flipped", custom_qos);
            subscription_ = image_transport::create_subscription(this, "image",
                    std::bind(&ImageFlipper::image_callback, this, std::placeholders::_1), "raw", custom_qos);
        }

    protected:
        void image_callback(const image_transport::ImageTransport::ImageConstPtr&  msg) 
        {
            cv::Mat output;
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            cv::flip(cv_ptr->image, output,0);
            sensor_msgs::msg::Image::SharedPtr I = cv_bridge::CvImage(msg->header, msg->encoding, output).toImageMsg();

            I->is_bigendian = msg->is_bigendian;
            publisher_.publish(I);
        }
        image_transport::Publisher publisher_;
        image_transport::Subscriber subscription_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageFlipper>());
    rclcpp::shutdown();
    return 0;
}
