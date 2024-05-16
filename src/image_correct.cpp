#include "video_utils.hpp"

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>


class ImageCorrectNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_subscription_ = nullptr;  
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_publisher_ = nullptr; 

    // rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr hbmem_subscription_ = nullptr; 
    rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr hbmem_publisher_ = nullptr;
    std::string sub_image_topic_ = "/image";
    std::string pub_image_topic_ = "/image_correct";
    bool is_shared_mem_sub_ = true;

    uint8_t *mPtrIn = nullptr;
public:

    ImageCorrectNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()): rclcpp::Node(node_name, options) {

        this->declare_parameter<std::string>("sub_img_topic", sub_image_topic_);
        this->declare_parameter<std::string>("pub_image_topic", pub_image_topic_);
        this->declare_parameter<bool>("is_shared_mem_sub", is_shared_mem_sub_);
        
        this->get_parameter<std::string>("sub_img_topic", sub_image_topic_);
        this->get_parameter<std::string>("pub_image_topic", pub_image_topic_);
        this->get_parameter<bool>("is_shared_mem_sub", is_shared_mem_sub_);

        ros_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            sub_image_topic_, 1,
            std::bind(&ImageCorrectNode::in_ros_topic_cb, this, std::placeholders::_1));
        // ros_publisher_ =
        //     this->create_publisher<sensor_msgs::msg::Image>(pub_image_topic_, 10);


        // hbmem_subscription_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        //     sub_image_topic_, rclcpp::SensorDataQoS(),
        //     std::bind(&ImageCorrectNode::in_hbmem_topic_cb, this, std::placeholders::_1));
        hbmem_publisher_ = this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
                pub_image_topic_, 1);

    }

    void correct(cv::Mat &image, cv::Mat &correct_result){
        int total_time = 0;
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 544.81663, 0.000000, 341.09116, 0.000000, 547.40127, 163.93764, 0.000000, 0.000000, 1.000000);
        cv::Mat distortionCoefficients = (cv::Mat_<double>(1, 5) << -0.471083, 0.184250, -0.000296, 0.001877, 0.000000);
        cv::undistort(image, correct_result, cameraMatrix, distortionCoefficients);
    }

    void in_ros_topic_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg){
        sensor_msgs::msg::Image img_msg;
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat correct_result;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        correct(cv_ptr->image, correct_result);

        // std_msgs::msg::Header header;
        // header.stamp = msg->header.stamp;
        // cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, "bgr8", correct_result);
        // img_bridge.toImageMsg(img_msg);
        // ros_publisher_->publish(img_msg);

        if (nullptr == mPtrIn) {
            mPtrIn = new uint8_t[msg->width * msg->height * 3 / 2];
        }
        video_utils::BGR24_to_NV12(correct_result.ptr<uint8_t>(), mPtrIn, msg->width, msg->height);

        auto loanedMsg = hbmem_publisher_->borrow_loaned_message();
        if (loanedMsg.is_valid()) {
            auto& pub_msg = loanedMsg.get();
            pub_msg.time_stamp.sec = msg->header.stamp.sec;
            pub_msg.time_stamp.nanosec = msg->header.stamp.nanosec;
            pub_msg.height = msg->height;
            pub_msg.width = msg->width;
            pub_msg.step = msg->width;
            pub_msg.data_size = msg->width * msg->height * 3 / 2;
            memcpy(pub_msg.encoding.data(), "nv12", strlen("nv12"));
            memcpy(&pub_msg.data[0], mPtrIn, msg->width * msg->height * 3 / 2);
            hbmem_publisher_->publish(std::move(loanedMsg));
        }


    }

    ~ImageCorrectNode(){
        if (mPtrIn) {
            delete[] mPtrIn;
            mPtrIn = nullptr;
        }
    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ImageCorrectNode>("ImageCorrectNode"));

    rclcpp::shutdown();

    RCLCPP_WARN(rclcpp::get_logger("ImageCorrectNode"), "Pkg exit");

    return 0;
}