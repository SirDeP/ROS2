#include "UsbCamSub.hpp"

namespace assignment::two::usb_cam_sub
{

UsbCamSub::UsbCamSub() : Node("usb_cam_reader_node")
{
    this->declare_parameter<int>("edge_detection_choice", 1);
    this->declare_parameter<int>("circle_radius", 50);
    this->declare_parameter<int>("circle_thickness", 10);
    this->declare_parameter("circle_color", std::vector<long>{0, 0, 255});

    edge_detection_choice_  = this->get_parameter("edge_detection_choice").as_int();
    circle_radius_          = this->get_parameter("circle_radius").as_int();
    circle_thickness_       = this->get_parameter("circle_thickness").as_int();

    // Get color parameter and convert from vector<long> to vector<uint8_t>
    std::vector<long> color_param = this->get_parameter("circle_color").as_integer_array();
    if (color_param.size() == 3) {
        circle_color_ = {static_cast<uint8_t>(color_param[0]),
                        static_cast<uint8_t>(color_param[1]),
                        static_cast<uint8_t>(color_param[2])};
        RCLCPP_INFO(this->get_logger(), "Circle color set to [%ld, %ld, %ld]",
                    color_param[0], color_param[1], color_param[2]);
    } else {
        circle_color_ = {0, 0, 255}; // Default red in BGR
        RCLCPP_WARN(this->get_logger(), "Invalid circle_color size, using default [0, 0, 255]");
    }

    subscriber_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 5,
      std::bind(&UsbCamSub::subscriber_camera_callback, this, std::placeholders::_1));
    publisher_camera_ = this->create_publisher<sensor_msgs::msg::Image>("gray_image", 5);
}

void UsbCamSub::subscriber_camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Convert ROS image message to OpenCV image
    try
    {
        cv_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
    }

    // Run image processing
    image_processing();
}

void UsbCamSub::image_processing()
{
    try
    {
        // divide into 4 parts
        cv::Mat top_left = cv_frame_(cv::Rect(0, 0, cv_frame_.cols / 2, cv_frame_.rows / 2));
        cv::Mat top_right = cv_frame_(cv::Rect(cv_frame_.cols / 2, 0, cv_frame_.cols / 2, cv_frame_.rows / 2));
        cv::Mat bottom_left = cv_frame_(cv::Rect(0, cv_frame_.rows / 2, cv_frame_.cols / 2, cv_frame_.rows / 2));
        cv::Mat bottom_right = cv_frame_(cv::Rect(cv_frame_.cols / 2, cv_frame_.rows / 2, cv_frame_.cols / 2, cv_frame_.rows / 2));

        // fill top left with white
        top_left.setTo(cv::Scalar(255, 255, 255));

        // add a circle to top right
        cv::circle(top_right, cv::Point(top_right.cols / 2, top_right.rows / 2), circle_radius_, cv::Scalar(circle_color_[0], circle_color_[1], circle_color_[2]), circle_thickness_);

        // bottom left set b of bgr to 0
        std::vector<cv::Mat> channels;
        cv::split(bottom_left, channels);
        channels[0] = cv::Mat::zeros(bottom_left.rows, bottom_left.cols,
                                        channels[0].type());
        cv::merge(channels, bottom_left);


        switch (edge_detection_choice_)
        {
            case 1: {
                // bottom right use sobel edge detection
                cv::Mat gray, grad_x, grad_y, abs_grad_x, abs_grad_y;
                cv::cvtColor(bottom_right, gray, cv::COLOR_BGR2GRAY);
                cv::Sobel(gray, grad_x, CV_64F, 1, 0, 3);
                cv::Sobel(gray, grad_y, CV_64F, 0, 1, 3);
                cv::convertScaleAbs(grad_x, abs_grad_x);
                cv::convertScaleAbs(grad_y, abs_grad_y);
                cv::Mat edge;
                cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edge);
                cv::cvtColor(edge, bottom_right, cv::COLOR_GRAY2BGR);
                break;
            }
            case 2: {
                // bottom right use canny edge detection
                cv::Mat gray_canny, edges;
                cv::cvtColor(bottom_right, gray_canny, cv::COLOR_BGR2GRAY);
                cv::Canny(gray_canny, edges, 100, 200);
                cv::cvtColor(edges, bottom_right, cv::COLOR_GRAY2BGR);
                break;
            }
            case 3: {
                // bottom right use laplacian edge detection
                cv::Mat gray_lap, laplacian, abs_laplacian;
                cv::cvtColor(bottom_right, gray_lap, cv::COLOR_BGR2GRAY);
                cv::Laplacian(gray_lap, laplacian, CV_64F);
                cv::convertScaleAbs(laplacian, abs_laplacian);
                cv::cvtColor(abs_laplacian, bottom_right, cv::COLOR_GRAY2BGR);
                break;
            }
            default:
                break;
        }

        // merge back together into one image with division lines
        cv::Mat top_half, bottom_half;

        cv::hconcat(top_left, top_right, top_half);
        cv::hconcat(bottom_left, bottom_right, bottom_half);
        cv::vconcat(top_half, bottom_half, transformed_frame_);

        // Draw divide 4 lines
        cv::line(transformed_frame_, cv::Point(0, transformed_frame_.rows / 2), cv::Point(transformed_frame_.cols, transformed_frame_.rows / 2), cv::Scalar(255, 0, 255), 5);
        cv::line(transformed_frame_, cv::Point(transformed_frame_.cols / 2, 0), cv::Point(transformed_frame_.cols / 2, transformed_frame_.rows), cv::Scalar(255, 0, 255), 5);

        // Convert processed OpenCV image back to ROS image message
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "transformed_image";

        auto processed_msg = cv_bridge::CvImage(header, "bgr8", transformed_frame_).toImageMsg();
        publisher_camera_->publish(*processed_msg);
    } catch (cv_bridge::Exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


} // namespace assignment::two::usb_cam_sub
