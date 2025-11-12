#include "node/UsbCamSub.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<assignment::two::usb_cam_sub::UsbCamSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
