/* les2/service_client.cpp
 * Basic example of a service client node.
 *
 * Reviewed by: <x>
 * Changelog:
 * [04-09-2025] Wessel T: Implement template
 * [11-09-2025] Wessel T: (BASE) Working service client
 */

#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "les_interfaces/srv/my_vec.hpp"

using namespace std::chrono;
using les_interfaces::srv::MyVec;

class NodeLes2ServiceClient : public rclcpp::Node {
public:
	NodeLes2ServiceClient()
	: Node("node_les2_service_client")
	{
		client_ = this->create_client<MyVec>("vectorserviceserver");
		timer_ = this->create_wall_timer(
			milliseconds(100),
      std::bind(&NodeLes2ServiceClient::send_request, this)
    );
	}

private:
	void send_request() {
		if (!client_->wait_for_service(seconds(1))) {
			RCLCPP_WARN(this->get_logger(), "Service not available");
			return;
		}

		auto request = std::make_shared<MyVec::Request>();
    request->vx = rand() % 100;
    request->vy = rand() % 100;
    request->vz = rand() % 100;

		auto future = client_->async_send_request(request,
			std::bind(&NodeLes2ServiceClient::handle_response, this, std::placeholders::_1)
    );
	}

	void handle_response(rclcpp::Client<MyVec>::SharedFuture future) {
		auto response = future.get();
		RCLCPP_INFO(this->get_logger(), "Received vec len: %.6lf", response->vl);
	}

	rclcpp::Client<MyVec>::SharedPtr client_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  srand(time(NULL));
  rclcpp::init(argc, argv);

	auto node = std::make_shared<NodeLes2ServiceClient>();
	rclcpp::spin(node);
	rclcpp::shutdown();

  return 0;
}
