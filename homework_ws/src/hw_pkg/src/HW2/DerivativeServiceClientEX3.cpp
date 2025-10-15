/* les2/service_client.cpp
 * Basic example of a service client node.
 *
 * Reviewed by: <x>
 * Changelog:

 */

#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "hw_interfaces/srv/derivative.hpp"

using namespace std::chrono;
using hw_interfaces::srv::Derivative;

class NodeLes2ServiceClient : public rclcpp::Node {
public:
	NodeLes2ServiceClient()
	: Node("node_les2_service_client")
	{
		client_ = this->create_client<Derivative>("derivativeserviceserver");
		timer_ = this->create_wall_timer(
			milliseconds(1000),
      std::bind(&NodeLes2ServiceClient::send_request, this)
    );
	}

private:
	void send_request() {
		if (!client_->wait_for_service(seconds(1))) {
			RCLCPP_WARN(this->get_logger(), "Service not available");
			return;
		}

		auto request = std::make_shared<Derivative::Request>();
    	request->t = rand()%10+1;
    	request->a = rand()%3+1;
		request->h = 1;
		request->n = 50;
		request->func_id = rand()%8+1;//func_id_;
		if (func_id_ < 8) {func_id_ += 1;}// else {func_id_ = 1;}

		auto future = client_->async_send_request(request,
			std::bind(&NodeLes2ServiceClient::handle_response, this, std::placeholders::_1)
    );
	}

	void handle_response(rclcpp::Client<Derivative>::SharedFuture future) {
		auto response = future.get();
		dfdt_list_.push_front(response->dfdt);
		if (dfdt_list_.size() >= 2) {
			RCLCPP_INFO(this->get_logger(), "Received dfdt1: %.6lf, recieved dfdt2: %.6lf", dfdt_list_.front(), dfdt_list_.back());
			RCLCPP_INFO(this->get_logger(), "Somregel: %.6lf", dfdt_list_.front() + dfdt_list_.back());
			dfdt_list_.clear();
		}
	}

	rclcpp::Client<Derivative>::SharedPtr client_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<Derivative::Request> request_temp_ = std::make_shared<Derivative::Request>();
	int64_t func_id_ = 1;
	std::list<double> dfdt_list_;
};

int main(int argc, char **argv) {
  srand(time(NULL));
  rclcpp::init(argc, argv);

	auto node = std::make_shared<NodeLes2ServiceClient>();
	rclcpp::spin(node);
	rclcpp::shutdown();

  return 0;
}
