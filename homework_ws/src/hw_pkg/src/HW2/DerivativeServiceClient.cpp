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
    	request->t = 1;
    	request->a = 2;
		request->h = 1;
		request->n = 50;
		request->func_id = func_id_;
		if (func_id_ < 8) {func_id_ += 1;} else {func_id_ = 1;}
		// expected_dfdt_ = calculateExpected(request->t, request->a, func_id_);
		request_temp_  = request;
		auto future = client_->async_send_request(request,
			std::bind(&NodeLes2ServiceClient::handle_response, this, std::placeholders::_1)
    );
	}

	void handle_response(rclcpp::Client<Derivative>::SharedFuture future) {
		auto response = future.get();
		double expected_dfdt = calculateExpected(request_temp_->t, request_temp_->a, request_temp_->func_id);
		RCLCPP_INFO(this->get_logger(), "Received dfdt len: %.6lf, expected dfdt: %.6lf, deviation: %.6lf", response->dfdt, expected_dfdt, response->dfdt-expected_dfdt);
	}

	double calculateExpected(double t, double a, int64_t ID) {
		double expected_;
		switch (ID)
		{
		case 1:
			expected_ = (a * pow(t, a-1));
			break;
		case 2:
			expected_ = ((-a) / pow(t, a+1));
			break;
		case 3:
			expected_ = exp(t);
			break;
		case 4:
			expected_ = (pow(a, t) * log(a));
			break;
		case 5:
			expected_ = cos(t);
			break;
		case 6:
			expected_ = -sin(t);
			break;
		case 7:
			expected_ = 1 / t;
			break;
		case 8:
			expected_ = 1 / (t * log(a));
		default:
			break;
		}
		return expected_;
	}

	rclcpp::Client<Derivative>::SharedPtr client_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<Derivative::Request> request_temp_ = std::make_shared<Derivative::Request>();
	int64_t func_id_ = 1;
};

int main(int argc, char **argv) {
  srand(time(NULL));
  rclcpp::init(argc, argv);

	auto node = std::make_shared<NodeLes2ServiceClient>();
	rclcpp::spin(node);
	rclcpp::shutdown();

  return 0;
}
