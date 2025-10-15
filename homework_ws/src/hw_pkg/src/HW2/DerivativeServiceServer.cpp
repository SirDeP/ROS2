/*
Node description: what is the node doing , what are the node objects used
*/

/*
Software changes (one line by change):
(1) 31.3.2025 created by Tilmann Koster
(2) 1.4.2025 xxx functionality added ... by Tilmann Koster
...
*/
#define MIN 0
#define MAX 50
//--general includes
#include "rclcpp/rclcpp.hpp" // altijd nodig
// #include <cmath>
//--custom includes
#include "hw_interfaces/srv/derivative.hpp" // your own service interface =small letter, CAPs is translated to _

//--using
using Derivative = hw_interfaces::srv::Derivative;

using namespace std::placeholders;

//--Node class
class DerivativeServiceServer : public rclcpp::Node
{
public:
  //-- constuctor:
  DerivativeServiceServer() : Node("derivativeserviceserver_node")
  {
    //--communication and timer objects:
    template_serviceserver_ = this->create_service<Derivative>("derivativeserviceserver",
                                                               std::bind(&DerivativeServiceServer::callBackTemplateS, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Servcie Server started");
    //--customs functions:
  }

  //-- communication and timer functions
  void callBackTemplateS(const Derivative::Request::SharedPtr request, const Derivative::Response::SharedPtr response)
  {
    // your code
    // response->vl = sqrt((pow(request->vx, 2) + pow(request->vy, 2) + pow(request->vz, 2)));
    if (request->h > 50) {
      h_ = 50;
    } else if (request->h < 0.0) {
      h_ = 0;
    } else {
      h_ = request->h;
    }
    
    for (long int i = 0; i < request->n; i++)
    {
      response->dfdt = calculateFunctions(h_, request->t, request->a, request->func_id);
      // RCLCPP_INFO(this->get_logger(), "the answer is: %lf at h value of %.8f", response->dfdt, h_);
      h_ = h_ * 0.5;
    }

    // response->dfdt = (pow((request->t+request->h), 2) - pow(request->t, 2 )) / request->h;

    // your code
  };

  double calculateFunctions(double h, double t, double a, long int ID)
  {
    switch (ID)
    {
    case 1:
      answer_ = (pow((t + h), a) - pow(t, a)) / h;
      break;

    case 2:
      answer_ = ((1 / (pow((t + h), a))) - 1 / pow(t, a)) / h;
      break;
    case 3:
      answer_ = (exp(t + h) - exp(t)) / h;
      break;
    case 4:
      answer_ = (pow(a, t + h) - pow(a, t)) / h;
      break;
    case 5:
      answer_ = (sin(t + h) - sin(t)) / h;
      break;
    case 6:
      answer_ = (cos(t + h) - cos(t)) / h;
      break;
    case 7:
      answer_ = (log(t + h) - log(t)) / h;
      break;
    case 8: // log(5) / log 10 = log^10(5)
      answer_ = ((log(t + h) / log(a)) - (log(t) / log(a))) / h;
      break;
    default:
      break;
    }

    return answer_;
  }
  //--customs functions:
  //...

private:
  //--rclcpp variables:
  rclcpp::Service<Derivative>::SharedPtr template_serviceserver_;

  //--custom variables:
  double h_;
  double answer_;
};

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  auto node = std::make_shared<DerivativeServiceServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
