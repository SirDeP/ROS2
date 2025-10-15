/*
--Node description: 
...what the node is doing (functionally)... 
When node is created it wil receive all messages with the topic "the_answer"
and wil print it on the terminal using Logger 
*/ 

/*
--Software changes:
one line per change 
(1) created 31.3.2025: developer-Tilmann Koster reviewer(s)-Niek Ottens 
(2) changed 01.4.2025: xxx functionality added ... : developer-Tilmann Koster reviewer(s)-Niek Ottens 
...
*/

//-- tester: Sander Gieling


//--general includes 
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"

//--custom includes 
//#include "geometry_msgs/msg/point.hpp"  
#include "les_interfaces/msg/hardware_status.hpp"

//--using 
using namespace std::placeholders;

//--Node class 
class TemplateSubscriber : public rclcpp::Node
{
  public:
  //-- constuctor: 
  TemplateSubscriber(): Node("templatesubscriber_node")
  {
     //--communication and timer objects: 
    //subscriber_theanswer_ = this->create_subscription<geometry_msgs::msg::Point>(
      //"the_answer", 10,
      subscriber_theanswer_ = this->create_subscription<les_interfaces::msg::HardwareStatus>(
      "the_answer", 10,
      std::bind(&TemplateSubscriber::subscriber_theanswer_callback, this, _1));
  }


  //-- communication and timer functions 
  void subscriber_theanswer_callback(const les_interfaces::msg::HardwareStatus::SharedPtr msg)
  { 
    /* your code */
    // x_    = msg->x; //gets the message data (field name data!)
    // y_    = msg->y;
    // z_    = msg->z;
    temp_ = msg->temperature;
    RCLCPP_INFO(this->get_logger() ,"temp=%d, ",temp_); //example code  
   /* your code */
    
  }
  
  //--customs functions:
  //--custom variables: 
private:
  //--rclcpp variables:
  rclcpp::Subscription<les_interfaces::msg::HardwareStatus>::SharedPtr subscriber_theanswer_;
  //--custom variables:
  int temp_  = 0;
  
};

int main(int argc, char * argv[])
{
  
rclcpp::init(argc, argv);

auto node = std::make_shared<TemplateSubscriber>();

rclcpp::spin(node);

rclcpp::shutdown();
return 0;
}
