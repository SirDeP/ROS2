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
#include "hw_interfaces/msg/name_number.hpp"

//--using 
using namespace std::placeholders;

//--Node class 
class HwSub_Test : public rclcpp::Node
{
  public:
  //-- constuctor: 
  HwSub_Test(): Node("HwSub_Test_node")
  {
     //--communication and timer objects: 
    //subscriber_hw_ = this->create_subscription<geometry_msgs::msg::Point>(
      //"the_answer", 10,
      subscriber_hw_ = this->create_subscription<hw_interfaces::msg::NameNumber>(
      "hw", 10,
      std::bind(&HwSub_Test::subscriber_hw_callback, this, _1));
  }


  //-- communication and timer functions 
  void subscriber_hw_callback(const hw_interfaces::msg::NameNumber::SharedPtr msg)
  { 
    /* your code */
    // x_    = msg->x; //gets the message data (field name data!)
    // y_    = msg->y;
    // z_    = msg->z;

    RCLCPP_INFO(this->get_logger() ,"name=%s, number=%d", msg->name.c_str(), msg->number); //example code  
   /* your code */
    
  }
  
  //--customs functions:
  //--custom variables: 
private:
  //--rclcpp variables:
  rclcpp::Subscription<hw_interfaces::msg::NameNumber>::SharedPtr subscriber_hw_;
  //--custom variables:
  
};

int main(int argc, char * argv[])
{
  
rclcpp::init(argc, argv);

auto node = std::make_shared<HwSub_Test>();

rclcpp::spin(node);

rclcpp::shutdown();
return 0;
}
