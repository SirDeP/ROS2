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
#include "std_msgs/msg/int32.hpp"  

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
    subscriber_theanswer_ = this->create_subscription<std_msgs::msg::Int32>(
      "the_answer", 10,
      std::bind(&TemplateSubscriber::subscriber_theanswer_callback, this, _1));
  }


  //-- communication and timer functions 
  void subscriber_theanswer_callback(const std_msgs::msg::Int32::SharedPtr msg)
  { 
    /* your code */
    answer_to_all_    = msg->data; //gets the message data (field name data!)
    RCLCPP_INFO(this->get_logger() ,"Hello subscriber the answer is: %d",answer_to_all_); //example code  
   /* your code */
    
  }
  
  //--customs functions:
  //--custom variables: 
private:
  //--rclcpp variables:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_theanswer_;
  //--custom variables:
  int answer_to_all_  = 0;
  
};

int main(int argc, char * argv[])
{
  
rclcpp::init(argc, argv);

auto node = std::make_shared<TemplateSubscriber>();

rclcpp::spin(node);

rclcpp::shutdown();
return 0;
}
