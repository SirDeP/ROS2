
/*
--Node description: 
...what the node is doing (functionally)... 
When node is created the node prints the answer to all every 1000 milliseconds   

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
#include "hw_interfaces/msg/name_number.hpp"

//--custom includes 
//#include "geometry_msgs/msg/point.hpp"      //header for the standard message that is used for communication



class HwPub_Test : public rclcpp::Node
{
	public:
	//-- constuctor: 
	HwPub_Test() : Node("HwPub_Test_node")
	{	
		//--communication and timer objects: 
		// publisher_hw_ = this->create_publisher<geometry_msgs::msg::Point>("the_answer",10);
		publisher_hw_ = this->create_publisher<hw_interfaces::msg::NameNumber>("hw",10);
		hw_theanswer_ = this->create_wall_timer(
 	    std::chrono::milliseconds(1000), std::bind(&HwPub_Test::hw_topic_hw_function, this));
		
		//--customs functions:
		

	}

	//-- communication and timer functions 
	void hw_topic_hw_function()
	{   

		/*your code where you publish*/
		// message_.x =  42 ;
		// message_.y =  32 ;
		// message_.z =  22 ;
		message_.name = "Vincent";
		message_.number = 34.5;
		publisher_hw_ ->publish(message_) ;
		/*your code where you publish*/
	}


	private:
		//--rclcpp variables:

		// rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_hw_;
		rclcpp::Publisher<hw_interfaces::msg::NameNumber>::SharedPtr publisher_hw_;
		rclcpp::TimerBase::SharedPtr hw_theanswer_ ;
		
		//--custom variables:
		// geometry_msgs::msg::Point message_ ;
		hw_interfaces::msg::NameNumber message_;
};


int main(int argc,char *argv[])
{
rclcpp::init(argc,argv)	;

auto node = std::make_shared<HwPub_Test>();


rclcpp::spin(node);

rclcpp::shutdown();

return 0;

}
