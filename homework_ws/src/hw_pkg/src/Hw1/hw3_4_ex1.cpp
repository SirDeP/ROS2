
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

//--custom includes 
#include "std_msgs/msg/int32.hpp"      //header for the standard message that is used for communication



class HwPub : public rclcpp::Node
{
	public:
	//-- constuctor: 
	HwPub() : Node("HwPub_node")
	{	
		//--communication and timer objects: 
		publisher_hw_ = this->create_publisher<std_msgs::msg::Int32>("hw",10);
		timer_hw_ = this->create_wall_timer(
 	    std::chrono::milliseconds(3000), std::bind(&HwPub::timer_hw_function, this));
		
		//--customs functions:
		

	}

	//-- communication and timer functions 
	void timer_hw_function()
	{   
		srand(time(0));
		/*your code where you publish*/
		message_.data =  rand() % 101 ;
		publisher_hw_ ->publish(message_) ;
		/*your code where you publish*/
	}


	private:
		//--rclcpp variables:

		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_hw_;
		rclcpp::TimerBase::SharedPtr timer_hw_ ;
		
		//--custom variables:
		std_msgs::msg::Int32 message_ ;
};


int main(int argc,char *argv[])
{
rclcpp::init(argc,argv)	;

auto node = std::make_shared<HwPub>();


rclcpp::spin(node);

rclcpp::shutdown();

return 0;

}
