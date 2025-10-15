
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
#include "les_interfaces/msg/hardware_status.hpp"

//--custom includes 
//#include "geometry_msgs/msg/point.hpp"      //header for the standard message that is used for communication



class TemplatePublisher : public rclcpp::Node
{
	public:
	//-- constuctor: 
	TemplatePublisher() : Node("templatepublisher_node")
	{	
		//--communication and timer objects: 
		// publisher_theanswer_ = this->create_publisher<geometry_msgs::msg::Point>("the_answer",10);
		publisher_theanswer_ = this->create_publisher<les_interfaces::msg::HardwareStatus>("the_answer",10);
		timer_theanswer_ = this->create_wall_timer(
 	    std::chrono::milliseconds(1000), std::bind(&TemplatePublisher::timer_theanswer_function, this));
		
		//--customs functions:
		

	}

	//-- communication and timer functions 
	void timer_theanswer_function()
	{   

		/*your code where you publish*/
		// message_.x =  42 ;
		// message_.y =  32 ;
		// message_.z =  22 ;
		message_.temperature = 34.5;
		publisher_theanswer_ ->publish(message_) ;
		/*your code where you publish*/
	}


	private:
		//--rclcpp variables:

		// rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_theanswer_;
		rclcpp::Publisher<les_interfaces::msg::HardwareStatus>::SharedPtr publisher_theanswer_;
		rclcpp::TimerBase::SharedPtr timer_theanswer_ ;
		
		//--custom variables:
		// geometry_msgs::msg::Point message_ ;
		les_interfaces::msg::HardwareStatus message_;
};


int main(int argc,char *argv[])
{
rclcpp::init(argc,argv)	;

auto node = std::make_shared<TemplatePublisher>();


rclcpp::spin(node);

rclcpp::shutdown();

return 0;

}
