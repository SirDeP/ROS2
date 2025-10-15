/*
--Node description: 
...what the node is doing (functionally)... 
When node is created a timer is activated that will print every second a number. 
The next number is always a the old number plus one 
further it does nothing more than being alive
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
//...

//--using 
//...

class Clock : public rclcpp::Node 
{
 public: 
    //-- constuctor: 
    Clock() : Node("Clock")   /*constructor = multiple inheritance*/ 
    {
        //--communication and timer objects: 
        int64_t total_nanoseconds = 20LL * 1000000000LL + 88778567LL;
        rclcpp::sleep_for(std::chrono::nanoseconds(total_nanoseconds));
        //--customs functions:
        //... 
        auto end = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "SysTime : %f", end.seconds());
    }
};


int main(int argc,char *argv[])
{

rclcpp::init(argc,argv);

auto node = std::make_shared<Clock>();
// rclcpp::spin(node);
rclcpp::shutdown();

return 0;
}
