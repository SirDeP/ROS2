/*
Node description: what is the node doing , what are the node objects used 
*/ 


/*
Software changes (one line by change):
(1) 31.3.2025 created by Tilmann Koster 
(2) 1.4.2025 xxx functionality added ... by Tilmann Koster
...
*/




//--general includes 
#include "rclcpp/rclcpp.hpp" // altijd nodig 

//--custom includes  
#include "les_interfaces/srv/my_vec.hpp" // your own service interface =small letter, CAPs is translated to _ 

//--using 
using MyVec = les_interfaces::srv::MyVec ;

using namespace std::placeholders;

//--Node class 
class VectorServiceServer : public rclcpp::Node
{
  public :
  //-- constuctor: 
  VectorServiceServer() : Node("vectorserviceserver_node")
  {
     //--communication and timer objects: 
    template_serviceserver_ = this -> create_service<MyVec>("vectorserviceserver",
                                      std::bind(&VectorServiceServer::callBackTemplateS,this,_1,_2));
    RCLCPP_INFO(this->get_logger(), "Servcie Server started");
     //--customs functions:
  }

  //-- communication and timer functions 
  void callBackTemplateS(const MyVec::Request::SharedPtr request, const MyVec::Response::SharedPtr response)
  {
    // your code   
    response->vl = sqrt((pow(request->vx, 2) + pow(request->vy, 2) + pow(request->vz, 2)));
    //your code
  };


//--customs functions:
//... 

private :
//--rclcpp variables:
rclcpp::Service<MyVec>::SharedPtr template_serviceserver_; 

//--custom variables:
//...


};




int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);

  auto node = std::make_shared<VectorServiceServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
