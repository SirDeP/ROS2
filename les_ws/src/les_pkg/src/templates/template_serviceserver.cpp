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
#include "templates_interfaces/srv/template_s.hpp" // your own service interface =small letter, CAPs is translated to _ 

//--using 
using TemplateS = templates_interfaces::srv::TemplateS ;

using namespace std::placeholders;

//--Node class 
class TemplateServiceServer : public rclcpp::Node
{
  public :
  //-- constuctor: 
  TemplateServiceServer() : Node("templateserviceserver_node")
  {
     //--communication and timer objects: 
    template_serviceserver_ = this -> create_service<TemplateS>("templateserviceserver",
                                      std::bind(&TemplateServiceServer::callBackTemplateS,this,_1,_2));
    RCLCPP_INFO(this->get_logger(), "Servcie Server started");
     //--customs functions:
  }

  //-- communication and timer functions 
  void callBackTemplateS(const TemplateS::Request::SharedPtr request, const TemplateS::Response::SharedPtr response)
  {
    // your code   
    if(request->sname == "Tilmann") 
      { response -> snumber = 666;}
    else 
      {
       response -> snumber = 42; 
      }
    //your code
  };

//--customs functions:
//... 

private :
//--rclcpp variables:
rclcpp::Service<TemplateS>::SharedPtr template_serviceserver_; 

//--custom variables:
//...


};




int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TemplateServiceServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
