/*
--Node description: 
...what is the node doing... 
*/ 


/*
--Software changes:
one line per change 
(1) created 31.3.2025: developer-Tilmann Koster reviewer(s)-Niek Ottens 
(2) changed 01.4.2025: xxx functionality added ... : developer-Tilmann Koster reviewer(s)-Niek Ottens 
...
*/


//--general includes 
#include "rclcpp/rclcpp.hpp" // altijd nodig 

//--custom includes 
#include "rclcpp_action/rclcpp_action.hpp" // nodig  voor action server
#include "les_interfaces/action/template_a.hpp" //    
//--using 
using TemplateA = les_interfaces::action::TemplateA; // makes readeing easier 
using TemplateAGoalHandle = rclcpp_action::ServerGoalHandle<TemplateA>;  // makes reading easier 
using namespace std::placeholders;

//--Node class 
class TemplateActionserver : public rclcpp::Node
{
public:
//--constructor
    TemplateActionserver() : Node("templateactionserver_node")
    {    
         //--communication and timer objects: 
        //call back groups make thread handling possible
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        template_actionserver_ = rclcpp_action::create_server<TemplateA>(
            this,
            "template_actionserver",   
            std::bind(&TemplateActionserver::goalCallback, this, _1, _2),
            std::bind(&TemplateActionserver::cancelCallback, this, _1),
            std::bind(&TemplateActionserver::executeCallback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );

         //--customs functions:
         //...

        RCLCPP_INFO(this->get_logger(), "Action server has been started.");
    }


//-- communication and timer functions 
//...
//--customs functions:
//...

private:
    //-- communication and timer functions     
    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const TemplateA::Goal> goal)
    {
        (void)uuid;  // needed to avoid warning in compliler every sent goal gets an id 
        RCLCPP_INFO(this->get_logger(), "Received a goal");

        // your code - optional you can decide the case you want to reject a goal 
        if (goal->gname =="Tilmann") {
            RCLCPP_WARN(this->get_logger(), "Rejecting the goal,target name is Tilmann");
            return rclcpp_action::GoalResponse::REJECT;
        }
        // end of your code 
        
         
        RCLCPP_INFO(this->get_logger(), "Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

        // Any cancel request will be processed here, we can accept or reject it
    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<TemplateAGoalHandle> goal_handle)
    {
        (void)goal_handle; //avoiding compiler warning 
        RCLCPP_INFO(this->get_logger(), "Received a cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void executeCallback(const std::shared_ptr<TemplateAGoalHandle> goal_handle)
    {   
        // the goal from defined action (here TemplateA)
        std::string gname = goal_handle->get_goal()->gname; 
        
        //defines result and feedback
        auto result = std::make_shared<TemplateA::Result>();
        auto feedback = std::make_shared<TemplateA::Feedback>();
        RCLCPP_INFO(this->get_logger(), "Executing the goal");

        // your real code starts here !!!

        rclcpp::Rate loop_rate(0.5);
        int i = 0;
        
        while(i<10)
        {
            i++;
            feedback->fnumber = i;
            goal_handle->publish_feedback(feedback);
            
            //here you should handle as well when a cancel request is sent

            if(goal_handle->is_canceling()) 
            {
                RCLCPP_INFO(this->get_logger(), "Canceling goal");
                result->rnumber = i; // if you want to send something back as result
                goal_handle->canceled(result);
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "looping");
            loop_rate.sleep();

        }

            // end of your code

        result->rnumber = i;
        goal_handle->succeed(result);
    
    }
    //--rclcpp variables:
    rclcpp_action::Server<TemplateA>::SharedPtr template_actionserver_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;  

    //--custom variables:
	//...

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TemplateActionserver>();
    //MultiThreadedExecutor is important otherwise it will not work = deadlook
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}