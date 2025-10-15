/*
--Node description: 
...what the node is doing (functionally)... 
When node is created the node prints "Hello you are new to ROS2" on terminal, 
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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//--custom includes 
#include "les_interfaces/action/template_a.hpp"

//--using 
using TemplateA = les_interfaces::action::TemplateA;
using TemplateAGoalHandle = rclcpp_action::ClientGoalHandle<TemplateA>;
using namespace std::placeholders;

//--Node class 

class TemplateActionclient : public rclcpp::Node
{
public:
    //-- constuctor: 
    TemplateActionclient() : Node("template_actionclient_node")
    {   
    //--communication and timer objects:
        template_actionclient_ = rclcpp_action::create_client<TemplateA>(this, "template_actionserver");
    }
    //-- communication and timer functions 
    void sendGoal(std::string gname)
    {
        template_actionclient_->wait_for_action_server();
        auto goal = TemplateA::Goal();

        goal.gname = gname ;
    
        auto options = rclcpp_action::Client<TemplateA>::SendGoalOptions();
        options.goal_response_callback = std::bind(&TemplateActionclient::goalResponseCallback, this, _1);
        options.result_callback = std::bind(&TemplateActionclient::goalResultCallback, this, _1);
        options.feedback_callback = std::bind(&TemplateActionclient::goalFeedbackCallback, this, _1, _2);
        

        template_actionclient_->async_send_goal(goal, options);
        RCLCPP_INFO(this->get_logger(), "Sent request");
    }


      // Method to send a cancel request
    void cancelGoal()
    {
        RCLCPP_INFO(this->get_logger(), "Sent a cancel goal request");
        template_actionclient_->async_cancel_all_goals();
    }

private:
    //-- communication and timer functions 
    
    void goalResponseCallback(const TemplateAGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }

    void goalResultCallback(const TemplateAGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }
        int number = result.result->rnumber;
        RCLCPP_INFO(this->get_logger(), "Result: %d", number);
    }
 

    void goalFeedbackCallback(const TemplateAGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const TemplateA::Feedback> feedback)
    {
        (void)goal_handle;
        int number = feedback->fnumber;
        RCLCPP_INFO(this->get_logger(), "Got feedback: %d", number);
        
        // now here the option to cancel request it if 
        // feedback has certain value 
        
        // if (number >= 2) {
        //     cancelGoal();
        // }
    }
    //--customs functions:
	//...

    //--rclcpp variables:
    rclcpp_action::Client<TemplateA>::SharedPtr template_actionclient_;

};


int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TemplateActionclient>();
    std::string name = "jan";
    node->sendGoal(name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
