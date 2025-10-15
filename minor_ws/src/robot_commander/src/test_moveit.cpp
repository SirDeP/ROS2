#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    // we need a new thread at this point to spin node
    // - thread with instructions to move the robot
    // - thread to spin the node
    // create single threaded executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });
    // create rest of code below 
    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");
    // Add some moves from named goals from robot.srdf

    arm.setStartStateToCurrentState();  
    arm.setNamedTarget("pose_1");
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success1) {
        arm.execute(plan1);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}