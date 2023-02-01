#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "controller_interfaces/srv/target_position.hpp"
#include <math.h>

using std::placeholders::_1;
using std::placeholders::_2;

class ControllerNode : public rclcpp::Node 
{
public:
    ControllerNode() : Node("controller_node") 
    {
        // declare parameters
        this->declare_parameter("pose_frequency","/turtle1/pose");
        this->declare_parameter("queue_size",10);
        this->declare_parameter("target_service","/target_service");

    
        // fetch parameters
        poseTopic = this->get_parameter("pose_frequency").as_string();
        subscriberQueue = this->get_parameter("queue_size").as_int();
        serviceName = this->get_parameter("target_service").as_string();

        // create subscriber
        poseSubscriber = this->create_subscription<turtlesim::msg::Pose>(poseTopic,subscriberQueue,std::bind(&ControllerNode::controllerCallback,this,_1));

        // create service
        targetService = this->create_service<controller_interfaces::srv::TargetPosition>(serviceName,std::bind(&ControllerNode::callbackService,this,_1,_2));

        // log
        RCLCPP_INFO(this->get_logger(),"CONTROLLER NODE SERVICE ONLINE");
    }

    void controllerCallback(const turtlesim::msg::Pose::SharedPtr msg){
        currentPose.x = msg->x;
        currentPose.y = msg->y;
        currentPose.theta = msg->theta;
        //this->printCurrentPose();
    }

    void callbackService(const controller_interfaces::srv::TargetPosition::Request::SharedPtr request,const controller_interfaces::srv::TargetPosition::Response::SharedPtr response){
        // validation
        float x,y,theta;
        x = request->x;
        y = request->y;
        theta = request->theta;
        if(x > 11 || x <= 0 || y > 11 || y <= 0 || theta > M_PI || theta < 0){
            RCLCPP_ERROR(this->get_logger(),"SOME TARGET CONSTRAINT IS NOT VALID FOR THE ENVIRONMENT");
            response->response = false;
        }else{
            targetPose.x = x;
            targetPose.y = y;
            targetPose.theta = theta; 
            RCLCPP_INFO(this->get_logger(),"NEW TARGET AQUIRED -> x: %f | y: %f | theta: %f",targetPose.x,targetPose.y,targetPose.theta);
            response->response = true;
        }
        
    }

    void printCurrentPose(void){
        RCLCPP_INFO(this->get_logger(),"x: %f | y: %f | theta: %f",currentPose.x,currentPose.y,currentPose.theta);
    }

private:
    // subscribers & services
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr poseSubscriber;
    rclcpp::Service<controller_interfaces::srv::TargetPosition>::SharedPtr targetService;

    // important placeholders
    turtlesim::msg::Pose currentPose;
    turtlesim::msg::Pose targetPose;

    // parameters
    std::string poseTopic;
    std::string serviceName;
    int subscriberQueue;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
