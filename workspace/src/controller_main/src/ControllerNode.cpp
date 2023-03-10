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
        this->declare_parameter("pose_topic","/turtle1/pose");
        this->declare_parameter("vel_topic","/turtle1/cmd_vel");
        this->declare_parameter("queue_size",10);
        this->declare_parameter("target_service","/target_service");
        this->declare_parameter("stop_thresholds",std::vector<double>{0.05,0.2}); // order -> {angular,distance}
        this->declare_parameter("base_speeds",std::vector<double>{0.4,1.5}); // order -> {min,max}
        this->declare_parameter("controller_gains",std::vector<double>{0.05,2.0}); // order -> {kp,kd}


        // fetch parameters
        poseTopic = this->get_parameter("pose_topic").as_string();
        velTopic = this->get_parameter("vel_topic").as_string();
        queueSize = this->get_parameter("queue_size").as_int();
        serviceName = this->get_parameter("target_service").as_string();
        std::vector<double> thresholds = this->get_parameter("stop_thresholds").as_double_array();
        angularThreshold = (float) thresholds[0];
        distanceThreshold = (float) thresholds[1];
        std::vector<double> speeds = this->get_parameter("base_speeds").as_double_array();
        minLinearSpeed = (float) speeds[0];
        maxLinearSpeed = (float) speeds[1];
        std::vector<double> gains =  this->get_parameter("controller_gains").as_double_array();
        kp = (float) speeds[0];
        kd = (float) speeds[1];

        // create callback groups
    
        // subcriber
        //auto subscriberCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        subCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        subOptions.callback_group = subCallbackGroup;

        // publisher
        //auto serviceCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        pubCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        pubOptions.callback_group = pubCallbackGroup;

        // servuce
        serviceCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // create subscriber
        poseSubscriber = this->create_subscription<turtlesim::msg::Pose>(poseTopic,queueSize,std::bind(&ControllerNode::controllerCallback,this,_1),subOptions);

        // create service
        targetService = this->create_service<controller_interfaces::srv::TargetPosition>(serviceName,std::bind(&ControllerNode::callbackService,this,_1,_2));

        // create publisher
        velPublisher = this->create_publisher<geometry_msgs::msg::Twist>(velTopic,queueSize,pubOptions);

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
            this->rotateAtan2();
            this->go2Goal();
            this->rotateFinal();
            response->response = true;
        }
        
    }

    void printCurrentPose(void){
        RCLCPP_INFO(this->get_logger(),"x: %f | y: %f | theta: %f",currentPose.x,currentPose.y,currentPose.theta);
    }

    // other
    void rotateAtan2(void){
        directionFactor = 1.0;
        this->initializeAngularError();
        while(angularError > angularThreshold){
            this->updateAngularError();
            auto cmdVel = geometry_msgs::msg::Twist();
            cmdVel.angular.z = directionFactor * (angularError * kp + (angularError-pastAngularError) * kd);
            this->velPublisher->publish(cmdVel);
        }
        auto cmdVel = geometry_msgs::msg::Twist();
        cmdVel.angular.z = 0.0;
        this->velPublisher->publish(cmdVel);
    }

    void go2Goal(void){
        this->updateDistanceError();
        this->initializeMaxScale();
        float speed;
        while(distanceError > distanceThreshold){
            this->updateDistanceError();
            auto cmdVel = geometry_msgs::msg::Twist();
            speed = distanceError * (maxLinearSpeed / maxScale);
            cmdVel.linear.x = speed > minLinearSpeed ? speed : minLinearSpeed;
            this->velPublisher->publish(cmdVel);
        }
        auto cmdVel = geometry_msgs::msg::Twist();
        cmdVel.linear.x = 0.0;
        this->velPublisher->publish(cmdVel);
    }

    void rotateFinal(void){
        directionFactor = 1.0;
        this->initializeAngularError();
        while(angularError > angularThreshold){
            this->updateAngularError2();
            auto cmdVel = geometry_msgs::msg::Twist();
            cmdVel.angular.z = directionFactor * (angularError * kp + (angularError-pastAngularError) * kd);
            this->velPublisher->publish(cmdVel);
        }
        auto cmdVel = geometry_msgs::msg::Twist();
        cmdVel.angular.z = 0.0;
        this->velPublisher->publish(cmdVel);    
    }


    // this is used to rotate towards the target position
    void updateAngularError(void){
        float xTarget,yTarget,thetaTarget;
        float xCurr,yCurr,thetaCurr;
        float xDiff, yDiff,thetaDiff;
        // target position
        xTarget = targetPose.x;
        yTarget = targetPose.y;
        // current position
        xCurr = currentPose.x;
        yCurr = currentPose.y;
        thetaCurr = currentPose.theta;
        // difference
        xDiff = xTarget-xCurr;
        yDiff = yTarget-yCurr;
        thetaTarget = atan2(yDiff,xDiff);
        thetaDiff = thetaTarget-thetaCurr;
        pastAngularError = angularError;
        angularError = abs(thetaDiff);
        this->calculateFactor(thetaDiff);
    }

    // this is used to rotate towards the final position
    void updateAngularError2(void){
        float thetaTarget,thetaCurr,thetaDiff;
        thetaCurr = currentPose.theta;
        thetaTarget = targetPose.theta;
        thetaDiff = thetaTarget-thetaCurr;
        pastAngularError = angularError;
        angularError = abs(thetaDiff);
        this->calculateFactor(thetaDiff);
    }

    void initializeAngularError(void){
        angularError = M_PI;
        pastAngularError = M_PI;
    }


    void updateDistanceError(void){
        distanceError = this->computeEuclideanDistance();
    }

    void initializeMaxScale(void){
        maxScale = this->computeEuclideanDistance();
    }

    float computeEuclideanDistance(void){
        float xTarget,yTarget;
        float xCurr,yCurr;
        float xDiff, yDiff;
        // target position
        xTarget = targetPose.x;
        yTarget = targetPose.y;
        // current position
        xCurr = currentPose.x;
        yCurr = currentPose.y;
        // difference
        xDiff = xTarget-xCurr;
        yDiff = yTarget-yCurr;
        return sqrt(pow(xDiff,2)+pow(yDiff,2));
    }

    // calculate factor
    void calculateFactor(float thetaDiff){

        if(angularError > M_PI && thetaDiff > 0){
            directionFactor = -1.0;
        }
        else if(angularError > M_PI && thetaDiff < 0){
            directionFactor = 1.0;
        }
        else if(angularError < M_PI && thetaDiff < 0){
            directionFactor = -1.0;
        }
        else{
            directionFactor = 1.0;
        }
        
    }

private:
    // subscribers, publishers & services
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr poseSubscriber;
    rclcpp::Service<controller_interfaces::srv::TargetPosition>::SharedPtr targetService;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPublisher;
    
    // important placeholders
    turtlesim::msg::Pose currentPose;
    turtlesim::msg::Pose targetPose;
    float angularError;
    float pastAngularError;
    float distanceError;
    float angularThreshold;
    float distanceThreshold;
    float directionFactor;

    // parameters
    std::string poseTopic;
    std::string serviceName;
    std::string velTopic;
    int queueSize;

    // gains
    float kp;
    float kd;

    // speed controller stuff
    float maxLinearSpeed;
    float minLinearSpeed;
    float maxScale; 

    // callback group options
    rclcpp::CallbackGroup::SharedPtr subCallbackGroup;
    rclcpp::CallbackGroup::SharedPtr pubCallbackGroup;
    rclcpp::CallbackGroup::SharedPtr serviceCallbackGroup;

    // options
    rclcpp::SubscriptionOptions subOptions;
    rclcpp::PublisherOptions pubOptions;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>(); 
    //rclcpp::spin(node);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();    
    
    rclcpp::shutdown();
    return 0;
}
